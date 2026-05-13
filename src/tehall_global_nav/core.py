import argparse
import fcntl
import json
import math
import os
import signal
import struct
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from statistics import median
from typing import Iterable, List, Optional, Sequence, Tuple

import lcm

from .lcm_types import robot_control_cmd_lcmt, robot_control_response_lcmt


Target = Tuple[float, float, Optional[float]]
LOCK_PATH = "/tmp/tehall_main.lock"
CMD_URL = "udpm://239.255.76.67:7671?ttl=255"
CMD_CHANNEL = "robot_control_cmd"
RESP_URL = "udpm://239.255.76.67:7670?ttl=255"
RESP_CHANNEL = "robot_control_response"
POSE_URL = "udpm://239.255.76.67:7667?ttl=255"
GLOBAL_CHANNEL = "global_to_robot"
SIM_CHANNEL = "simulator_state"

_UINT64_MASK = 0xFFFFFFFFFFFFFFFF
_BASE_HASH = 0x7E246F0371A27D89
LOCALIZATION_FINGERPRINT = struct.pack(
    ">Q",
    (((_BASE_HASH << 1) & _UINT64_MASK) + (_BASE_HASH >> 63)) & _UINT64_MASK,
)
SIM_FINGERPRINT = bytes.fromhex("fafded315b37d41e")
ALIGN_SAMPLES = 15
ALIGN_TIMEOUT = 3.0
ALIGN_MAX_AGE = 0.25
ALIGN_MAX_DT = 0.08
UNSAFE_SWITCH_STATUS = {2, 3, 4, 5, 6, 7}


@dataclass
class DemoConfig:
    targets: List[Target] = field(default_factory=list)
    target_frame: str = "global"
    gait_id: int = 27
    rpy: Sequence[float] = (0.0, -0.01, 0.0)
    step_height: Sequence[float] = (0.06, 0.06)
    tolerance: float = 0.03
    kp: float = 0.20
    min_vel: float = 0.05
    near_min_vel: float = 0.018
    max_vel: float = 0.25
    slow_down_distance: float = 0.45
    min_vel_release_distance: float = 0.12
    yaw_tolerance_deg: float = 5.0
    yaw_kp: float = 0.9
    yaw_min_speed: float = 0.05
    yaw_max_speed: float = 0.25
    pose_timeout: float = 5.0
    pose_max_age: float = 0.5
    target_timeout: float = 60.0
    yaw_timeout: float = 20.0
    send_interval: float = 0.08
    cmd_duration: float = 0.12
    prepare_sec: float = 0.4
    stand_sec: float = 1.0
    stop_sec: float = 0.5
    skip_stand: bool = False
    ignore_unsafe: bool = False
    log_file: Optional[str] = None


def norm(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def rounded(value, digits=4):
    return None if value is None else round(float(value), digits)


def brief_pose(pose):
    if pose is None:
        return None
    return {
        "x": rounded(pose["x"], 4),
        "y": rounded(pose["y"], 4),
        "yaw_deg": rounded(math.degrees(pose["yaw"]), 2),
        "age_ms": rounded((time.time() - pose["recv"]) * 1000.0, 1),
    }


def to_body(dx, dy, yaw):
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy


def circular_mean(values):
    return math.atan2(sum(math.sin(v) for v in values), sum(math.cos(v) for v in values))


def decode_global_pose(data):
    if len(data) < 44 or data[:8] != LOCALIZATION_FINGERPRINT:
        raise ValueError("bad global_to_robot packet")
    x, y = struct.unpack_from(">2f", data, 8)
    yaw = struct.unpack_from(">3f", data, 32)[2]
    return {"x": float(x), "y": float(y), "yaw": float(yaw), "recv": time.time()}


def decode_sim_pose(data):
    if len(data) < 248 or data[:8] != SIM_FINGERPRINT:
        raise ValueError("bad simulator_state packet")
    yaw = struct.unpack_from(">3d", data, 32)[2]
    x, y = struct.unpack_from(">2d", data, 224)
    return {"x": float(x), "y": float(y), "yaw": float(yaw), "recv": time.time()}


def decode_response(data):
    msg = robot_control_response_lcmt.decode(data)
    return {
        "switch_status": int(getattr(msg, "switch_status", 0)),
        "ori_error": int(getattr(msg, "ori_error", 0)),
        "footpos_error": int(getattr(msg, "footpos_error", 0)),
        "recv": time.time(),
    }


class Logger:
    def __init__(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.file = open(path, "a", encoding="utf-8", buffering=1)
        self.lock = threading.Lock()

    def log(self, event, **fields):
        line = json.dumps(
            {
                "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                "event": event,
                **fields,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        with self.lock:
            print(line, flush=True)
            self.file.write(line + "\n")

    def close(self):
        with self.lock:
            self.file.close()


class Monitor:
    def __init__(self, url, channel, decode, logger, stop_event):
        self.channel = channel
        self.decode = decode
        self.logger = logger
        self.stop_event = stop_event
        self.lc = lcm.LCM(url)
        self.lock = threading.Lock()
        self.latest = None
        self.last_error_log = 0.0
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.lc.subscribe(self.channel, self._callback)
        self.thread.start()
        self.logger.log("monitor_start", channel=self.channel)

    def _callback(self, channel, data):
        try:
            item = self.decode(data)
        except Exception as exc:
            now = time.time()
            if now - self.last_error_log > 1.0:
                self.logger.log("decode_error", channel=channel, error=repr(exc))
                self.last_error_log = now
            return
        with self.lock:
            self.latest = item

    def _loop(self):
        while not self.stop_event.is_set():
            try:
                self.lc.handle_timeout(20)
            except Exception as exc:
                self.logger.log("monitor_error", channel=self.channel, error=repr(exc))
                time.sleep(0.05)

    def get(self):
        with self.lock:
            return None if self.latest is None else dict(self.latest)

    def wait(self, timeout_sec):
        deadline = time.time() + timeout_sec
        while not self.stop_event.is_set() and time.time() < deadline:
            item = self.get()
            if item is not None:
                return item
            time.sleep(0.02)
        return None

    def join(self):
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)


class Publisher:
    def __init__(self, logger, gait_id, rpy, step_height, cmd_url=CMD_URL, cmd_channel=CMD_CHANNEL):
        self.logger = logger
        self.gait_id = int(gait_id)
        self.rpy = list(rpy)
        self.step_height = list(step_height)
        self.cmd_channel = cmd_channel
        self.lc = lcm.LCM(cmd_url)
        self.life_count = 0
        self.lock = threading.Lock()

    def send(
        self,
        mode,
        vel=(0.0, 0.0, 0.0),
        duration_ms=0,
        reason="cmd",
        gait_id=None,
        rpy=None,
        step_height=None,
    ):
        msg = robot_control_cmd_lcmt()
        msg.mode = int(mode)
        msg.gait_id = self.gait_id if gait_id is None else int(gait_id)
        msg.contact = 0
        with self.lock:
            self.life_count = (self.life_count + 1) % 128
            msg.life_count = self.life_count
        msg.vel_des = list(vel)
        msg.rpy_des = self.rpy if rpy is None else list(rpy)
        msg.step_height = self.step_height if step_height is None else list(step_height)
        msg.duration = int(duration_ms)
        self.lc.publish(self.cmd_channel, msg.encode())
        self.logger.log(
            "cmd",
            reason=reason,
            mode=msg.mode,
            gait_id=msg.gait_id,
            vel=[rounded(v, 4) for v in msg.vel_des],
            duration_ms=msg.duration,
        )


class Demo:
    def __init__(self, config: DemoConfig, logger, stop_event):
        self.args = config
        self.logger = logger
        self.stop_event = stop_event
        self.pose = Monitor(POSE_URL, GLOBAL_CHANNEL, decode_global_pose, logger, stop_event)
        self.sim = (
            Monitor(POSE_URL, SIM_CHANNEL, decode_sim_pose, logger, stop_event)
            if config.target_frame == "sim"
            else None
        )
        self.response = (
            None
            if config.ignore_unsafe
            else Monitor(RESP_URL, RESP_CHANNEL, decode_response, logger, stop_event)
        )
        self.pub = Publisher(logger, config.gait_id, config.rpy, config.step_height)

    def start(self):
        self.pose.start()
        if self.sim is not None:
            self.sim.start()
        if self.response is not None:
            self.response.start()

    def join(self):
        self.pose.join()
        if self.sim is not None:
            self.sim.join()
        if self.response is not None:
            self.response.join()

    def send_for(self, seconds, **kwargs):
        deadline = time.time() + max(seconds, 0.0)
        while not self.stop_event.is_set() and time.time() < deadline:
            self.pub.send(**kwargs)
            time.sleep(self.args.send_interval)

    def fresh_pose(self):
        pose = self.pose.get()
        if pose is None:
            return None
        if time.time() - pose["recv"] > self.args.pose_max_age:
            return None
        return pose

    def unsafe(self):
        if self.response is None:
            return False
        status = self.response.get()
        if not status:
            return False
        bad = (
            status["switch_status"] in UNSAFE_SWITCH_STATUS
            or status["ori_error"] != 0
            or status["footpos_error"] != 0
        )
        if bad:
            self.logger.log("unsafe", response=status)
        return bad

    def wait_ready(self):
        pose = self.pose.wait(self.args.pose_timeout)
        if pose is None:
            raise RuntimeError(f"{self.args.pose_timeout:.1f}s 内没有收到 {GLOBAL_CHANNEL}")
        sim_pose = self.sim.wait(1.0) if self.sim is not None else None
        if self.sim is not None and sim_pose is None:
            raise RuntimeError("使用 --target-frame sim 前没有收到 simulator_state 位姿")
        self.logger.log("initial_pose", pose=brief_pose(pose), sim_pose=brief_pose(sim_pose))

    def prepare_robot(self):
        if self.args.skip_stand:
            return
        zeros = [0.0, 0.0, 0.0]
        self.send_for(self.args.prepare_sec, mode=7, reason="prepare", gait_id=0, rpy=zeros, step_height=[0.0, 0.0])
        self.send_for(self.args.stand_sec, mode=12, reason="stand", gait_id=0, rpy=zeros, step_height=[0.0, 0.0])

    def stop_motion(self, reason="stop"):
        zeros = [0.0, 0.0, 0.0]
        self.send_for(self.args.stop_sec, mode=12, reason=reason, gait_id=0, rpy=zeros, step_height=[0.0, 0.0])
        self.pub.send(mode=12, reason=f"{reason}_final", gait_id=0, rpy=zeros, step_height=[0.0, 0.0])

    def sample_alignment(self):
        samples = []
        last_pair = None
        deadline = time.time() + ALIGN_TIMEOUT
        while not self.stop_event.is_set() and len(samples) < ALIGN_SAMPLES and time.time() < deadline:
            sim_pose = self.sim.get()
            global_pose = self.pose.get()
            now = time.time()
            if sim_pose and global_pose:
                pair = (sim_pose["recv"], global_pose["recv"])
                if (
                    pair != last_pair
                    and now - sim_pose["recv"] <= ALIGN_MAX_AGE
                    and now - global_pose["recv"] <= ALIGN_MAX_AGE
                    and abs(sim_pose["recv"] - global_pose["recv"]) <= ALIGN_MAX_DT
                ):
                    samples.append(
                        (
                            global_pose["x"] - sim_pose["x"],
                            global_pose["y"] - sim_pose["y"],
                            norm(global_pose["yaw"] - sim_pose["yaw"]),
                        )
                    )
                    last_pair = pair
            time.sleep(0.02)
        if len(samples) < 3:
            raise RuntimeError(f"simulator_state -> global_to_robot 标定失败: {len(samples)} samples")
        dx, dy, dyaw = zip(*samples)
        result = {"dx": median(dx), "dy": median(dy), "dyaw": circular_mean(dyaw)}
        self.logger.log(
            "sim_to_global",
            dx=rounded(result["dx"], 4),
            dy=rounded(result["dy"], 4),
            dyaw_deg=rounded(math.degrees(result["dyaw"]), 2),
            samples=len(samples),
        )
        return result

    def resolve_target(self, target):
        x, y, yaw_deg = target
        if self.args.target_frame == "global":
            return x, y, yaw_deg
        align = self.sample_alignment()
        resolved_yaw = None if yaw_deg is None else math.degrees(norm(math.radians(yaw_deg) + align["dyaw"]))
        resolved = (x + align["dx"], y + align["dy"], resolved_yaw)
        self.logger.log(
            "target_resolved",
            input_target={"x": rounded(x, 4), "y": rounded(y, 4), "yaw_deg": rounded(yaw_deg, 2)},
            resolved_target={"x": rounded(resolved[0], 4), "y": rounded(resolved[1], 4), "yaw_deg": rounded(resolved[2], 2)},
        )
        return resolved

    def speed_for_error(self, error):
        min_vel = self.args.min_vel if error > self.args.min_vel_release_distance else min(self.args.min_vel, self.args.near_min_vel)
        speed = clamp(self.args.kp * error, min_vel, self.args.max_vel)
        if error < self.args.slow_down_distance:
            span = max(self.args.slow_down_distance - self.args.tolerance, 1e-6)
            speed = min(speed, self.args.max_vel * clamp((error - self.args.tolerance) / span, 0.0, 1.0))
        return speed

    def align_yaw(self, target_yaw, index):
        deadline = time.time() + self.args.yaw_timeout
        last_send = 0.0
        reason = "stopped"
        while not self.stop_event.is_set():
            if time.time() > deadline:
                reason = "timeout"
                break
            if self.unsafe():
                reason = "unsafe"
                break
            pose = self.fresh_pose()
            if pose is None:
                time.sleep(0.02)
                continue
            yaw_error = norm(target_yaw - pose["yaw"])
            if abs(math.degrees(yaw_error)) <= self.args.yaw_tolerance_deg:
                reason = "target"
                break
            yaw_speed = clamp(self.args.yaw_kp * yaw_error, -self.args.yaw_max_speed, self.args.yaw_max_speed)
            if abs(yaw_speed) < self.args.yaw_min_speed:
                yaw_speed = math.copysign(self.args.yaw_min_speed, yaw_speed)
            if time.time() - last_send >= self.args.send_interval:
                self.pub.send(
                    mode=11,
                    vel=(0.0, 0.0, yaw_speed),
                    duration_ms=int(max(self.args.cmd_duration, self.args.send_interval) * 1000),
                    reason=f"target_{index}_yaw",
                )
                self.logger.log(
                    "tick",
                    index=index,
                    phase="yaw",
                    yaw_error_deg=rounded(math.degrees(yaw_error), 2),
                    yaw_speed_radps=rounded(yaw_speed, 4),
                    pose=brief_pose(pose),
                )
                last_send = time.time()
            time.sleep(0.02)
        self.stop_motion(reason=f"target_{index}_yaw_{reason}")
        return reason

    def walk_to_target(self, target, index):
        target_x, target_y, target_yaw_deg = self.resolve_target(target)
        target_yaw = None if target_yaw_deg is None else math.radians(target_yaw_deg)
        start_pose = self.fresh_pose() or self.pose.wait(self.args.pose_timeout)
        if start_pose is None:
            raise RuntimeError("开始导航前没有 fresh global_to_robot 位姿")
        self.logger.log(
            "target_start",
            index=index,
            target={"x": rounded(target_x, 4), "y": rounded(target_y, 4), "yaw_deg": rounded(target_yaw_deg, 2)},
            start_pose=brief_pose(start_pose),
        )

        deadline = time.time() + self.args.target_timeout
        last_send = 0.0
        reason = "stopped"
        while not self.stop_event.is_set():
            if time.time() > deadline:
                reason = "timeout"
                break
            if self.unsafe():
                reason = "unsafe"
                break
            pose = self.fresh_pose()
            if pose is None:
                time.sleep(0.02)
                continue
            dx = target_x - pose["x"]
            dy = target_y - pose["y"]
            error = math.hypot(dx, dy)
            if error <= self.args.tolerance:
                reason = "target"
                break
            speed = self.speed_for_error(error)
            body_x, body_y = to_body(dx, dy, pose["yaw"])
            scale = speed / max(error, 1e-6)
            vel = [body_x * scale, body_y * scale, 0.0]
            if time.time() - last_send >= self.args.send_interval:
                self.pub.send(
                    mode=11,
                    vel=vel,
                    duration_ms=int(max(self.args.cmd_duration, self.args.send_interval) * 1000),
                    reason=f"target_{index}_xy",
                )
                self.logger.log(
                    "tick",
                    index=index,
                    phase="xy",
                    error_m=rounded(error, 4),
                    vel=[rounded(v, 4) for v in vel],
                    pose=brief_pose(pose),
                )
                last_send = time.time()
            time.sleep(0.02)

        self.stop_motion(reason=f"target_{index}_{reason}")
        yaw_reason = "skipped"
        if reason == "target" and target_yaw is not None and not self.stop_event.is_set():
            yaw_reason = self.align_yaw(target_yaw, index)

        final_pose = self.pose.get()
        final_error = None if final_pose is None else math.hypot(target_x - final_pose["x"], target_y - final_pose["y"])
        final_yaw_error = None
        if final_pose is not None and target_yaw is not None:
            final_yaw_error = math.degrees(norm(target_yaw - final_pose["yaw"]))
        self.logger.log(
            "target_done",
            index=index,
            reason=reason,
            yaw_reason=yaw_reason,
            final_error_m=rounded(final_error, 4),
            final_yaw_error_deg=rounded(final_yaw_error, 2),
            final_pose=brief_pose(final_pose),
        )
        return reason == "target" and yaw_reason not in {"timeout", "unsafe"}

    def run(self, targets):
        self.wait_ready()
        self.prepare_robot()
        for index, target in enumerate(targets, start=1):
            if self.stop_event.is_set() or not self.walk_to_target(target, index):
                break


def parse_targets(raw_targets: Optional[Iterable[Sequence[object]]]) -> List[Target]:
    if not raw_targets:
        return []
    targets = []
    for raw in raw_targets:
        if len(raw) not in (2, 3):
            raise ValueError(f"--target 需要 2 或 3 个数值: {raw}")
        x, y = float(raw[0]), float(raw[1])
        yaw = None if len(raw) == 2 or raw[2] is None else float(raw[2])
        targets.append((x, y, yaw))
    return targets


def build_arg_parser():
    parser = argparse.ArgumentParser(description="最小绝对坐标导航 demo")
    parser.add_argument("--target", action="append", nargs="+", metavar="VALUE", help="X Y [YAW_DEG]，可重复")
    parser.add_argument("--target-frame", choices=("global", "sim"), default="global")
    parser.add_argument("--gait-id", type=int, default=27)
    parser.add_argument("--rpy", type=float, nargs=3, default=[0.0, -0.01, 0.0])
    parser.add_argument("--step-height", type=float, nargs=2, default=[0.06, 0.06])
    parser.add_argument("--tolerance", type=float, default=0.03)
    parser.add_argument("--kp", type=float, default=0.20)
    parser.add_argument("--min-vel", type=float, default=0.05)
    parser.add_argument("--near-min-vel", type=float, default=0.018)
    parser.add_argument("--max-vel", type=float, default=0.25)
    parser.add_argument("--slow-down-distance", type=float, default=0.45)
    parser.add_argument("--min-vel-release-distance", type=float, default=0.12)
    parser.add_argument("--yaw-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--yaw-kp", type=float, default=0.9)
    parser.add_argument("--yaw-min-speed", type=float, default=0.05)
    parser.add_argument("--yaw-max-speed", type=float, default=0.25)
    parser.add_argument("--pose-timeout", type=float, default=5.0)
    parser.add_argument("--pose-max-age", type=float, default=0.5)
    parser.add_argument("--target-timeout", type=float, default=60.0)
    parser.add_argument("--yaw-timeout", type=float, default=20.0)
    parser.add_argument("--send-interval", type=float, default=0.08)
    parser.add_argument("--cmd-duration", type=float, default=0.12)
    parser.add_argument("--prepare-sec", type=float, default=0.4)
    parser.add_argument("--stand-sec", type=float, default=1.0)
    parser.add_argument("--stop-sec", type=float, default=0.5)
    parser.add_argument("--skip-stand", action="store_true")
    parser.add_argument("--ignore-unsafe", action="store_true")
    parser.add_argument("--log-file", default=None)
    return parser


def default_log_file(base_dir):
    log_dir = os.path.join(base_dir, "logs")
    return os.path.join(log_dir, f"global_nav_demo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl")


def acquire_lock():
    fd = open(LOCK_PATH, "w")
    try:
        fcntl.flock(fd.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError(f"检测到已有控制程序在运行: {LOCK_PATH}") from exc
    fd.write(f"{os.getpid()}\n")
    fd.flush()
    return fd


def run_demo(config: DemoConfig, *, base_dir: Optional[str] = None):
    if not config.targets:
        raise SystemExit("没有目标点。用 --target X Y [YAW_DEG] 传入，或者在 DemoConfig.targets 里填写。")

    root_dir = base_dir or os.getcwd()
    log_path = config.log_file or default_log_file(root_dir)
    logger = Logger(log_path)
    stop_event = threading.Event()
    lock_fd = None
    demo = Demo(config, logger, stop_event)
    started = False

    def handle_signal(signum, _frame):
        logger.log("signal", signum=int(signum))
        stop_event.set()

    old_int = signal.getsignal(signal.SIGINT)
    old_term = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        lock_fd = acquire_lock()
        logger.log(
            "demo_start",
            log_file=log_path,
            target_frame=config.target_frame,
            targets=[{"x": x, "y": y, "yaw_deg": yaw} for x, y, yaw in config.targets],
        )
        demo.start()
        started = True
        demo.run(config.targets)
        logger.log("demo_done")
    except Exception as exc:
        logger.log("demo_error", error=repr(exc))
        raise
    finally:
        stop_event.set()
        if started:
            try:
                demo.stop_motion(reason="finally_stop")
            except Exception as exc:
                logger.log("finally_stop_error", error=repr(exc))
            demo.join()
        if lock_fd is not None:
            fcntl.flock(lock_fd.fileno(), fcntl.LOCK_UN)
            lock_fd.close()
        logger.log("demo_exit", log_file=log_path)
        logger.close()
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)


def config_from_args(args):
    return DemoConfig(
        targets=parse_targets(args.target),
        target_frame=args.target_frame,
        gait_id=args.gait_id,
        rpy=args.rpy,
        step_height=args.step_height,
        tolerance=args.tolerance,
        kp=args.kp,
        min_vel=args.min_vel,
        near_min_vel=args.near_min_vel,
        max_vel=args.max_vel,
        slow_down_distance=args.slow_down_distance,
        min_vel_release_distance=args.min_vel_release_distance,
        yaw_tolerance_deg=args.yaw_tolerance_deg,
        yaw_kp=args.yaw_kp,
        yaw_min_speed=args.yaw_min_speed,
        yaw_max_speed=args.yaw_max_speed,
        pose_timeout=args.pose_timeout,
        pose_max_age=args.pose_max_age,
        target_timeout=args.target_timeout,
        yaw_timeout=args.yaw_timeout,
        send_interval=args.send_interval,
        cmd_duration=args.cmd_duration,
        prepare_sec=args.prepare_sec,
        stand_sec=args.stand_sec,
        stop_sec=args.stop_sec,
        skip_stand=args.skip_stand,
        ignore_unsafe=args.ignore_unsafe,
        log_file=args.log_file,
    )
