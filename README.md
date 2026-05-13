# tehall-global-nav

目录：

- `src/tehall_global_nav/`: 库代码
- `src/tehall_global_nav/lcm_types/`: 独立带上的 LCM Python 类型
- `global_nav_demo.py`: 脚本兼容入口

安装：

```bash
cd /home/daoshi1593/cankao/tehall_global_nav
pip install -e .
```

命令行：

```bash
python3 global_nav_demo.py --target 1.0 0.0
tehall-global-nav --target 1.0 0.0 90 --target-frame sim
```

Python API：

```python
from tehall_global_nav import DemoConfig, run_demo

run_demo(
    DemoConfig(
        targets=[(1.0, 0.0, None)],
        target_frame="global",
    )
)
```
