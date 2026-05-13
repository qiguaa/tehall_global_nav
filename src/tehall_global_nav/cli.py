import os

from .core import build_arg_parser, config_from_args, run_demo


def main():
    args = build_arg_parser().parse_args()
    config = config_from_args(args)
    run_demo(config, base_dir=os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
