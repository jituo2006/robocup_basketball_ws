#!/usr/bin/env python3
"""Read-only competition rule check; does not initialize ROS or command hardware."""
from pathlib import Path
import argparse
import sys
import yaml

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src" / "rb_mission"))
from rb_mission.match_rules import competition_gaps, validate_match_config  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=WS / "src/rb_mission/config/mission.yaml")
    args = parser.parse_args(argv)
    try:
        cfg = yaml.safe_load(args.config.read_text())
        validate_match_config(cfg)
    except (OSError, ValueError, TypeError, AttributeError, yaml.YAMLError) as exc:
        print(f"配置校验失败：{exc}")
        return 2
    print("已确认的软件配置：启动延迟 5–15s、每任务 <=2 次动作、持球上限 <=2、双球角色")
    gaps = competition_gaps(cfg)
    for item in gaps:
        print(f"待完成：{item}")
    print("上述事项未完成前，只能调试，不能声明已通过参赛验收。")
    return 1 if gaps else 0


if __name__ == "__main__":
    raise SystemExit(main())
