"""2026 revised rules: configuration checks and observation-based scoring.

Source: supplied September revision, PDF pages 7, 9, 10 and 12.
Scores require observed outcomes; a successful ROS service is not a scored shot.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


def validate_match_config(cfg: dict) -> None:
    """Reject settings that would violate explicit software constraints."""
    delay = cfg.get("timeouts", {}).get("start_delay_s", 8.0)
    if isinstance(delay, bool) or not isinstance(delay, (int, float)):
        raise ValueError("start_delay_s 必须为数值")
    if not math.isfinite(delay) or not 5.0 <= delay <= 15.0:
        raise ValueError("§2.3-7：start_delay_s 必须在 5–15 秒内")
    rules = cfg.get("rules", {})
    for name in ("actions_per_mission", "max_ball_count"):
        value = rules.get(name, 2)
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 2:
            raise ValueError(f"§2.5：{name} 必须为 1 或 2")
    expected = {
        "PASS": ("ball_basketball", "ball_volleyball"),
        "SHOOT": ("ball_volleyball", "ball_basketball"),
    }
    labels = cfg.get("mission_labels", {})
    for mission, (ball, interference) in expected.items():
        entry = labels.get(mission, {})
        if entry.get("ball") != ball or entry.get("interference") != interference:
            raise ValueError(f"§2.5：{mission} 的目标球/干扰球类别配置错误")


def competition_gaps(cfg: dict) -> list[str]:
    """List implementation/evidence gaps; declarations do not certify hardware."""
    gaps = []
    try:
        validate_match_config(cfg)
    except (ValueError, TypeError, AttributeError) as exc:
        gaps.append(str(exc))
    if not cfg.get("avoidance", {}).get("enabled", False):
        gaps.append("§2.2/2.5：视觉避障当前关闭；不能按赛场模式运行")
    # These limitations cannot be cleared merely by editing a confirmation flag.
    gaps.extend([
        "§2.4：今年场地示意图与实测坐标尚未核对，现有动作点含旧值/占位值",
        "§2.5：尚无承重轮轮廓与出发区多边形的回位验收，中心到点不足以证明回位得分",
        "§2.5：现有持球 Bool/软件计数不能证明真实持球数 <= 2，需持球计数传感器证据",
        "§2.3-6：软件急停已有，但自主碰撞风险检测与红色硬件断动力急停未验收",
        "§2.2：决赛移动篮筐的跟踪、时延和投篮策略未实测",
        "§2.3-5/2.4-6：现场须本机触发并禁用无线/遥控，SSH 仅用于调试",
    ])
    return gaps


@dataclass(frozen=True)
class ObservedAction:
    released: bool = False
    in_pass_zone: bool = False
    outside_shoot_line: bool = False
    pass_target: int = 0       # 0=miss, 1/2/3=observed target area
    shot_result: str = "miss"  # miss / contact (rim or backboard) / basket
    outside_three_point: bool = False


def score_round(mission: str, actions: list[ObservedAction], *, entered=False,
                returned=False, remaining_s=0.0, interference_contacts=0) -> int:
    """PDF p10 scoring, using observations supplied by a referee/review process.

    No runtime recognition of baskets or official round duration is assumed.
    `returned` means the supporting wheels are verified inside the start area.
    """
    if mission not in ("PASS", "SHOOT"):
        raise ValueError("mission 必须为 PASS 或 SHOOT")
    if len(actions) > 2:
        raise ValueError("每回合最多记录两次动作")
    if not math.isfinite(remaining_s) or remaining_s < 0:
        raise ValueError("remaining_s 必须为有限非负数")
    if (isinstance(interference_contacts, bool) or not isinstance(interference_contacts, int)
            or interference_contacts < 0):
        raise ValueError("interference_contacts 必须为非负整数")
    score = 5 if entered else 0
    for action in actions:
        if action.pass_target not in (0, 1, 2, 3):
            raise ValueError("pass_target 必须为 0/1/2/3")
        if action.shot_result not in ("miss", "contact", "basket"):
            raise ValueError("shot_result 必须为 miss/contact/basket")
        if not action.released:
            continue
        if mission == "PASS":
            score += 10 if action.in_pass_zone else 5
            score += (0, 10, 30, 50)[action.pass_target]
        else:
            score += 10 if action.outside_shoot_line else 5
            score += (30 if action.outside_three_point else 10) if action.shot_result == "basket" \
                else (5 if action.shot_result == "contact" else 0)
    if returned and sum(a.released for a in actions) == 2:
        score += 5 if mission == "PASS" else 15
        score += min(10, math.floor(remaining_s / (5 if mission == "PASS" else 2)))
    return score - 2 * interference_contacts
