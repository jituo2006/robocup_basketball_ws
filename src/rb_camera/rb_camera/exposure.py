"""Driver exposure capabilities; values use the camera's native V4L2 units."""
import re


def exposure_capabilities(text):
    for name in ("exposure_time_absolute", "exposure_absolute"):
        match = re.search(r"^\s*" + name + r"\s+0x[0-9a-fA-F]+.*$", text, re.M)
        if not match:
            continue
        fields = dict((k, int(v)) for k, v in
                      re.findall(r"\b(min|max|step|default|value)=(-?\d+)", match[0]))
        if not {"min", "max", "value"} <= fields.keys():
            continue
        fields.setdefault("step", 1)
        if fields["step"] <= 0 or fields["max"] < fields["min"]:
            continue
        return dict(fields, control=name)
    return None


def validate_exposure(value, caps):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("曝光值必须为整数")
    if not caps:
        raise ValueError("摄像头未报告曝光范围")
    if not caps["min"] <= value <= caps["max"]:
        raise ValueError(f"曝光范围为 {caps['min']}–{caps['max']}")
    if (value - caps["min"]) % caps["step"]:
        raise ValueError(f"曝光步长为 {caps['step']}")
    return value
