"""Run on the car to update shared steering calibration; does not start ROS."""
from pathlib import Path
import re
import shutil
from datetime import datetime


def updated(text):
    center = re.findall(r"(?m)^\s*servo_center_pwm:\s*([0-9.]+)", text)
    if len(center) != 1 or float(center[0]) != 1489.0:
        raise ValueError("Expected servo_center_pwm=1489; configuration left unchanged")
    text = text.replace(
        "# 用户观察左右各偏移 190 已接近最大偏角，采用 1299～1679；实际前轮角度待测。",
        "# 用户指定 PWM 范围 1228～1750，中值两侧各 261；实际前轮角度待测。",
    )
    for key, value in {
        "servo_left_pwm": "1750.0",
        "servo_right_pwm": "1228.0",
        "yaw_correction_limit_deg": "15.0",
    }.items():
        pattern = rf"(?m)^(\s*{key}:)[ \t]*[^\r\n#]*(?P<comment>#[^\r\n]*)?$"
        text, count = re.subn(
            pattern,
            lambda m: m[1] + " " + value + ("  " + m["comment"] if m["comment"] else ""),
            text,
        )
        if count != 1:
            raise ValueError(f"Expected one {key}, found {count}; configuration left unchanged")
    return text


def main():
    root = Path.home() / "racecar"
    paths = [
        root / "src/racecar/config/driver_calibration.yaml",
        root / "install/racecar/share/racecar/config/driver_calibration.yaml",
    ]
    # Resolve symlink installs and validate both configurations before writing.
    targets = list(dict.fromkeys(p.resolve(strict=True) for p in paths))
    changes = [(p, updated(p.read_text(encoding="utf-8"))) for p in targets]
    suffix = datetime.now().strftime(".before-servo-1750-%Y%m%d-%H%M%S-%f")
    for path, _ in changes:
        shutil.copy2(path, str(path) + suffix)
    for path, content in changes:
        path.write_text(content, encoding="utf-8")
        print(f"Saved: {path}")
    print("servo_center_pwm=1489, servo_left_pwm=1750, servo_right_pwm=1228")
    print("yaw_correction_limit_deg=15; total steering angle limits unchanged")
    print("Saved only. Restart the driver before expecting the new values to take effect.")


if __name__ == "__main__":
    main()
