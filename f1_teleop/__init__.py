"""F1 轮式人形机器人 PICO 遥操作（基于 XRoboToolkit-Teleop-Sample-Python R1 Lite 范式）。"""

import os as _os

__version__ = "0.1.0"

# pyarmor 加密版 common 包位于 common_dist/common/；存在时优先于源码 common/，
# 仍以 f1_teleop.common 名字导入（pyarmor gen -O common_dist -r -i common）
_common_dist = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "common_dist")
if _os.path.isdir(_os.path.join(_common_dist, "common")):
    __path__.insert(0, _common_dist)
