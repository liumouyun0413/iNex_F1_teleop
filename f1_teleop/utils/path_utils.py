import os

PACKAGE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # .../f1_teleop
_parent = os.path.dirname(PACKAGE_PATH)
# 加密发布版：包位于 <项目根>/f1_teleop_dist/f1_teleop/，项目根需再上一级（assets/data/log 仍在项目根）
PROJECT_ROOT = os.path.dirname(_parent) if os.path.basename(_parent) == "f1_teleop_dist" else _parent
ASSET_PATH = os.path.join(PROJECT_ROOT, "assets")
DEFAULT_CONFIG_PATH = os.path.join(PACKAGE_PATH, "config", "f1_teleop.yaml")
