#!/bin/bash
# Gazebo GUI の 3D ビューの視点を設定する（動画撮影用。world 座標）。コンテナ内で実行:
#   bash ~/final_task/scripts/sim/gui_view.sh [x y z 注視x 注視y 注視z]
# 撮影用カメラセンサー（presentation_camera.sh）はソフトウェア描画でシミュレーションを大きく遅くするため、
# 動画は GUI の 3D ビューを画面録画する（GUI の描画はシミュレーションの進行に影響しない）
set -eu
read -r X Y Z QX QY QZ QW < <(python3 - "${1:-0.22}" "${2:-0.14}" "${3:-1.17}" "${4:-0.06}" "${5:--0.05}" "${6:-1.04}" <<'PY'
import math, sys
cx, cy, cz, tx, ty, tz = map(float, sys.argv[1:])
dx, dy, dz = tx - cx, ty - cy, tz - cz
yaw = math.atan2(dy, dx); pitch = math.atan2(-dz, math.hypot(dx, dy))
cp, sp, cyw, syw = math.cos(pitch / 2), math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2)
print(cx, cy, cz, -sp * syw, sp * cyw, cp * syw, cp * cyw)
PY
)
ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean --timeout 3000 \
  --req "pose: {position: {x: $X, y: $Y, z: $Z}, orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}}" > /dev/null
