#!/bin/bash
# 撮影用カメラ（models/presentation_camera.sdf）を world に追加し、画像を ROS へ中継する。コンテナ内で実行:
#   bash ~/final_task/scripts/sim/presentation_camera.sh add [x y z look_x look_y look_z]   # 既定は斜め前方から
#   bash ~/final_task/scripts/sim/presentation_camera.sh remove
# 画像トピック: /task/presentation/image（sensor_msgs/Image）
set -eu
WORLD=${WORLD:-default}
SDF=~/final_task/ros2_ws/src/pai_task_env/models/presentation_camera.sdf
case ${1:-add} in
  add)
    read -r X Y Z QX QY QZ QW < <(python3 - "${2:-0.52}" "${3:-0.36}" "${4:-1.36}" "${5:-0.06}" "${6:--0.04}" "${7:-1.06}" <<'PY'
import math, sys
cx, cy, cz, tx, ty, tz = map(float, sys.argv[1:])
dx, dy, dz = tx - cx, ty - cy, tz - cz
yaw = math.atan2(dy, dx); pitch = math.atan2(-dz, math.hypot(dx, dy))
cp, sp, cyw, syw = math.cos(pitch / 2), math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2)
print(cx, cy, cz, -sp * syw, sp * cyw, cp * syw, cp * cyw)
PY
)
    ign service -s /world/$WORLD/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 5000 \
      --req "sdf_filename: \"$SDF\", pose: {position: {x: $X, y: $Y, z: $Z}, orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}}"
    pkill -f "presentation/image@" || true
    nohup ros2 run ros_gz_bridge parameter_bridge '/task/presentation/image@sensor_msgs/msg/Image[ignition.msgs.Image' \
      > /tmp/presentation_bridge.log 2>&1 &
    ;;
  remove)
    pkill -f "presentation/image@" || true
    ign service -s /world/$WORLD/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 5000 \
      --req 'name: "presentation_camera", type: MODEL'
    ;;
esac
