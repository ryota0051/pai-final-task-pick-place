#!/bin/bash
# 指定した物体位置で方策を評価し、試行中のトピックを rosbag に記録する（失敗の原因解析用）。コンテナ内で実行:
#   bash ~/final_task/scripts/sim/eval_positions.sh <label> <infer_config> "<x0,y0,x1,y1,...>" [evaluator の追加引数...]
#   例: ... "0.13,-0.01" -p observe.enabled:=true
# 出力: ~/dataset/eval/<label>_<日時>.csv、rosbag は ~/dataset/eval_bags/<label>
set -u
LABEL=$1; CFG=$2; POS=$3; shift 3
EVAL_CFG=~/final_task/ros2_ws/src/pai_task_env/config/policy_evaluator.yaml
BAG_DIR=~/dataset/eval_bags/$LABEL
TOPICS="/joint_states /task/object_pose /task/status /policy/status \
  /crane_plus_arm_controller/joint_trajectory /crane_plus_gripper_controller/joint_trajectory"

pkill -f scripts/infer.py; sleep 2
(cd ~/pai_training && nohup pixi run --frozen infer --config "$CFG" > /tmp/infer_$LABEL.log 2>&1 &)
# 推論ノードの起動を待つ（固定時間だと GPU を他と共有しているときに間に合わないことがある）
for _ in $(seq 1 60); do ros2 service list 2>/dev/null | grep -q /policy/enable && break; sleep 2; done
rm -rf "$BAG_DIR"; mkdir -p "$(dirname "$BAG_DIR")"
ros2 bag record -o "$BAG_DIR" $TOPICS > /tmp/bag_$LABEL.log 2>&1 &
BAG_PID=$!
sleep 2
ros2 run pai_task_env policy_evaluator --ros-args --params-file "$EVAL_CFG" \
  -p label:=$LABEL -p "positions:=[$POS]" "$@" 2>&1 | grep -E "trial|summary" | sed "s/.*\] //"
kill -INT $BAG_PID; wait $BAG_PID 2>/dev/null
pkill -f scripts/infer.py
