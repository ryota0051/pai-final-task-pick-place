#!/bin/bash
# 置き場所もランダムに変えたスクリプトデモを収録する（実験12）。コンテナ内で実行:
#   bash ~/final_task/scripts/sim/collect_placerand.sh <固定開始の本数> <開始姿勢ばらつきの本数>
# 環境は置き場所ランダム化・専用の録画先で起動しておく:
#   TASK_ENV_ARGS="randomize_place:=true" \
#   WEB_TELEOP_ARGS="recorder_config:=$HOME/final_task/ros2_ws/install/pai_web_teleop/share/pai_web_teleop/config/recorder_placerand.yaml" \
#   ~/final_task/scripts/sim/start_stack.sh
# 録画先: ~/dataset/rosbags/pick_place_placerand（失敗したエピソードは自動で破棄）
set -u
N_FIXED=$1; N_RAND=$2
CFG=~/final_task/ros2_ws/src/pai_task_env/config/scripted_demo.yaml
# 実験9 と同じ構成: ホームから開始するデモ ＋ 開始姿勢を ±3cm（z は ±2cm）ずらしたデモ
ros2 run pai_task_env scripted_demo --ros-args --params-file "$CFG" -p episodes:=$N_FIXED -p seed:=1200
ros2 run pai_task_env scripted_demo --ros-args --params-file "$CFG" -p episodes:=$N_RAND -p seed:=1201 \
  -p "randomize.start_offset:=[0.03,0.03,0.02]"
