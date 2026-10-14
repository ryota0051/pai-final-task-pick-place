#!/bin/bash
# 置き場所もランダムに変えたスクリプトデモを収録する（実験12）。コンテナ内で実行:
#   bash ~/final_task/scripts/sim/collect_placerand.sh <固定開始の本数> <開始姿勢ばらつきの本数> [<seed 固定開始> <seed ばらつき>]
#   seed の既定は 1200 / 1201（最初の 339 本）。追加分は 1300 / 1301 で recorder_placerand2.yaml（pick_place_placerand2）へ
# 環境は置き場所ランダム化・専用の録画先で起動しておく:
#   TASK_ENV_ARGS="randomize_place:=true" \
#   WEB_TELEOP_ARGS="recorder_config:=$HOME/final_task/ros2_ws/install/pai_web_teleop/share/pai_web_teleop/config/recorder_placerand.yaml" \
#   ~/final_task/scripts/sim/start_stack.sh
# 録画先: ~/dataset/rosbags/pick_place_placerand（失敗したエピソードは自動で破棄）
# 環境変数 DEMO_ARGS で scripted_demo に追加の引数を渡せる。外乱（DART, 実験13）入りのデモは recorder_dart.yaml で起動して:
#   DEMO_ARGS="-p dart.max_xy:=0.01" bash collect_placerand.sh 200 200 1400 1401
set -u
N_FIXED=$1; N_RAND=$2; SEED_FIXED=${3:-1200}; SEED_RAND=${4:-1201}
CFG=~/final_task/ros2_ws/src/pai_task_env/config/scripted_demo.yaml
DEMO_ARGS=${DEMO_ARGS:-}
# 実験9 と同じ構成: ホームから開始するデモ ＋ 開始姿勢を ±3cm（z は ±2cm）ずらしたデモ
ros2 run pai_task_env scripted_demo --ros-args --params-file "$CFG" -p episodes:=$N_FIXED -p seed:=$SEED_FIXED $DEMO_ARGS
ros2 run pai_task_env scripted_demo --ros-args --params-file "$CFG" -p episodes:=$N_RAND -p seed:=$SEED_RAND \
  -p "randomize.start_offset:=[0.03,0.03,0.02]" $DEMO_ARGS
