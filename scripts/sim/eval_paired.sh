#!/bin/bash
# 同じ物体位置リストで複数の推論設定を評価する（条件間の対応のある比較）。コンテナ内で実行:
#   bash ~/final_task/scripts/sim/eval_paired.sh <seed> <trials> <label:infer_config> [<label:infer_config> ...]
#   評価ノードへの追加引数は環境変数 EVAL_ARGS で渡す（例: EVAL_ARGS="-p start.offset:=[0.01,0.01,0.01]"）
set -u
SEED=$1; TRIALS=$2; shift 2
EVAL_CFG=~/final_task/ros2_ws/src/pai_task_env/config/policy_evaluator.yaml
for c in "$@"; do
  label=${c%%:*}; cfg=${c#*:}
  pkill -f scripts/infer.py; sleep 2
  (cd ~/pai_training && nohup pixi run --frozen infer --config "$cfg" > /tmp/infer_$label.log 2>&1 &)
    # 推論ノードの起動を待つ（固定時間だと GPU を他と共有しているときに間に合わないことがある）
  for _ in $(seq 1 60); do ros2 service list 2>/dev/null | grep -q /policy/enable && break; sleep 2; done
  ros2 run pai_task_env policy_evaluator --ros-args --params-file "$EVAL_CFG" \
    -p trials:=$TRIALS -p label:=$label -p positions_seed:=$SEED -p observe.enabled:=true ${EVAL_ARGS:-} 2>&1 \
    | grep -E "summary" | sed "s/.*summary: //"
done
pkill -f scripts/infer.py
