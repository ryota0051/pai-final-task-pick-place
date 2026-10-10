#!/bin/bash
# 物体位置の観測劣化に対する頑健性評価（案 A）。コンテナ内で実行:
#   docker exec -u ubuntu pai_final_task bash -ic '~/final_task/scripts/sim/eval_robustness.sh <infer_config> <trials>'
# 条件ごとに推論ノードを再起動し、policy_evaluator で評価する。結果は ~/dataset/eval/robust_*.csv
set -u
CFG=${1:-configs/pick/infer_objpos_ep117.yaml}
TRIALS=${2:-20}
EVAL_CFG=~/final_task/ros2_ws/src/pai_task_env/config/policy_evaluator.yaml

# ラベル:推論ノードへの追加オプション
CONDITIONS=(
  "robust_bias5mm:--object-bias 0.005"
  "robust_bias10mm:--object-bias 0.01"
  "robust_bias20mm:--object-bias 0.02"
  "robust_noise10mm:--object-noise 0.01"
  "robust_delay500ms:--object-delay 10"
)

for c in "${CONDITIONS[@]}"; do
  label=${c%%:*}; opts=${c#*:}
  pkill -f scripts/infer.py; sleep 2
  (cd ~/pai_training && nohup pixi run --frozen infer --config "$CFG" $opts > /tmp/infer_$label.log 2>&1 &)
  sleep 25
  ros2 run pai_task_env policy_evaluator --ros-args --params-file "$EVAL_CFG" \
    -p trials:=$TRIALS -p label:=$label 2>&1 | grep -E "summary" | sed "s/.*summary: //"
done
pkill -f scripts/infer.py
