#!/bin/bash
# 実験13（外乱入りのデモ, DART）を収録から評価まで続けて実行する。コンテナ内で実行:
#   nohup bash ~/final_task/scripts/sim/pipeline_dart.sh > /tmp/pipeline_dart.log 2>&1 &
# 1. 置き場所ランダム・録画先 pick_place_dart で環境を起動し、外乱入りのデモを収録（固定開始 200 ＋ 開始姿勢ばらつき 200）
# 2. 変換・学習（configs/pick/*_placerand_latch_dart.yaml。ラベルは外乱なしの指令 /teleop/expert_arm_command）
# 3. 評価用に環境を起動し直す（置き場所ランダム・赤色検出なし）→ 2 クラスの検出ノードを起動
# 4. 評価: seed 0 の 30 配置で A' と同じ 4 条件、seed 1 の 30 配置で置き場所ランダム／固定（真値）を A' と DART の両方
#    SKIP_COLLECT=1 で収録を、SKIP_TRAIN=1 で変換・学習を飛ばす
set -u
log() { echo "[$(date +%H:%M:%S)] $*"; }
SIM=~/final_task/scripts/sim
DART_ARGS=${DART_ARGS:--p dart.max_xy:=0.01}

if [ -z "${SKIP_COLLECT:-}" ]; then
  TASK_ENV_ARGS="randomize_place:=true" \
  WEB_TELEOP_ARGS="recorder_config:=$HOME/final_task/ros2_ws/install/pai_web_teleop/share/pai_web_teleop/config/recorder_dart.yaml" \
    $SIM/start_stack.sh | tail -1
  DEMO_ARGS="$DART_ARGS" bash $SIM/collect_placerand.sh 200 200 1400 1401 > /tmp/collect_dart.log 2>&1
  log "収録完了: $(grep 'done:' /tmp/collect_dart.log | sed 's/.*done: //' | tr '\n' ' ')"
fi

if [ -z "${SKIP_TRAIN:-}" ]; then
  cd ~/pai_training
  pixi run --frozen python scripts/rosbag2lerobot.py --config configs/pick/conversion_objpos_placerand_latch_dart.yaml \
    --overwrite > /tmp/conv_dart.log 2>&1 || { log "[NG] 変換に失敗: /tmp/conv_dart.log"; exit 1; }
  log "変換完了: $(grep -c 'saved episode' /tmp/conv_dart.log) エピソード"
  pixi run --frozen python scripts/train.py --config configs/pick/train_objpos_placerand_latch_dart.yaml \
    > /tmp/train_dart.log 2>&1 || { log "[NG] 学習に失敗: /tmp/train_dart.log"; exit 1; }
  log "学習完了: $(tail -1 /tmp/train_dart.log)"
fi

TASK_ENV_ARGS="randomize_place:=true red_detector:=false" $SIM/start_stack.sh | tail -1
(cd ~/pai_training && nohup pixi run --frozen detect-node --config configs/detection/detect_rtdetr_v2.yaml \
  > /tmp/detect_v2.log 2>&1 &)
sleep 30

P=configs/pick
BASE="-p timeout:=40.0"
RAND="$BASE -p randomize_place:=true"
EVAL=$SIM/eval_paired.sh
EVAL_ARGS="$RAND" bash $EVAL 0 30 dart_gt:$P/infer_objpos_placerand_latch_dart.yaml
EVAL_ARGS="$RAND" bash $EVAL 0 30 dart_learned:$P/infer_objpos_placerand_latch_dart_learned.yaml
EVAL_ARGS="$RAND -p scene_randomization.enabled:=true -p scene_randomization.place:=true" \
  bash $EVAL 0 30 dart_learned_dr:$P/infer_objpos_placerand_latch_dart_learned.yaml
EVAL_ARGS="$BASE" bash $EVAL 0 30 dart_gt_fixedplace:$P/infer_objpos_placerand_latch_dart.yaml
log "評価（seed 0）完了"
# 別の 30 配置（seed 1）で A' と DART を対応ありで比べる（試行数を増やして差を見る）
EVAL_ARGS="$RAND" bash $EVAL 1 30 latch_gt_s1:$P/infer_objpos_placerand_latch.yaml \
  dart_gt_s1:$P/infer_objpos_placerand_latch_dart.yaml
EVAL_ARGS="$BASE" bash $EVAL 1 30 latch_gt_fixedplace_s1:$P/infer_objpos_placerand_latch.yaml \
  dart_gt_fixedplace_s1:$P/infer_objpos_placerand_latch_dart.yaml
log "評価（seed 1）完了"
