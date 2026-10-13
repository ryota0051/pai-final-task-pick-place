#!/bin/bash
# 実験12（デモを倍に増やしたモデル x2）を、追加収録の完了から評価まで続けて実行する。コンテナ内で実行:
#   bash ~/final_task/scripts/sim/pipeline_placerand_x2.sh
# 1. 追加収録（collect_placerand.sh 200 200 1300 1301, ログ /tmp/collect_placerand2.log）の完了を待つ
# 2. 変換・学習（configs/pick/*_placerand_latch_x2.yaml）
# 3. 評価用に環境を起動し直す（置き場所ランダム・赤色検出なし）→ 2 クラスの検出ノードを起動
# 4. 評価 4 条件: 置き場所ランダムで 真値 / 検出器 / 検出器＋見た目ランダム化、置き場所固定で 真値
set -u
log() { echo "[$(date +%H:%M:%S)] $*"; }
until [ "$(grep -c 'done:' /tmp/collect_placerand2.log 2>/dev/null)" = "2" ]; do sleep 60; done
log "収録完了: $(grep 'done:' /tmp/collect_placerand2.log | sed 's/.*done: //' | tr '\n' ' ')"

cd ~/pai_training
pixi run --frozen python scripts/rosbag2lerobot.py --config configs/pick/conversion_objpos_placerand_latch_x2.yaml \
  --overwrite > /tmp/conv_x2.log 2>&1 || { log "[NG] 変換に失敗: /tmp/conv_x2.log"; exit 1; }
log "変換完了: $(grep -c 'saved episode' /tmp/conv_x2.log) エピソード"
pixi run --frozen python scripts/train.py --config configs/pick/train_objpos_placerand_latch_x2.yaml \
  > /tmp/train_x2.log 2>&1 || { log "[NG] 学習に失敗: /tmp/train_x2.log"; exit 1; }
log "学習完了: $(tail -1 /tmp/train_x2.log)"

TASK_ENV_ARGS="randomize_place:=true red_detector:=false" ~/final_task/scripts/sim/start_stack.sh | tail -1
(cd ~/pai_training && nohup pixi run --frozen detect-node --config configs/detection/detect_rtdetr_v2.yaml \
  > /tmp/detect_v2.log 2>&1 &)
sleep 30

BASE="-p timeout:=40.0"
RAND="$BASE -p randomize_place:=true"
EVAL=~/final_task/scripts/sim/eval_paired.sh
EVAL_ARGS="$RAND" bash $EVAL 0 30 x2_gt:configs/pick/infer_objpos_placerand_latch_x2.yaml
EVAL_ARGS="$RAND" bash $EVAL 0 30 x2_learned:configs/pick/infer_objpos_placerand_latch_x2_learned.yaml
EVAL_ARGS="$RAND -p scene_randomization.enabled:=true -p scene_randomization.place:=true" \
  bash $EVAL 0 30 x2_learned_dr:configs/pick/infer_objpos_placerand_latch_x2_learned.yaml
EVAL_ARGS="$BASE" bash $EVAL 0 30 x2_gt_fixedplace:configs/pick/infer_objpos_placerand_latch_x2.yaml
log "評価完了"
