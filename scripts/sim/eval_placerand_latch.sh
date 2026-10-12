#!/bin/bash
# 実験12 の改良 A'（物体位置をグリッパが閉じている間は固定）のモデルを、置き場所もランダムに変えた配置固定 30 か所で評価する。
# 学習が終わるのを待ってから、真値 → 学習した検出器（元のシーン）→ 学習した検出器（見た目をランダム化）の順に評価する。コンテナ内で実行:
#   bash ~/final_task/scripts/sim/eval_placerand_latch.sh
# 前提: タスク環境を TASK_ENV_ARGS="randomize_place:=true red_detector:=false" で起動し、
#       2 クラスの検出ノード（configs/detection/detect_rtdetr_v2.yaml）を動かしておく
set -u
MODEL=~/checkpoints/train/crane_plus_pick_objpos_placerand_latch_act
until [ -f "$MODEL/model.safetensors" ] || grep -q Traceback /tmp/train_latch.log 2>/dev/null; do sleep 60; done
grep -q Traceback /tmp/train_latch.log 2>/dev/null && { echo "[NG] 学習が失敗しました: /tmp/train_latch.log"; exit 1; }

BASE="-p randomize_place:=true -p timeout:=40.0"
EVAL=~/final_task/scripts/sim/eval_paired.sh
EVAL_ARGS="$BASE" bash $EVAL 0 30 latch_gt:configs/pick/infer_objpos_placerand_latch.yaml
EVAL_ARGS="$BASE" bash $EVAL 0 30 latch_learned:configs/pick/infer_objpos_placerand_latch_learned.yaml
EVAL_ARGS="$BASE -p scene_randomization.enabled:=true -p scene_randomization.place:=true" \
  bash $EVAL 0 30 latch_learned_dr:configs/pick/infer_objpos_placerand_latch_learned.yaml
