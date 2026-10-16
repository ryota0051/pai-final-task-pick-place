#!/bin/bash
# カメラ画像を入力する方策（A 案: 関節角 + 俯瞰カメラ画像, 物体位置なし）を収録から評価まで続けて実行する。コンテナ内で実行:
#   nohup bash ~/final_task/scripts/sim/pipeline_image.sh > /tmp/pipeline_image.log 2>&1 &
# 1. 置き場所ランダム・録画先 pick_place_image（カメラ画像つき）で環境を起動し、A' と同じ条件・seed でデモを収録
#    （固定開始 200 ＋ 開始姿勢ばらつき 200、seed 1200 / 1201）
# 2. 変換・学習（configs/pick/*_image.yaml）
# 3. 評価用に環境を起動し直す（置き場所ランダム）
# 4. 評価: A' と同じ seed 0 / 1 の 30 配置で、置き場所ランダム・置き場所固定・見た目ランダム化
#    SKIP_COLLECT=1 で収録を、SKIP_TRAIN=1 で変換・学習を飛ばす。N_FIXED / N_RAND で収録本数を変える（試験用）
set -u
log() { echo "[$(date +%H:%M:%S)] $*"; }
SIM=~/final_task/scripts/sim
N_FIXED=${N_FIXED:-200}; N_RAND=${N_RAND:-200}
# 起動は Gazebo の通信不調で失敗することがあるので 3 回まで試す
start() { for _ in 1 2 3; do $SIM/start_stack.sh | tail -1 | tee /dev/stderr | grep -q '^\[OK\]' && return 0; done
          log "[NG] 環境を起動できない"; exit 1; }

if [ -z "${SKIP_COLLECT:-}" ]; then
  TASK_ENV_ARGS="randomize_place:=true" \
  WEB_TELEOP_ARGS="recorder_config:=$HOME/final_task/ros2_ws/install/pai_web_teleop/share/pai_web_teleop/config/recorder_image.yaml" \
    start
  bash $SIM/collect_placerand.sh "$N_FIXED" "$N_RAND" 1200 1201 > /tmp/collect_image.log 2>&1
  log "収録完了: $(grep 'done:' /tmp/collect_image.log | sed 's/.*done: //' | tr '\n' ' ')"
fi

if [ -z "${SKIP_TRAIN:-}" ]; then
  cd ~/pai_training
  pixi run --frozen python scripts/rosbag2lerobot.py --config configs/pick/conversion_image.yaml \
    --overwrite > /tmp/conv_image.log 2>&1 || { log "[NG] 変換に失敗: /tmp/conv_image.log"; exit 1; }
  log "変換完了: $(grep -c 'saved episode' /tmp/conv_image.log) エピソード"
  pixi run --frozen python scripts/train.py --config configs/pick/train_image.yaml ${TRAIN_ARGS:-} \
    > /tmp/train_image.log 2>&1 || { log "[NG] 学習に失敗: /tmp/train_image.log"; exit 1; }
  log "学習完了: $(tail -1 /tmp/train_image.log)"
fi

TASK_ENV_ARGS="randomize_place:=true" start

P=configs/pick
BASE="-p timeout:=40.0"
RAND="$BASE -p randomize_place:=true"
EVAL=$SIM/eval_paired.sh
EVAL_ARGS="$RAND" bash $EVAL 0 30 image:$P/infer_image.yaml
EVAL_ARGS="$BASE" bash $EVAL 0 30 image_fixedplace:$P/infer_image.yaml
EVAL_ARGS="$RAND -p scene_randomization.enabled:=true -p scene_randomization.place:=true" \
  bash $EVAL 0 30 image_dr:$P/infer_image.yaml
log "評価（seed 0）完了"
EVAL_ARGS="$RAND" bash $EVAL 1 30 image_s1:$P/infer_image.yaml
EVAL_ARGS="$BASE" bash $EVAL 1 30 image_fixedplace_s1:$P/infer_image.yaml
log "評価（seed 1）完了"
