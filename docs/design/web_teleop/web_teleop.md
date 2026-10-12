# ブラウザ手先テレオペ仕様

パッケージ: `ros2_ws/src/pai_web_teleop`

## 起動
```bash
# Gazebo（タスク環境は pai_task_env の launch を使用）
ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
# rosbridge(9090) + ee_teleop_node + recorder_node + HTML 配信(8080)
ros2 launch pai_web_teleop web_teleop.launch.py
```
ホストのブラウザで `http://localhost:8080`（rosbridge の URL は `?ws=ws://host:9090` で上書き可）。

## ノード
### ee_teleop_node（設定: `config/teleop.yaml`）
| I/F | 名前 | 型 | 内容 |
|---|---|---|---|
| sub | `/teleop/ee_target` | geometry_msgs/PoseStamped | 手先絶対目標（姿勢は y 軸回り回転 = pitch） |
| sub | `/teleop/ee_velocity` | geometry_msgs/Twist | 手先速度 (linear xyz [m/s], angular.y = pitch 速度)。`velocity_timeout` 途絶で停止 |
| sub | `/teleop/gripper` | std_msgs/Float64 | 0=開 〜 1=閉 |
| sub | `/joint_states` | sensor_msgs/JointState | 再有効化時の姿勢同期用 |
| pub | `/crane_plus_arm_controller/joint_trajectory` | JointTrajectory | 20Hz, time_from_start 0.1s（演習と同一 I/F） |
| pub | `/crane_plus_gripper_controller/joint_trajectory` | JointTrajectory | 同上 |
| pub | `/teleop/ee_state` | PoseStamped | 現在の手先目標（crane_plus_base 基準）。収録対象 |
| pub | `/teleop/status` | std_msgs/String(JSON) | enabled, ik_ok, pitch_relaxed, joints, target, gripper |
| srv | `/teleop/home` | std_srvs/Trigger | ホーム姿勢（`home_ee`。pitch 緩和付きで解く）へ |
| srv | `/teleop/enable` | std_srvs/SetBool | 方策実行中は false にして publish を止める。true 時は現在関節角から目標を再構成 |

**IK 失敗時の挙動**: 作業空間クリップ → 要求 pitch で IK → 解けなければ pitch を `pitch_search_step` 刻みで要求値に近い順に探索（位置優先）→ それでも解なしなら直前の姿勢を維持し `ik_ok=false`。
要求 pitch は別途保持するため、届く位置に戻ると要求 pitch に復帰する。

### recorder_node（設定: `config/recorder.yaml`）
| I/F | 名前 | 内容 |
|---|---|---|
| srv | `/recorder/start` | `ros2 bag record -s sqlite3 -o <output_root>/ep{N} <topics>` を起動。N は既存 `ep*` の最大+1 |
| srv | `/recorder/stop` | SIGINT で停止 |
| srv | `/recorder/discard_last` | 直前エピソードを `_discarded/` に退避（削除はしない） |
| pub | `/recorder/status` | recording, episode, elapsed, next_episode, last_episode (JSON) |

`/clock` は 1kHz で容量を圧迫するため収録しない。

## UI（`web/`、設定は `web/config.json`）
- スライダー: x / y / z / pitch の絶対目標（`/teleop/ee_state` で同期）
- キーボード: W/S=x±, A/D=y±, R/F=z±, T/G=pitch±, O/C=開/閉, H=ホーム, Space=録画トグル
- ゲームパッド: 左スティック=xy, 右スティック上下=z, LB/RB=pitch, A=閉, B=開
- ボタン: タスクリセット（`/task/reset` → ホーム → 開）、録画開始/停止、直前を破棄、方策 ON/OFF
- 表示: IK 状態、関節角、録画状態、物体位置、タスク状態、方策状態

## ブラウザ UI で学習済みモデルを動かす（自律実行）
推論ノードは UI から起動できないので、ターミナルで起動してから UI の「方策 ON / OFF」で操作する。

1. シミュレーションとテレオペ UI を起動する（コンテナ内）
   ```bash
   ~/final_task/scripts/sim/start_stack.sh
   ```
2. 推論ノードを起動する（別のターミナル。物体位置の取り方で設定を選ぶ）
   ```bash
   cd ~/pai_training
   pixi run --frozen infer --config configs/pick/infer_objpos_startrand.yaml          # 物体位置は真値
   pixi run --frozen infer --config configs/pick/infer_objpos_startrand_camera.yaml   # 俯瞰カメラの赤色検出で推定
   ```
   学習した物体検出器を使う場合は、手順 1 を `TASK_ENV_ARGS="red_detector:=false" ~/final_task/scripts/sim/start_stack.sh` で起動し、
   `pixi run --frozen detect-node --config configs/detection/detect_rtdetr.yaml` を動かしてから、カメラ版の設定で推論ノードを起動する
   （詳細は [perception/detector.md](../perception/detector.md)）
3. ブラウザで http://localhost:8080 を開き、Gazebo の様子は <http://localhost:6080>（noVNC）で見る
4. 「タスクリセット」で物体をランダムな位置に置き直す（腕はホームに戻り、グリッパが開く）
5. 腕が止まったら「方策 ON」を押す。テレオペが無効になり、方策が腕を動かす
   - 「自律実行（方策）」の状態欄に `実行中 / 推論時間 ms / pred: 予測した関節角` が出る
   - データ収集の欄の「タスク」が「成功」になれば完了（物体を置き場所に置いて 0.5 秒静止）
6. 「方策 OFF」で止める（テレオペが有効に戻る。腕はその場で止まる）。続けて試すときは 4 から繰り返す

注意:
- 「方策 ON」で方策の内部状態（temporal ensembling の履歴）はリセットされる。動作中に OFF → ON した場合も最初からやり直しになる
- 推論ノードは物体位置を受け取るまで指令を出さない。カメラ版で腕の根元付近に物体があると、ホーム姿勢の腕に隠れて検出できないことがある。
  その場合は手先を一度その場から動かして（スライダーやキー）物体を見せてから、ホームに戻して ON にする
- 推論中の予測・関節角・物体位置は rerun（<http://localhost:9877/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy>）で見られる（`?url=` を付けないと rerun の紹介画面になる）
- 最終モデル（開始姿勢をばらつかせたデモを追加）は開始姿勢のずれに強いが、固定開始のデモだけで学習したモデル（`infer_objpos_ep117*.yaml`）は
  ホームで腕が止まる前に ON にすると失敗しやすい（[training_eval](../training_eval/training_eval.md) の実験7・8）

## 動作確認（2026-10-10）
- Gazebo 上で FK（`crane_plus_kinematics`）と TF（crane_plus_base→link3/link4）が 3 姿勢で一致
- ブラウザから W キー 1 秒で x: 0.12→0.18、pitch 自動緩和、グリッパ閉を確認。Gazebo の関節角・TF も追従
- 録画ボタンで ep0 を収録し `ros2 bag info` で全トピックを確認、破棄ボタンで退避
- ホームは `[0.10, 0.0, 0.08]`、pitch 65°（作業範囲全域で IK 可能な傾き。task_env 設計書参照）
- グリッパ閉指令は 0.30rad（4cm 物体で爪が接触する 0.19rad より約 1cm 押し込む）。根拠は task_env 設計書のグリッパ幾何

## 人手デモの収録と操作時間の比較（手順）
### A. ブラウザ UI でデモを収録する（20 回程度）
1. 録画先を人手デモ用に分けてスタックを起動する（コンテナ内）
   ```bash
   WEB_TELEOP_ARGS="recorder_config:=$HOME/final_task/ros2_ws/install/pai_web_teleop/share/pai_web_teleop/config/recorder_human.yaml" ~/final_task/scripts/sim/start_stack.sh
   ```
2. 操作時間も測る場合は、別のターミナルで計測ノードを起動する（`~/dataset/teleop_time/browser.csv` に追記される）
   ```bash
   ros2 run pai_task_env episode_timer --ros-args --params-file ~/final_task/ros2_ws/src/pai_task_env/config/episode_timer.yaml -p label:=browser
   ```
3. ブラウザで <http://localhost:8080>（UI）と <http://localhost:6080>（Gazebo の画面）を開く。キー操作は UI のページをクリックしてから行う
4. 1 回分の操作
   1. 「タスクリセット」: 物体がランダムな位置に置かれ、腕がホームに戻る
   2. 「録画開始」（Space）
   3. 物体の真上へ: W/S で前後（x）、A/D で左右（y）
   4. 下ろして掴む: F で下げる → C で閉じる
   5. 持ち上げて運ぶ: R で上げる → W/S・A/D で青い置き場所の真上へ
   6. 置く: F で下げる → O で開く → R で少し上げる
   7. データ収集の欄の「タスク」が「成功」になったら「録画停止」（Space）
   8. 失敗したら「録画停止」のあと「直前を破棄」（`_discarded/` へ移る）
5. 4 を繰り返す。録画は `~/dataset/rosbags/pick_place_human/ep0, ep1, ...` に保存される

コツ: 手先の速さは最大 8cm/s。pitch（T/G）は変えなくてよい。ゲームパッドでも操作できる（左スティック=xy、右スティック上下=z、A=閉、B=開）。

### B. 関節キーボード（サンプルの操作方法）で操作時間を測る（5 回程度）
関節キーボードは起動すると腕を「全関節 0（真っすぐ立てた姿勢）」へ動かすので、その姿勢になってから物体を置き直して計測を始める。
1. ブラウザ側のテレオペを止める（有効なままだと指令がぶつかる）
   ```bash
   ros2 service call /teleop/enable std_srvs/srv/SetBool "{data: false}"
   ```
2. 計測ノードを `label:=keyboard` で起動する（`~/dataset/teleop_time/keyboard.csv`）
   ```bash
   ros2 run pai_task_env episode_timer --ros-args --params-file ~/final_task/ros2_ws/src/pai_task_env/config/episode_timer.yaml -p label:=keyboard
   ```
3. 対話できるターミナル（`docker exec -it -u ubuntu pai_final_task bash`）で関節キーボードを起動する
   ```bash
   ros2 run physicalai_teleop teleop_keyboard
   ```
   キー: Q/E で関節を選ぶ（joint1〜4）、W/S で選んだ関節を ±、R/F で全関節を ±（粗く）、O/C でグリッパ開/閉、X で全関節 0 に戻す、ESC で終了
4. 腕が止まったら、別のターミナルで物体を置き直す（ここから計測が始まる）
   ```bash
   ros2 service call /task/reset std_srvs/srv/Trigger
   ```
5. 物体を掴んで置き場所に置く。「成功」になると計測ノードに時間が表示される。X で姿勢を戻し、4 から繰り返す
6. 終わったら ESC で関節キーボードを止め、ブラウザ側のテレオペを戻す（`"{data: true}"`）

計測は「リセット後に関節が動き始めてから成功まで」（`episode_timer`, task_env 仕様書）。

## 人手デモの録画先
学習用のスクリプトデモ（`~/dataset/rosbags/pick_place`）と分けるため、人手テレオペは `config/recorder_human.yaml`
（`~/dataset/rosbags/pick_place_human`）で録画する:
```bash
WEB_TELEOP_ARGS="recorder_config:=$HOME/final_task/ros2_ws/install/pai_web_teleop/share/pai_web_teleop/config/recorder_human.yaml"   ~/final_task/scripts/sim/start_stack.sh
```
