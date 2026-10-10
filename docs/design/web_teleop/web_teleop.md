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

## 動作確認（2026-10-10）
- Gazebo 上で FK（`crane_plus_kinematics`）と TF（crane_plus_base→link3/link4）が 3 姿勢で一致
- ブラウザから W キー 1 秒で x: 0.12→0.18、pitch 自動緩和、グリッパ閉を確認。Gazebo の関節角・TF も追従
- 録画ボタンで ep0 を収録し `ros2 bag info` で全トピックを確認、破棄ボタンで退避
- グリッパ閉指令は 0.30rad（4cm 物体で爪が接触する 0.19rad より約 1cm 押し込む）。根拠は task_env 設計書のグリッパ幾何
