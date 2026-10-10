# 把持・配置タスク環境仕様

パッケージ: `ros2_ws/src/pai_task_env`（設定: `config/task.yaml`）

## 起動
```bash
ros2 launch pai_task_env task_env.launch.py          # Gazebo + CRANE+ + 物体 + task_manager_node
ros2 launch pai_web_teleop web_teleop.launch.py      # テレオペ UI（別ターミナル）
```
演習イメージの `crane_plus_with_table.launch.py` は world が `table.sdf` 固定のため、同等構成（ign gazebo / spawn / spawner / clock bridge）を
`task_env.launch.py` で組み直している。MoveIt・RViz は起動しない。

## world（`worlds/pick_place.sdf`）
- `table.sdf` と同じ Sun / Ground Plane / Table（Fuel）。テーブル上面 = ロボット基準 z≈0（ロボットは world z=1.015 にスポーン）
- `target_cube`: 4cm 角、0.03kg、mu=2.0。OdometryPublisher で world 姿勢を `/model/target_cube/odometry` に publish
- `place_zone`: 半径 3.5cm の可視マーカー（衝突なし・static）
- 物体・置き場所の位置の正は `task.yaml`。起動時とリセット時に `ign service /world/default/set_pose` で上書きする

## 座標系
観測・成功判定はすべて `crane_plus_base` 基準。world→base は `base_origin: [0, 0, 1.015]`, `base_yaw` で変換（`frames.BaseFrame`）。

## task_manager_node
| I/F | 名前 | 型 | 内容 |
|---|---|---|---|
| sub | `/task/object_odom` | nav_msgs/Odometry | ros_gz_bridge 経由の物体 world 姿勢 |
| sub | `/joint_states` | JointState | 把持中心（FK）と手先関節角 |
| pub | `/task/object_pose` | PoseStamped | 物体姿勢（crane_plus_base）。**学習の観測に使用** |
| pub | `/task/place_pose` | PoseStamped | 置き場所 |
| pub | `/task/status` | String(JSON) | lifted, in_place_zone, success, grasped, episode, object |
| pub | `/task/success` | Bool | 成功フラグ |
| pub | `/task/gz/attach`, `/task/gz/detach` | Empty | 把持アシスト（bridge で Gazebo へ） |
| srv | `/task/reset` | Trigger | 物体位置を再サンプル（detach → set_pose）、成功判定リセット |

### 物体位置サンプリング（`sampler.ObjectSampler`）
x/y 範囲から一様サンプルし、水平半径範囲 `r`（真下向き把持で IK 可能な範囲）と置き場所からの最小距離で棄却。`seed>=0` で再現可能。

### 成功判定（`success_checker.SuccessTracker`）
- pick: 物体が一度 `lift_height` 以上持ち上がり、その後置き場所半径内・テーブル上（z 許容差内）に `hold_time` 連続で静置
- reach（フォールバック）: 手先が物体に `reach_threshold` 以内で `hold_time` 連続

### 把持アシスト（`grasp_assist.GraspAssist`, ADR 0006）
ロボット記述をラッパー xacro（`urdf/crane_plus_task.urdf.xacro`）で読み込み、link4–物体間の DetachableJoint を追加。
グリッパ関節角 ≥ `close_threshold` かつ 把持中心と物体中心が `attach_distance` 以内で attach、関節角 ≤ `open_threshold` で detach。

## グリッパ幾何（メッシュから算出, 2026-10-10）
| 項目 | 値 |
|---|---|
| 固定爪 内面 | link4 y = 0.037、z = 0.074〜0.120 |
| 可動指 | hand 関節（link4 (0, -0.0145, 0.045)、-x 軸回り）。閉じると +y 方向へ |
| 爪間隔 | q=0.17: 4.1cm / q=0.30: 3.0cm / q=0.40: 2.1cm |
| 可動指先端の深さ | 閉動作中に最深 約 0.138（link4 z） |

→ 4cm キューブ、把持中心 `tool_length=0.125`, `tool_offset_y=0.017`、グリッパ閉指令 0.30rad を採用。

## 動作確認（2026-10-10）
スクリプト（上から接近→下降→閉→持ち上げ→置き場所へ→開）で 3 エピソード連続成功（`/task/reset` 込み）。
