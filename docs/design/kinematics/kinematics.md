# CRANE+ V2 運動学仕様

パッケージ: `ros2_ws/src/crane_plus_kinematics`（ROS 非依存の純 Python。ホストで `uv run pytest` 可能）

## パラメータ
`config/kinematics.yaml`（出典: rt-net/crane_plus humble の xacro）

| 名称 | 値 [m] | 説明 |
|---|---|---|
| base_to_joint1 | 0.0454 | crane_plus_base → joint1 |
| joint1_to_joint2 | 0.026 | joint1 → joint2（肩高さ = 0.0714） |
| joint2_to_joint3 (L2) | 0.083 | link2 |
| joint3_to_joint4 (L3) | 0.0935 | link3 |
| tool_length (Lt) | 0.125 | joint4 → 把持中心（可動指が机に当たらない深さ。根拠は task_env 設計書のグリッパ幾何） |
| tool_offset_y | 0.017 | 把持中心のアーム平面垂直方向オフセット（固定爪内面 0.037 − 物体半幅 0.02） |

関節リミットはサーボ原点 150° 基準で URDF の値を rad に換算して記載。

## 角度の定義
- joint1: z 軸回り（ヨー）。joint2: +y 軸回り。joint3, joint4: **-y 軸回り**。
- アーム平面内の角度 φ を「鉛直上向きから前方への傾き」とすると
  φ2 = q2, φ3 = q2 − q3, φ4 = q2 − q3 − q4
- 手先 pitch = φ4 − π/2（水平から下向きが正。真下 = +π/2、水平前向き = 0、真上 = −π/2）。全関節 0 は真上向き。

## FK
r = L2 sinφ2 + L3 sinφ3 + Lt sinφ4、h = z_sh + L2 cosφ2 + L3 cosφ3 + Lt cosφ4
(x, y) = Rz(q1)·(r, tool_offset_y)

## IK（`ArmKinematics.inverse(EndEffectorPose)`）
1. ヨー: r = ±√(x²+y²−d²)、q1 = atan2(y,x) − atan2(d, r)（前向き r>0 を優先し、後ろ向き r<0 も候補に）
2. リスト位置 W = 手先 − Lt·(sinφ4, cosφ4)、φ4 = pitch + π/2
3. 2 リンク平面 IK で β = ±acos(...) の 2 解
4. 関節リミット内の候補のうち **肘が最も高い解（肘上解）** を返す。解なしは `None`
   - 呼び出し側（テレオペノード）は `None` のとき直前の姿勢を維持する

## 作業空間クリップ（`Workspace`）
直方体 x/y/z と水平半径 r の範囲でクリップ。範囲は各ノードの設定 YAML で与える。

## テスト
`test/test_kinematics.py`, `test/test_workspace.py`
- 既知姿勢の FK、joint 回転方向、IK∘FK 往復（ランダム 2000 姿勢）、肘上解選択、到達不能、リミット判定、クリップ

## 未検証事項
- なし（tool_length / tool_offset_y は 2026-10-10 の把持試験で確定）
- テストは `test/fixtures/kinematics_test.yaml`（tool_length=0.07, offset=0）を使い、調整値の変更でテストが壊れないようにしている



## 検証済み
- Gazebo 上で `crane_plus_base→crane_plus_link3/link4` の TF と FK の肘・リスト位置が 3 姿勢で一致（2026-10-10）
- `/joint_states` の並びは joint1, joint2, **joint4, joint3**, hand（利用側は名前で並べ替えること）
