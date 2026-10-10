# システム概要

## 目的
第5回 ROS 2入門(2/2) 実践演習（CRANE+ V2 を関節キーボードでテレオペ → rosbag → LeRobotDataset → ACT → Gazebo デプロイ）をベースに、
「テーブル上のランダムな位置にある物体へ手を伸ばし、掴んで所定位置に置く」タスクを模倣学習で解く。

## サンプルの弱点と改良
| 弱点 | 改良 | 担当パッケージ |
|---|---|---|
| 関節空間キーボード操作が直感的でない | ブラウザ（rosbridge + roslibjs）で手先 xyz＋pitch を操作、解析 IK で関節角へ変換 | `crane_plus_kinematics`, `pai_web_teleop` |
| 観測が関節角のみ・毎回同じ動作で丸暗記 | 物体位置をエピソードごとにランダム化し、観測に物体位置を追加 | `pai_task_env`, `external/pai_training` |
| 学習・推論が CPU 前提 | Docker に GPU を渡し `device: cuda` で学習・推論 | `docker/`, `external/pai_training` |
| デバッグがオフライン比較のみ | 推論中の予測・実関節角・物体位置を rerun / ブラウザ UI にストリーミング | `external/pai_training`, `pai_web_teleop` |

## 構成
```
[Browser UI] --rosbridge(9090)--> ee_teleop_node --IK--> /crane_plus_*_controller/joint_trajectory --> Gazebo(Ignition Fortress)
                                   recorder_node --> ros2 bag record (ep{N} 自動採番)
pai_task_env: randomizer_node(/task/reset) / object pose bridge(/task/object_pose) / evaluator_node(/task/success)
pai_training(fork): rosbag2lerobot(物体位置付き) -> ACT 学習(GPU) -> infer(物体位置観測, rerun 9876)
```

## フェーズ
0. リポジトリ初期化 / 1. Docker・GPU・サンプル再現 / 2. FK/IK・ブラウザ手先テレオペ / 3. タスク環境・データ / 4. 学習・評価 / 5. リアルタイム可視化 / 6. 提出物
