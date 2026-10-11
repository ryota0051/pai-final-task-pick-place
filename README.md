# CRANE+ V2 把持・配置タスクの模倣学習（Physical AI 2026 最終課題）

テーブル上の**ランダムな位置にある物体を掴み、所定の位置に置く**タスクを、Gazebo 上の CRANE+ V2 と模倣学習（ACT）で解きます。
講義の実践演習（関節キーボードでテレオペ → rosbag → LeRobotDataset → ACT → Gazebo で自律実行）をベースに、
操作・観測・評価の各段階を改良しました。

## 結果
Gazebo 上での自律実行の成功率（改良版は物体の配置を固定した同じ 30 か所で比較）:

| モデル | 物体位置の入力 | 成功率 |
|---|---|---|
| ベースライン（演習と同じ観測） | なし（関節角のみ） | 10%（20 試行） |
| 改良版 | 真値 | **100%**（30/30） |
| 改良版 | **俯瞰カメラから推定** | **100%**（30/30） |

詳細な実験記録（学習量・データ量・temporal ensembling・観測劣化への頑健性・失敗原因の分析）は
[docs/design/training_eval/training_eval.md](docs/design/training_eval/training_eval.md) を参照してください。

## 演習からの主な改良
| 演習の弱点 | 改良 |
|---|---|
| 関節空間のキーボード操作が直感的でない | ブラウザ UI（rosbridge）で手先の xyz・pitch を操作し、解析的逆運動学で関節角に変換 |
| 毎回同じ動作で、観測が関節角のみ | 物体位置をエピソードごとにランダム化し、観測に物体位置を追加 |
| 物体位置が既知の前提 | 俯瞰カメラの画像から物体位置を推定し、真値なしで自律実行 |
| 学習・推論が CPU 前提 | Docker に GPU を渡して学習・推論（RTX 4090 で約 50ms/step、60k step で約 1 時間） |
| 評価が目視・オフライン比較のみ | 成功判定・自動評価（Wilson 信頼区間、配置固定の比較）、rerun での推論の可視化、rosbag による失敗解析 |
| 行動チャンクの先頭だけを実行 | ACT の temporal ensembling で実行時の誤差の蓄積を抑制（成功率 55% → 100%） |

## 構成
```mermaid
flowchart LR
  subgraph collect["デモ収集"]
    UI["ブラウザ UI<br>手先 xyz・pitch"] -- rosbridge --> TELEOP["ee_teleop_node<br>解析 IK"]
    DEMO["scripted_demo<br>IK で軌道計画"]
  end
  subgraph sim["Gazebo"]
    ROBOT["CRANE+ V2・机<br>物体・置き場所"]
    CAM["俯瞰カメラ"]
  end
  subgraph task["pai_task_env"]
    MGR["task_manager_node<br>ランダム配置・成功判定<br>把持アシスト"]
    DET["object_detector_node<br>物体位置の推定"]
    EVAL["policy_evaluator<br>自動評価"]
  end
  subgraph learn["pai_training"]
    BAG[("rosbag")] --> CONV["LeRobotDataset<br>関節角＋物体位置"] --> TRAIN["ACT 学習<br>GPU"]
    TRAIN --> INFER["推論ノード<br>temporal ensembling"]
    INFER --> RERUN["rerun<br>可視化"]
  end
  TELEOP -- 関節指令 --> ROBOT
  DEMO -- 関節指令 --> ROBOT
  TELEOP -. 録画 .-> BAG
  DEMO -. 録画 .-> BAG
  ROBOT -- 関節角 --> INFER
  MGR -- 物体位置（真値） --> INFER
  CAM --> DET -- 物体位置（推定） --> INFER
  INFER -- 関節指令 --> ROBOT
  EVAL -. 開始・判定 .-> INFER
```

| ディレクトリ | 内容 |
|---|---|
| [ros2_ws/src/crane_plus_kinematics](ros2_ws/src/crane_plus_kinematics) | CRANE+ V2 の順運動学・解析的逆運動学（ROS 非依存） |
| [ros2_ws/src/pai_web_teleop](ros2_ws/src/pai_web_teleop) | ブラウザ手先テレオペ・録画・方策の ON/OFF |
| [ros2_ws/src/pai_task_env](ros2_ws/src/pai_task_env) | タスク環境（world・ランダム配置・成功判定・把持アシスト・カメラ検出・スクリプトデモ・評価） |
| [external/pai_training](https://github.com/ryota0051/pai_training/tree/feature/pick-place) | 演習の学習リポジトリの fork（submodule）。変換・学習・推論・解析 |
| [docker](docker) | 開発用 Docker（GPU）環境 |
| [scripts/sim](scripts/sim) | シミュレーション起動・評価スクリプト |
| [docs](docs) | 設計書（[docs/design](docs/design/index.md)）、設計判断（[docs/decision](docs/decision/index.md)）、作業ログ |

## 動かし方
前提: Docker（NVIDIA GPU を使う場合は NVIDIA Container Toolkit / Docker Desktop の GPU 対応）。

```bash
git clone --recursive https://github.com/ryota0051/pai-final-task-pick-place.git
cd pai-final-task-pick-place/docker
cp .env.example .env
docker compose up -d --build
docker exec -it -u ubuntu pai_final_task bash      # 以降はコンテナ内
cd ~/pai_training && pixi install                    # 初回のみ
~/final_task/scripts/sim/start_stack.sh              # ビルド + Gazebo + タスク環境 + テレオペ UI を起動
```
- Gazebo の画面: http://localhost:6080 （noVNC）
- テレオペ UI: http://localhost:8080
- 推論の可視化（rerun）: http://localhost:9877

### デモの収集
- 人のテレオペ: テレオペ UI で「タスクリセット」→「録画開始」→ 物体を掴んで置き場所へ →「録画停止」
- スクリプトデモ（IK で計画した軌道。学習にはこちらを 117 エピソード使用）:
  ```bash
  cd ~/final_task/ros2_ws
  ros2 run pai_task_env scripted_demo --ros-args --params-file src/pai_task_env/config/scripted_demo.yaml -p episodes:=40 -p seed:=0
  ```

### 変換・学習・推論・評価（`~/pai_training`）
```bash
pixi run --frozen convert --config configs/pick/conversion_objpos_ep117.yaml --overwrite   # rosbag → LeRobotDataset
pixi run --frozen train --config configs/pick/train_objpos_ep117.yaml                      # ACT 学習（GPU, 60k step で約 1 時間）
pixi run --frozen infer --config configs/pick/infer_objpos_ep117.yaml                      # 推論ノード（カメラ推定版は infer_objpos_ep117_camera.yaml）
```
推論ノードの起動後、テレオペ UI の「方策 ON」で自律実行します（腕がホームで静止してから ON にしてください）。

自動評価（物体配置を固定して真値 / カメラ推定を比較）:
```bash
bash ~/final_task/scripts/sim/eval_paired.sh 0 30 truth:configs/pick/infer_objpos_ep117.yaml camera:configs/pick/infer_objpos_ep117_camera.yaml
```
結果は `~/dataset/eval/` に CSV と成功率のサマリ（JSON）で出力されます。

### テスト
```bash
cd ~/final_task/ros2_ws/src/pai_task_env && python3 -m pytest test -q      # 各 ROS パッケージ
cd ~/pai_training && pixi run --frozen test
```

## 注意
- 学習データ・学習済みモデル（`data/`）は容量が大きいためリポジトリに含めていません。上記の手順で生成できます。
- ベースイメージは講義の演習イメージ（`airobotbook/ros2-desktop-ai-robot-book-humble`）、学習は講義の [pai_training](https://github.com/matsuolab/pai_training) を fork して拡張しています。
