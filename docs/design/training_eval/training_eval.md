# 学習・推論・評価仕様

## 学習（`external/pai_training/scripts/train.py`）
```bash
pixi run --frozen train --config configs/pick/train_objpos.yaml            # GPU（device: cuda）
pixi run --frozen train --config configs/pick/train_objpos.yaml --device cpu --steps 500 --output /tmp/cpu_bench
```
- 追加設定: `wandb_mode`（既定 disabled）, `seed`, `episodes`, `weight_decay`, `act`（ACTConfig 追加引数）
- 学習時間・ms/step・最終 loss を `<output_directory>/train_info.json` に保存（CPU/GPU 比較用）
- `delta_timestamps.action` は 20fps に合わせ 0.0〜0.75s の 16 ステップ。演習設定は -0.1s 始まりで、`n_action_steps: 1` だと実行される行動が 0.1s 前のものになっていた

## 推論（`scripts/infer.py`）
```bash
source /opt/ros/humble/setup.bash
pixi run --frozen infer --config configs/pick/infer_objpos.yaml
```
| I/F | 名前 | 内容 |
|---|---|---|
| sub | `/joint_states`, `/task/object_pose` | 観測（`conversion_config` と同じ規則で組み立て = 学習時と一致） |
| pub | arm / gripper の joint_trajectory | 予測チャンクの先頭（または temporal ensembling） |
| pub | `/policy/status` | enabled, ready, step, latency_ms, predicted, device（ブラウザ UI に表示） |
| srv | `/policy/enable` (SetBool) | 有効化時に `policy.reset()` |
| srv | `/policy/reset` (Trigger) | |
| rerun | web 9877 / gRPC 9876 | 実関節角・指令・予測チャンク・物体位置・レイテンシ |

`start_enabled: false` のため、ブラウザ UI の「方策 ON」（テレオペを無効化して方策を有効化）か評価スクリプトから開始する。

## 評価（`ros2 run pai_task_env policy_evaluator`）
```bash
ros2 run pai_task_env policy_evaluator --ros-args \
  --params-file ~/final_task/ros2_ws/src/pai_task_env/config/policy_evaluator.yaml -p trials:=20 -p label:=objpos
```
1 試行: 方策停止 → テレオペでホーム → `/task/reset`（物体ランダム配置）→ テレオペ停止 → 方策 reset/enable → 成功 or timeout(30s)。
結果は `~/dataset/eval/<label>_<日時>.csv` と `_summary.json`（成功率・Wilson 95% 区間・持ち上げ率・平均成功時間）。

## オフライン評価（`scripts/debug_policy.py`）
```bash
pixi run --frozen debug-policy --dataset-root ... --repo-id ... --policy-path ... --episode-index 0 --device cuda --quiet --save /tmp/ep0.png
```
