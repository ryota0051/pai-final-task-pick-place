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

## 計測結果（スモークテスト, 2026-10-10）
暫定データ 15 エピソード（4485 フレーム）、batch 32、300 step:

| device | ms/step | 300 step |
|---|---|---|
| RTX 4090 (cuda) | 45.6 | 13.7 s |
| CPU | 329.9 | 99.0 s |

推論レイテンシ（GPU, チャンク 1 回予測）: 約 11 ms（20Hz 制御周期 50ms に十分収まる）

## 実験記録

### 実験1（2026-10-10）: スクリプトデモ 41 エピソード、20k step
| モデル | 観測 | 成功率（20 試行） | 95% CI | 平均成功時間 |
|---|---|---|---|---|
| ベースライン | 関節角のみ | 6/20 = 30% | 15–52% | 11.6 s |
| 改良版 | 関節角 + 物体位置（真値） | 8/20 = 40% | 22–61% | 7.4 s |

- 学習時間: 各 約 14 分（RTX 4090, 41 ms/step）。最終 loss 0.06〜0.07 でまだ低下中
- ベースラインの成功は物体が (0.10, ±0.03) 付近のときのみで、所要時間がほぼ一定（11.6 s）→ 平均的な軌道の再生
- 改良版の成功は y < 0.03 付近に偏る。オフライン誤差（学習データ上の 1 ステップ予測）は各関節 0.01〜0.04 rad と小さい
- 失敗試行のトレース（物体 (0.140, +0.060)）: 手先が (0.128, +0.036) に向かい、x 1.2cm・y 2.4cm 手前で閉じて指で物体を押し出していた
  → 物体位置への追従が平均側に引っ張られている（学習不足 / データ不足を疑う）
- 対策: 学習ステップ延長（60k）とデータ追加（+80 エピソード）を切り分けて検証中
