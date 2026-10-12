# rosbag → LeRobotDataset 変換仕様

対象: `external/pai_training`（fork, ブランチ `feature/pick-place`）

## 構成
| ファイル | 役割 |
|---|---|
| `src/pai_training/conversion.py` | 変換の純粋ロジック（名前による関節並べ替え、sample-and-hold、観測の組み立て）。`tests/test_conversion.py` |
| `scripts/rosbag2lerobot.py` | rosbag 読み出しと LeRobotDataset 書き込み。`--config`, `--overwrite` |
| `configs/pick/conversion_{baseline,objpos}.yaml` | 把持・配置タスク用設定 |
| `configs/pick/conversion_objpos_startrand.yaml` | 固定開始 117 ＋ 開始姿勢ばらつきデモの混合（training_eval の実験9） |
| `configs/pick/features_{baseline,objpos}.py` | LeRobot features 定義 |

```bash
cd ~/pai_training
pixi run --frozen convert --config configs/pick/conversion_objpos.yaml --overwrite
pixi run --frozen convert --config configs/pick/conversion_baseline.yaml --overwrite
pixi run --frozen test   # 変換・推論ユーティリティの単体テスト
```
※ `pixi install` で lock が再解決されることがあるため `--frozen` を付ける。

## 設定項目（演習の conversion_config.yaml に追加）
| キー | 既定値（＝演習互換） | 内容 |
|---|---|---|
| `joint_order` | j1, j2, **j4, j3**, hand | 観測・行動の関節順。**名前で並べ替える** |
| `topics.object_pose` | なし | 物体位置トピック（PoseStamped） |
| `observation.state` / `observation.environment_state` | `[joints]` / `[joints]` | `joints`・`object_xyz` の連結 |
| `pad_to_longest` | true | 短いエピソードを最終フレームで延長（演習と同じ） |
| `bag_dir` | 文字列 | 収録ディレクトリ。**リストで複数指定すると順に連結**して 1 つのデータセットにする（`episode_discovery.find_episode_dirs`、`_` で始まるディレクトリは除外） |

把持・配置タスクでは `joint_order` を j1, j2, j3, j4, hand とし、
- ベースライン: state=関節角(5), environment_state=関節角(5)（演習と同じ構成）
- 改良版: state=関節角(5), environment_state=**物体位置(3)**

## 演習版からの変更点
1. `/joint_states` をメッセージ内の並び順のまま使っていた → 名前で並べ替え（推論側は名前で並べており不整合のリスクがあった）
2. 指令（JointTrajectory）も `joint_names` で並べ替え（自作テレオペは j1..j4 順で publish するため）
3. 変換ロジックを関数化してテスト可能に
4. サンプリング時刻を `linspace(start, end, n, endpoint=False)` に（フレーム間隔を 1/fps に揃える）
5. 出力先の上書き確認を `--overwrite` で省略可能に（自動化のため）
6. `_` で始まるディレクトリ（`_discarded` など）は変換対象外

## データセットの確認（2026-10-12）
### rerun での目視
```bash
cd ~/pai_training
pixi run --frozen python scripts/visualize_dataset.py --repo-id crane_plus_pick_objpos   --root ~/dataset/lerobot_dataset/crane_plus_pick_objpos_startrand --episode-index 0 --save 1 --output-dir ~/dataset/analysis/rrd --num-workers 0
pixi run --frozen rerun --serve-web --web-viewer-port 9877 --port 9876 ~/dataset/analysis/rrd/*.rrd   # http://localhost:9877
```
- サンプルの `visualize_dataset.py` は関節角と指令を添字で出すだけで物体位置を出さなかったため、関節ごとに指令（action）と関節角（state）を重ね、
  物体位置（`observation.environment_state`）も別のグラフに出すよう改修（`pai_training.visualization`）
- 左の Recordings で表示するエピソードを選ぶ

### 全エピソードの数値検査（`pixi run --frozen check-dataset --dataset-root ... --split 117`）
最終モデルの学習データ（固定開始 117 ＋ 開始姿勢ばらつき 118 = 235 エピソード）:

| 項目 | 結果 | 判断 |
|---|---|---|
| 指令に対する関節角の遅れ | 8〜10 フレーム（0.4〜0.5 秒）で全エピソードほぼ一定 | 時刻ずれ・関節の取り違えなし（コントローラの追従遅れ） |
| 把持の指令までに物体が動いた量 | 最大 0.8mm（ep153。他は 0.01mm 未満） | 物体位置は観測に正しく入っている |
| 末尾（延長部分）の指令の変化 | 最大 0.003 | 最終フレームで正しく延長されている |
| グリッパの閉指令から閉じ始めるまで | 1〜2 フレーム | 把持の時刻のずれなし |
| 最後の物体位置と置き場所の中心の距離 | 平均 3.5mm、最大 18.6mm（置き場所の半径 35mm 以内） | 全エピソードで置けている |
| 開始姿勢の標準偏差 | 固定開始 約 0.005rad、開始姿勢ばらつき 0.13〜0.22rad | ばらつきが意図どおり入っている |

結論: 変換は意図どおりで、学習データに問題は見つからなかった。
