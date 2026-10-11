# rosbag → LeRobotDataset 変換仕様

対象: `external/pai_training`（fork, ブランチ `feature/pick-place`）

## 構成
| ファイル | 役割 |
|---|---|
| `src/pai_training/conversion.py` | 変換の純粋ロジック（名前による関節並べ替え、sample-and-hold、観測の組み立て）。`tests/test_conversion.py` |
| `scripts/rosbag2lerobot.py` | rosbag 読み出しと LeRobotDataset 書き込み。`--config`, `--overwrite` |
| `configs/pick/conversion_{baseline,objpos}.yaml` | 把持・配置タスク用設定 |
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
