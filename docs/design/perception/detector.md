# 学習ベースの物体検出器（RT-DETRv2 ＋ ドメインランダム化）

方針は ADR 0008。俯瞰カメラ（320×240）の画像から物体（円柱）を検出し、物体位置を `/task/object_pose_est` に出す。
赤色検出ノード（`object_detector_node`）と同じトピック・同じ遮蔽時の扱い（`EstimateHolder`）なので、方策（ACT）は再学習せずに差し替えられる。

## 構成
| 段階 | 実装 | 内容 |
|---|---|---|
| 学習データ生成 | `pai_task_env/dr_dataset_generator`（設定 `config/domain_randomization.yaml`） | Gazebo でシーンをランダム化して俯瞰画像を保存。正解の枠は円柱を投影して自動で付ける（COCO 形式） |
| シーンのランダム化 | `pai_task_env/scene_randomizer.py`（生成と方策評価で共用） | 物体色・机の色味・照明・ダミーの箱を Gazebo のサービスで変更 |
| 学習 | fork `scripts/detection/train_detector.py`（設定 `configs/detection/train_rtdetr.yaml`） | `PekingU/rtdetr_v2_r18vd`（Apache-2.0, COCO 事前学習）を 1 クラスで追加学習 |
| 評価 | fork `scripts/detection/eval_detector.py`（`configs/detection/eval_rtdetr.yaml`） | 学習に使っていない画像で位置誤差を測り、赤色検出と比べる |
| 推論ノード | fork `scripts/detection/detect_node.py`（`configs/detection/detect_rtdetr.yaml`） | 枠の中心から位置を逆算して publish。タスク環境は `red_detector:=false` で起動する |

## ドメインランダム化（1 枚ごと）
| 項目 | 範囲 |
|---|---|
| 物体の位置 | タスクの配置範囲（`/task/reset`） |
| 物体の色 | 色相は全範囲、彩度 0.4〜1.0、明度 0.45〜1.0 |
| 机の色味 | RGB 各 0.5〜1.0 倍 |
| 照明 | 明るさ 0.5〜1.1、向きは鉛直から最大 0.7rad 傾ける（影の向きが変わる） |
| ダミーの箱 | 0〜3 個、一辺 2〜5cm、色ランダム、物体から 7cm 以上離す。見た目のみ（当たり判定なし） |
| 腕の姿勢 | 3 割はホーム、他は作業範囲内でランダム。腕（根元・肘・手首・手先を結ぶ線）が物体から 6cm 以内の姿勢は除く |

実装上の注意:
- 照明の設定（`light_config`）は範囲・減衰・強さも元の値で送る（省略すると 0 になり暗くなる）
- 箱は実行中に大きさを変えられないため、大きさ違い（6 種）を最初に作って位置と色だけ変える
- サービス呼び出しは 1 回 0.3 秒かかるので並列に送る。1 枚あたり約 5.6 秒
- 長時間の連続呼び出しの後に ign-transport が「Host unreachable」で応答しなくなったことがある（スタック再起動で復旧）

## 物体位置の求め方
検出した枠の中心を、物体中心の高さ（z=0.02m）の平面へそのまま投影すると、斜めから見える円柱の側面の分だけずれる（画像端で 1mm 以上）。
「その位置に円柱があるときの枠の中心」が検出値と一致するよう不動点反復で補正する（`domain_randomization.bbox_center_to_position`、幾何的な誤差 0.5mm 以内）。

## 学習（2026-10-11）
- データ: 1200 枚（学習 1080 / 検証 120）。入力は 640×640 に拡大
- 40 エポック、バッチ 16、AdamW（検出部 1e-4 / バックボーン 1e-5、コサイン減衰）、勾配クリップ 0.1
- 水増し: 明るさ・コントラスト・彩度・色相、ぼかし、ノイズ、左右反転
- RTX 4090 で約 13 分。検証損失 7.9 → 0.70

## 評価
### 画像単位（学習に使っていない 300 枚, 別シードで生成）
| | 検出率 | 位置誤差 平均 | 95% 点 | 5mm 以内 |
|---|---|---|---|---|
| 学習した検出器 | 300/300 | 0.28mm | 0.61mm | 100% |
| 赤色検出 | 9/300 | （誤検出を含む） | - | 1.7% |
| 赤色検出（物体が赤系の 36 枚のみ） | 6/36 | - | - | 14% |

- 赤色検出は淡い・暗い赤を見落とし、赤いダミーの箱を誤検出する
- 学習した検出器の誤差が 1 画素（約 1.5mm）未満なのは、シミュレーションで正解の枠が幾何的に正確で輪郭も鮮明なため。実機ではこの精度は期待できない

### 方策での評価（2026-10-11）
最終モデル（開始姿勢ばらつきデモ混合版, `infer_objpos_startrand_camera.yaml`）、配置固定 30 か所（seed 0）、observe 手順あり。
ランダム化は `policy_evaluator` の `scene_randomization.enabled:=true`（seed 0 で試行ごとに物体色・机・照明・ダミーの箱を変更）。

| シーン | 物体位置の推定 | 成功率 | 95% CI |
|---|---|---|---|
| ランダム化 | **学習した検出器** | **30/30 = 100%** | 89–100% |
| ランダム化 | 赤色検出 | 3/30 = 10% | 3–26% |
| 元のシーン（赤い物体） | 学習した検出器 | 30/30 = 100% | 89–100% |
| 元のシーン（赤い物体） | 赤色検出（実験9） | 30/30 = 100% | 89–100% |

- 学習した検出器は、見た目が変わっても元のシーンと同じ成功率を保った。赤色検出は物体が赤系でないと検出できず、ほぼ失敗する
- 方策（ACT）は再学習していない。物体位置の推定を差し替えるだけで頑健性が上がった

起動手順:
```bash
TASK_ENV_ARGS="red_detector:=false" ~/final_task/scripts/sim/start_stack.sh
cd ~/pai_training && pixi run --frozen detect-node --config configs/detection/detect_rtdetr.yaml &
EVAL_ARGS="-p scene_randomization.enabled:=true" bash ~/final_task/scripts/sim/eval_paired.sh 0 30 dr_learned:configs/pick/infer_objpos_startrand_camera.yaml
```

## 限界
- シミュレーション画像だけで学習しているため、実機の画像での動作は保証しない（主張は「実機に近い見た目の変化に頑健」まで）
- 対象は 1 個の円柱。ダミーは箱だけで、円柱同士の区別は扱っていない
- 置き場所の青い円盤は常に同じ見た目
