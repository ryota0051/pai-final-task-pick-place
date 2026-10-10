# Docker 環境

## 構成
| ファイル | 役割 |
|---|---|
| `docker/Dockerfile` | `airobotbook/ros2-desktop-ai-robot-book-humble`（演習イメージ。crane_plus / ros_gz ビルド済み）に rosbridge・pixi・upstream physicalai_teleop を追加 |
| `docker/compose.yaml` | GPU (`gpus: all`)、ポート、ボリューム定義 |
| `docker/.env.example` | ポート・コンテナ名・データディレクトリ（`docker/.env` にコピーして使用） |
| `docker/bashrc_extra.sh` | pixi の PATH、各ワークスペースの source、`ft_build` エイリアス |

注意: ベースイメージの ROS apt 署名鍵は 2025/06 に期限切れのため、Dockerfile で最新鍵に差し替えている。

## ポート
| ホスト | コンテナ | 用途 |
|---|---|---|
| 6080 | 80 | noVNC（デスクトップ） |
| 9090 | 9090 | rosbridge websocket |
| 9876 | 9876 | rerun gRPC / SDK |
| 9877 | 9877 | rerun web viewer |
| 8080 | 8080 | ブラウザテレオペ UI（HTML 配信） |

## ボリューム
| ホスト | コンテナ | 備考 |
|---|---|---|
| `ros2_ws/src` | `~/final_task/ros2_ws/src` | 自作パッケージ（bind） |
| named volume | `~/final_task/ros2_ws/{build,install,log}` | colcon 成果物 |
| `external/pai_training` | `~/pai_training` | fork（bind）。演習と同じパス |
| named volume `pixi_env` | `~/pai_training/.pixi` | Windows bind mount 上だと pixi が遅いため分離 |
| `data/dataset`, `data/checkpoints` | `~/dataset`, `~/checkpoints` | 演習と同じパス。git 管理外 |

## 使い方
```bash
cd docker
cp .env.example .env
docker compose up -d --build
docker exec -it -u ubuntu pai_final_task bash   # もしくは http://localhost:6080
ft_build                                        # 自作パッケージのビルド
~/final_task/scripts/sim/start_stack.sh          # Gazebo + タスク環境 + テレオペ UI を起動
cd ~/pai_training && pixi install               # 初回のみ
```

## GPU
- ホスト: Windows 11 + Docker Desktop (WSL2) + RTX 4090。Docker Desktop の nvidia ランタイムで `gpus: all` が有効。
- コンテナ内 `nvidia-smi` で GPU を確認済み。

## 注意: RMW 実装の統一
ベースイメージの `~/.bashrc` で `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` を設定している。
`docker exec ... bash -c '...'`（非対話シェル）では `.bashrc` が読まれず FastDDS で起動してしまい、
CycloneDDS 側のノードとトピックは一部通じてもサービスがハングする（2026-10-10 に推論ノードで発生、`RTPS_READER_HISTORY` エラー）。
コンテナ内のコマンドは `bash -ic` か対話シェル（`docker exec -it -u ubuntu pai_final_task bash`）で実行すること。
