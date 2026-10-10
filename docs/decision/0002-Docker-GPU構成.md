# 0002 Docker / GPU 構成

- ステータス: 承認
- 日付: 2026-10-10

## コンテキスト
演習は `airobotbook/ros2-desktop-ai-robot-book-humble` コンテナ（VNC デスクトップ）で行い、学習は CPU 前提だった。
最終課題ではブラウザ UI（rosbridge）と GPU 学習が必要。ホストは Windows 11 + Docker Desktop + RTX 4090。

## 決定
- 演習イメージをベースにした独自 Dockerfile と compose.yaml をリポジトリで管理する。
- compose で `gpus: all` を指定し、学習・推論を同一コンテナで GPU 実行する。
- pixi 環境とビルド成果物は named volume に置き、ソースは bind mount で共有する。

## 結果
- 演習と同じパス（`~/pai_training`, `~/dataset`, `~/checkpoints`）を維持でき、設定ファイルの互換性が保たれる。
- イメージが大きい（約 7.7GB）が、crane_plus / ros_gz の再ビルドが不要。
