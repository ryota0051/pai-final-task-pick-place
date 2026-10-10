# 0004 pixi と uv の使い分け

- ステータス: 承認
- 日付: 2026-10-10

## コンテキスト
プロジェクト方針では Python ライブラリは uv で管理する。一方、upstream の `pai_training` は pixi（conda-forge + PyPI、ffmpeg 等のネイティブ依存込み）で環境を構築している。

## 決定
- `external/pai_training`（LeRobot 学習・変換・推論）は upstream に合わせて **pixi** を使い続ける。ffmpeg 等の conda 依存を uv で再現するコストが大きく、演習との互換性も保てるため。
- 自作の純 Python コード（運動学ライブラリ、サンプラ、成功判定など ROS 非依存部分）のホスト側テストと集計スクリプトは **uv**（`tools/` の uv プロジェクト）で実行する。グローバル Python には何もインストールしない。
- ROS 2 パッケージはコンテナ内で colcon によりビルド・テストする。

## 結果
- 純 Python ロジックはホストで高速に TDD でき、ROS/GPU 依存部分はコンテナで検証する二段構えになる。
