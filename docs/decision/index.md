# ADR 一覧

| No | タイトル | 概要 |
|---|---|---|
| [0001](0001-リポジトリ構成.md) | リポジトリ構成 | 親リポジトリ＋pai_training fork を submodule。physicalai_teleop は取り込まない |
| [0002](0002-Docker-GPU構成.md) | Docker / GPU 構成 | 演習イメージベースの独自 Dockerfile + compose、gpus: all |
| [0003](0003-解析IKの採用.md) | 解析 IK の採用 | MoveIt を使わず URDF から導いた閉形式 IK を自作 |
| [0004](0004-pixiとuvの使い分け.md) | pixi と uv の使い分け | pai_training は pixi、自作純 Python/ホスト側は uv |
| [0005](0005-物体位置の取得方式.md) | 物体位置の取得方式 | OdometryPublisher + ros_gz_bridge で物体の world 姿勢を取得 |
| [0006](0006-把持アシスト.md) | 把持アシスト | 摩擦把持が滑落するため DetachableJoint で attach/detach |
