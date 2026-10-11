#!/bin/bash
# コンテナ内で シミュレーション + テレオペ UI 一式を（再）起動する開発用スクリプト。
#   docker exec -it -u ubuntu pai_final_task bash -ic '~/final_task/scripts/sim/start_stack.sh'
# ログ: /tmp/task_env.log, /tmp/web_teleop.log
# タスク環境の launch 引数は TASK_ENV_ARGS（例: TASK_ENV_ARGS="red_detector:=false"）、
# テレオペ側の launch 引数は WEB_TELEOP_ARGS で渡す（例: 人手デモの録画先を分ける
#   WEB_TELEOP_ARGS="recorder_config:=$HOME/final_task/ros2_ws/install/pai_web_teleop/share/pai_web_teleop/config/recorder_human.yaml"）
set -u
WS=~/final_task/ros2_ws
TASK_WAIT=${TASK_WAIT:-20}   # Gazebo・コントローラの起動待ち [s]

pkill -f "ros2 launch" ; pkill -f "ign gazebo"; pkill -f robot_state_publisher; pkill -f parameter_bridge
pkill -f rosbridge; pkill -f "http.server"; pkill -f ee_teleop_node; pkill -f recorder_node
pkill -f task_manager_node; pkill -f object_detector_node; pkill -f spawner
sleep 3

(cd "$WS" && colcon build --symlink-install 2>&1 | tail -1)
set +u; source "$WS/install/setup.bash"; set -u   # setup.bash は未定義変数を参照する
export DISPLAY=${DISPLAY:-:0}

nohup ros2 launch pai_task_env task_env.launch.py ${TASK_ENV_ARGS:-} > /tmp/task_env.log 2>&1 &
sleep "$TASK_WAIT"
nohup ros2 launch pai_web_teleop web_teleop.launch.py ${WEB_TELEOP_ARGS:-} > /tmp/web_teleop.log 2>&1 &
sleep 10

if grep -qiE "Traceback|process has died" /tmp/task_env.log /tmp/web_teleop.log; then
  echo "[NG] 起動エラーがあります:"; grep -iE "Traceback|process has died" /tmp/task_env.log /tmp/web_teleop.log | head
  exit 1
fi
echo "[OK] Gazebo: noVNC http://localhost:6080 / テレオペ UI: http://localhost:8080"
