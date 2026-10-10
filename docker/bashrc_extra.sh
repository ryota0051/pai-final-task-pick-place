# final_task 用の追加シェル設定（~/.bashrc から source される）
export PATH="$HOME/.pixi/bin:$PATH"

if [ -f "$HOME/pai_ws/install/setup.bash" ]; then
  source "$HOME/pai_ws/install/setup.bash"
fi
if [ -f "$HOME/final_task/ros2_ws/install/setup.bash" ]; then
  source "$HOME/final_task/ros2_ws/install/setup.bash"
fi

# 自作パッケージのビルド
alias ft_build='(cd ~/final_task/ros2_ws && colcon build --symlink-install)'
