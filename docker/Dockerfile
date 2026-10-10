# 演習で使用した AI ロボット本 Humble イメージ（crane_plus / ros_gz ビルド済み）をベースにする
ARG BASE_IMAGE=airobotbook/ros2-desktop-ai-robot-book-humble
FROM ${BASE_IMAGE}

ARG PIXI_VERSION=v0.48.2
ARG TELEOP_REPO=https://github.com/matsuolab/physicalai_teleop.git

SHELL ["/bin/bash", "-c"]

# ベースイメージの ROS apt 署名鍵が期限切れ (2025/06) のため最新鍵に差し替える
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

# rosbridge（ブラウザ UI 用）とテスト用ツール
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rosbridge-suite \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

USER ubuntu
WORKDIR /home/ubuntu

# pixi（pai_training の環境構築用）
RUN curl -fsSL https://pixi.sh/install.sh | PIXI_VERSION=${PIXI_VERSION} bash

# 操作性比較のベースラインとして upstream のキーボード関節テレオペをビルド
RUN mkdir -p ~/pai_ws/src && cd ~/pai_ws/src \
    && git clone --depth 1 ${TELEOP_REPO} \
    && cd ~/pai_ws \
    && source /opt/ros/humble/setup.bash && source ~/airobot_ws/install/setup.bash \
    && colcon build --symlink-install --packages-select physicalai_teleop

# named volume のマウント先を ubuntu 所有で事前作成（root 所有になるのを防ぐ）
RUN mkdir -p ~/final_task/ros2_ws/build ~/final_task/ros2_ws/install ~/final_task/ros2_ws/log \
             ~/pai_training/.pixi ~/dataset ~/checkpoints

COPY --chown=ubuntu:ubuntu bashrc_extra.sh /home/ubuntu/.bashrc_extra.sh
RUN echo 'source ~/.bashrc_extra.sh' >> ~/.bashrc

# ベースイメージの entrypoint は root で VNC 等を起動するため root に戻す
USER root
WORKDIR /
