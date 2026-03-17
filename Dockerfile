# ROS2 Humble with desktop (GUI) support for ME380
FROM ros:humble-ros-base

# Install desktop packages for RViz, Qt, and X11 GUI apps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xfixes0 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Optional: dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 in shell
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /me380

# Default: keep container running so you can exec in
CMD ["bash"]
