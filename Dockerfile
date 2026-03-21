# ROS2 Humble with desktop (GUI) support for ME380
FROM ros:humble-ros-base

# Install desktop packages for RViz, Qt, X11 GUI apps, and xacro
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
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

# Optional: dev tools + basic networking (ip, ping)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    iproute2 \
    iputils-ping \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Python deps for ik.py (inverse kinematics) and plotting
RUN pip3 install --no-cache-dir ikpy matplotlib

# Source ROS2 in shell
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /me380

# Default: keep container running so you can exec in
CMD ["bash"]
