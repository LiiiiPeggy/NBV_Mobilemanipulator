# FKIE NBV Planner environment: Ubuntu 20.04 + ROS Noetic + Gazebo (classic)
# Source code is mounted at /workspace; this image only provides dependencies.
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    QT_X11_NO_MITSHM=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    gdb \
    curl \
    wget \
    vim \
    tmux \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    python3-catkin-tools \
    python3-pip \
    python3-rosdep \
    python-is-python3 \
    libcgal-dev \
    libeigen3-dev \
    libpcl-dev \
    liboctomap-dev \
    ros-noetic-gps-common \
    ros-noetic-octomap \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-server \
    ros-noetic-octomap-rviz-plugins \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    ros-noetic-tf2 \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-visualization-msgs \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-mbf-msgs \
    ros-noetic-move-base-flex \
    ros-noetic-depth-image-proc \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-actionlib-tools \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && (rosdep update || true)

WORKDIR /workspace

# Persist catkin products under the mounted project tree.
ENV CATKIN_WS=/workspace/ws_fkie

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo 'if [ -f /workspace/ws_fkie/devel/setup.bash ]; then source /workspace/ws_fkie/devel/setup.bash; fi' >> /root/.bashrc

COPY entrypoint.sh /usr/local/bin/fkie-entrypoint.sh
RUN chmod +x /usr/local/bin/fkie-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/fkie-entrypoint.sh"]
CMD ["bash"]
