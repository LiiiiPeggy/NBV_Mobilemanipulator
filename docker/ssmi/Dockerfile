# SSMI demo environment adapted from ExistentialRobotics/SSMI-example
# Official base: osrf/ros:noetic-desktop-full (Ubuntu 20.04) + Ignition Fortress
# Source is mounted at /workspace; image installs deps only (no COPY of project src).
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    IGNITION_VERSION=fortress \
    QT_X11_NO_MITSHM=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    SSMI_WS=/workspace/third_party/SSMI-Example/ssmi_ws

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    git \
    vim \
    lsb-release \
    python3-catkin-tools \
    python3-pip \
    curl \
    ca-certificates \
    python-is-python3 \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libpcl-dev \
    liboctomap-dev \
    ros-noetic-octomap \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-rviz-plugins \
    ros-noetic-pcl-ros \
    tmux \
    ros-noetic-teleop-twist-keyboard \
    python3-tk \
    libgflags-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ignition-fortress && \
    rm -rf /var/lib/apt/lists/*

# Network to rosdistro can time out; deps we need are installed via apt above.
RUN rosdep init 2>/dev/null || true && (rosdep update || true)

RUN pip3 install --no-cache-dir scikit-image

WORKDIR /workspace

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo 'if [ -f /workspace/third_party/SSMI-Example/ssmi_ws/devel/setup.bash ]; then source /workspace/third_party/SSMI-Example/ssmi_ws/devel/setup.bash; fi' >> /root/.bashrc

COPY entrypoint.sh /usr/local/bin/ssmi-entrypoint.sh
RUN chmod +x /usr/local/bin/ssmi-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/ssmi-entrypoint.sh"]
CMD ["bash"]
