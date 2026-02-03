# Fiber Navigation Simulation Environment
# ROS 2 Humble + Gazebo + PX4 SITL

FROM px4io/px4-dev-ros2-humble:latest

LABEL maintainer="pavelguzenfeld@gmail.com"
LABEL description="GPS-denied fixed-wing navigation simulation with cable odometry"

# Environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PX4_HOME=/root/PX4-Autopilot
ENV WORKSPACE=/root/ws

# Install additional dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-message-filters \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-matplotlib \
    python3-pandas \
    python3-scipy \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for analysis
RUN pip3 install --no-cache-dir \
    numpy \
    matplotlib \
    pandas \
    scipy \
    seaborn \
    jupyter \
    plotly

# Clone and build PX4
WORKDIR /root
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git --branch v1.14.0 --depth 1 \
    && cd PX4-Autopilot \
    && make px4_sitl_default

# Build px4_msgs
RUN mkdir -p ${WORKSPACE}/src \
    && cd ${WORKSPACE}/src \
    && git clone https://github.com/PX4/px4_msgs.git --branch release/1.14 \
    && cd ${WORKSPACE} \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --packages-select px4_msgs

# Copy project files
COPY . ${WORKSPACE}/src/fiber-nav-sim/

# Build workspace
WORKDIR ${WORKSPACE}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && . ${WORKSPACE}/install/setup.sh \
    && colcon build --symlink-install

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Expose ports
# Gazebo
EXPOSE 11345
# PX4 SITL MAVLink
EXPOSE 14540/udp
EXPOSE 14550/udp
# ROS 2 DDS
EXPOSE 7400/udp
EXPOSE 7401/udp

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
