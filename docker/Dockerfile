# Fiber Navigation Simulation Environment
# ROS 2 Jazzy + Gazebo Harmonic + PX4 SITL + Latest GCC (C++23)

FROM osrf/ros:jazzy-desktop

LABEL maintainer="me@pavelguzenfeld.com"
LABEL description="GPS-denied fixed-wing navigation simulation with cable odometry"
LABEL version="0.0.1"

# Environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/root/ws
ENV PX4_HOME=/root/PX4-Autopilot

# Install latest GCC and core dependencies
# Note: ROS 2 Jazzy uses Gazebo Harmonic via ros-gz packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-14 \
    g++-14 \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-message-filters \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-matplotlib \
    python3-pandas \
    python3-scipy \
    git \
    wget \
    curl \
    cmake \
    build-essential \
    ninja-build \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-launch-testing \
    ros-jazzy-launch-testing-ament-cmake \
    ros-jazzy-image-tools \
    ros-jazzy-rqt-image-view \
    ffmpeg \
    xvfb \
    mesa-utils \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Set GCC 14 as default
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 100

# Verify GCC version
RUN gcc --version && g++ --version

# Install Python packages for analysis
RUN pip3 install --no-cache-dir --break-system-packages \
    numpy \
    matplotlib \
    pandas \
    scipy \
    seaborn

# Clone and build px4_msgs + px4-ros2-interface-lib
RUN mkdir -p ${WORKSPACE}/src \
    && cd ${WORKSPACE}/src \
    && git clone https://github.com/PX4/px4_msgs.git --branch main \
    && git clone https://github.com/Auterion/px4-ros2-interface-lib.git --branch main --depth 1 \
    && cd ${WORKSPACE} \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --packages-select px4_msgs px4_ros2_cpp

# Install MicroXRCE-DDS Agent for PX4 <-> ROS 2 communication
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/micro-xrce-dds-agent \
    && cd /tmp/micro-xrce-dds-agent \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/micro-xrce-dds-agent

# PX4-Autopilot with native Gazebo Harmonic support
# Gazebo dev libs are provided by ros-jazzy-ros-gz-sim vendor packages;
# source ROS setup so PX4 cmake finds them via CMAKE_PREFIX_PATH.
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git ${PX4_HOME} --branch main --depth 1 \
    && cd ${PX4_HOME} \
    && bash Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && DONT_RUN=1 make px4_sitl gz_x500 \
    && rm -rf ${PX4_HOME}/build/px4_sitl_default/tmp \
    && rm -rf /tmp/*

# Copy custom airframe for quad-tailsitter with vision (GPS-denied)
COPY docker/airframes/4251_gz_quadtailsitter_vision ${PX4_HOME}/build/px4_sitl_default/etc/init.d-posix/airframes/4251_gz_quadtailsitter_vision
RUN chmod +x ${PX4_HOME}/build/px4_sitl_default/etc/init.d-posix/airframes/4251_gz_quadtailsitter_vision

# Set PX4 environment for Gazebo integration
ENV PX4_SIM_MODEL=quadtailsitter
ENV PX4_GZ_MODEL=quadtailsitter

# Copy project files
COPY . ${WORKSPACE}/src/fiber-nav-sim/

# Build workspace with C++23 (skip already-built packages and interface-lib examples)
WORKDIR ${WORKSPACE}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && . ${WORKSPACE}/install/setup.sh \
    && colcon build --symlink-install \
       --packages-skip px4_msgs px4_ros2_cpp px4_ros2_py \
       --packages-skip-regex 'example_.*' \
       --cmake-args -DCMAKE_CXX_STANDARD=23

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Expose ports
# Foxglove bridge
EXPOSE 8765
# Gazebo GUI
EXPOSE 11345
# PX4 MAVLink
EXPOSE 14540/udp
EXPOSE 14550/udp
# MicroXRCE-DDS
EXPOSE 8888/udp

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
