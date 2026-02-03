# Fiber Navigation Simulation Environment
# ROS 2 Jazzy + Gazebo + Latest GCC (C++23)

FROM osrf/ros:jazzy-desktop

LABEL maintainer="me@pavelguzenfeld.com"
LABEL description="GPS-denied fixed-wing navigation simulation with cable odometry"

# Environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/root/ws

# Install latest GCC and dependencies
# Note: ROS 2 Jazzy uses Gazebo Harmonic via ros-gz packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-14 \
    g++-14 \
    ros-jazzy-ros-gz \
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

# Clone and build px4_msgs
RUN mkdir -p ${WORKSPACE}/src \
    && cd ${WORKSPACE}/src \
    && git clone https://github.com/PX4/px4_msgs.git --branch main \
    && cd ${WORKSPACE} \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --packages-select px4_msgs

# Copy project files
COPY . ${WORKSPACE}/src/fiber-nav-sim/

# Build workspace with C++23 (skip px4_msgs which is already built)
WORKDIR ${WORKSPACE}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && . ${WORKSPACE}/install/setup.sh \
    && colcon build --symlink-install \
       --packages-skip px4_msgs \
       --cmake-args -DCMAKE_CXX_STANDARD=23

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Expose ports
EXPOSE 11345
EXPOSE 14540/udp
EXPOSE 14550/udp

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
