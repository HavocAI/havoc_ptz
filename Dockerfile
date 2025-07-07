# HavocOS currently runs on ROS Iron
FROM ghcr.io/havocai/ros:iron-ros-core-jammy as havocos-builder

# Switch to root user to install packages
USER root

# Set frontend to noninteractive to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install software-properties-common to manage repositories,
# then add the PPA for Python 3.8.
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update

# Install Python 3.8 and related tools
RUN apt-get install -y --no-install-recommends \
    python3.8 \
    python3.8-dev \
    python3.8-distutils \
    python3-pip \
    curl && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && \
    apt install \
    software-properties-common \
    pkg-config \
    sudo \
    curl \
    wget \
    git \
    build-essential \
    gdb \
    zstd \
    libssl-dev \
    libprotobuf-dev \
    libusb-1.0-0-dev \
    libhidapi-libusb0 \
    libhidapi-dev \
    libavformat-dev \
    libavcodec-dev \
    libswscale-dev \
    libavdevice-dev \
    libavutil-dev \
    libgpiod-dev \
    gpiod \
    clangd \
    clang-tools \
    clang-tidy-14 \
    clang-format \
    libboost-filesystem-dev \
    libboost-program-options-dev \
    libboost-atomic-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-python-dev \
    python3-colcon-common-extensions \
    libpq-dev \
    uuid \
    uuid-dev \
    gdal-bin \
    libgdal-dev \
    iproute2 \
    iputils-ping \ 
    net-tools \
    ros-iron-cv-bridge \
    ros-iron-geographic-msgs \
    ros-iron-geometry-msgs \
    ros-iron-tf2-geometry-msgs \
    ros-iron-mavros \
    ros-iron-foxglove-bridge \
    ros-iron-foxglove-msgs \
    ros-iron-rosbag2 \
    ros-iron-rqt \
    ros-iron-rviz2 \
    ros-iron-robot-state-publisher \
    ros-iron-camera-info-manager \
    ros-iron-camera-calibration-parsers \
    ros-iron-xacro \
    can-utils \
    xsltproc \
    dnsutils \
    unzip \
    nano \
    vim \
    ncdu \
    htop \
    netcat \
    openssh-server \ 
    protobuf-compiler && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

# Configure SSH access to the container
ENV HAVOC_SSH_PORT 22223
RUN echo "root:$(echo 'd3JlNGtzCg==' | base64 --decode)" | chpasswd && \
  mkdir -p /run/sshd && \
  rm -rf /etc/update-motd.d/* && \
  echo "" > /etc/legal && \
  sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/g' /etc/ssh/sshd_config

# Set python3.8 as the default python3
# Priority 2 makes it the default over the system's python3.10 (priority 1)
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 2

# Use curl to get the pip bootstrap script and install pip for Python 3.8
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.8

# Update ROS keys to latest
RUN echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
  && apt update \
  && apt install -y python3-colcon-common-extensions && \
  rm -rf /var/lib/apt/lists/*

# Switch back to the default ros user
USER ros