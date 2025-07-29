# HavocOS currently runs on ROS Iron
FROM ghcr.io/havocai/ros:iron-ros-core-jammy AS havocos-builder

# Switch to root user to install packages
USER root

# Set frontend to noninteractive to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# Typical shell vars
SHELL ["/bin/bash", "-c"]
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
ENV TERM=xterm-color

# Setup apt for non-interactive mode. Using ARG here instead of ENV to avoid the
# variable being persisted in the image.
ARG DEBIAN_FRONTEND=noninteractive
RUN echo 'APT::Install-Suggests "0";' >> /etc/apt/apt.conf.d/00-docker && \
  echo 'APT::Install-Recommends "0";' >> /etc/apt/apt.conf.d/00-docker && \
  echo 'APT::Get::Assume-Yes "true";' >> /etc/apt/apt.conf.d/00-docker

# Update ROS keys to latest
RUN rm /etc/apt/sources.list.d/ros2-latest.list && apt update && apt install curl

RUN echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
  && apt update \
  && apt install ros2-apt-source \
  && apt install -y python3-colcon-common-extensions && \
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
ENV HAVOC_SSH_PORT=22223
RUN echo "root:$(echo 'd3JlNGtzCg==' | base64 --decode)" | chpasswd && \
  mkdir -p /run/sshd && \
  rm -rf /etc/update-motd.d/* && \
  echo "" > /etc/legal && \
  sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/g' /etc/ssh/sshd_config

# Update ROS keys to latest
RUN echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
  && apt update \
  && apt install -y python3-colcon-common-extensions && \
  rm -rf /var/lib/apt/lists/*


#################################################################
#                   CAMERA CONTROL SPECIFIC                     #
#################################################################
# Install software-properties-common
# then add the PPA for Python 3.8.
RUN apt-get update
RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt update
# Install Python 3.8 and related tools
RUN apt-get install -y --no-install-recommends \
    python3.8 \
    python3.8-dev \
    python3.8-distutils \
    python3-pip \
    curl && \
    rm -rf /var/lib/apt/lists/*
# Install onvif_zeep
RUN python3.8 -m pip install onvif-zeep==0.2.12
RUN python3.8 -m pip install lxml==5.0.0
RUN python3.8 -m pip install pygame
RUN python3.8 -m pip install numpy==1.24.4
RUN python3.8 -m pip install zeep
# Copy all code to container
RUN mkdir -p /havoc/ptz 
COPY . /havoc/ptz
WORKDIR /havoc/ptz
RUN source /ros_entrypoint.sh 
# && \
    # colcon build --packages-select havoc_ptz --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# WORKDIR /
# RUN python3.8 setup.py build && \
#      python3.8 setup.py install

#################################################################
#                 END CAMERA CONTROL SPECIFIC                   #
#################################################################

# Make and switch to workdir
RUN mkdir -p /havoc/periferials
WORKDIR /havoc/periferials

# Switch back to the default ros user
# USER ros
# COPY run.sh /
CMD ["/bin/bash", "-c", "/havoc/ptz/utils/run.sh"]
