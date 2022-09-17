ARG USERNAME=lc
ARG USER_UID=1000
ARG USER_GID=$USER_UID


FROM ghcr.io/priestofadanos/ros:galactic AS dep-install
ARG USERNAME
ARG USER_UID
ARG USER_GID

RUN apt-get update && apt-get install -y --no-install-recommends \
  wget nano build-essential libomp-dev clang lld git\
  ros-galactic-tf2-tools ros-galactic-tf-transformations \
  libeigen3-dev \
  libpcl-dev \
  libbluetooth-dev \
  ros-galactic-geodesy ros-galactic-pcl-ros ros-galactic-nmea-msgs \
  ros-galactic-sensor-msgs \
  ros-galactic-sensor-msgs-py \
  ros-galactic-octomap-mapping \
  python3-pip python3-dev python3-setuptools python3-wheel

RUN pip3 install RPi.GPIO transforms3d bluedot
RUN mkdir -p /opt/$USERNAME/src

RUN apt-get clean \
  && rm -rf /var/lib/apt/lists/*

FROM dep-install as dep-compile
ARG USERNAME
ARG USER_UID
ARG USER_GID

WORKDIR /opt/$USERNAME/src


RUN git clone https://github.com/PriestOfAdanos/lc_nodes.git
RUN git clone https://github.com/Slamtec/rplidar_ros.git -b ros2 
RUN git clone https://github.com/ros-perception/laser_geometry.git -b ros2
RUN git clone https://github.com/OctoMap/octomap_mapping.git -b ros2

COPY . /opt/$USERNAME/src/ndt_omp/

WORKDIR /opt/$USERNAME
RUN /bin/bash -c '. /opt/ros/galactic/setup.bash; colcon build'


FROM  dep-install AS final
ARG USERNAME
ARG USER_UID
ARG USER_GID

COPY --from=dep-compile /opt/$USERNAME/install /opt/$USERNAME/install

WORKDIR /

COPY lc_entrypoint_scripts/* /opt/$USERNAME/scripts/

USER $USERNAME


# ENTRYPOINT . /opt/lc/install/setup.bash && ros2  launch /opt/lc/launch_files/scanner_launch.py
