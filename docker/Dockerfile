ARG USERNAME=lc
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_VERSION=galactic
ARG BRANCH


FROM ghcr.io/priestofadanos/ros:$ROS_VERSION AS dep-install
ARG USERNAME
ARG USER_UID
ARG USER_GID


RUN apt-get update && apt-get install -y --no-install-recommends \
  wget nano build-essential libomp-dev clang lld git\
  ros-$ROS_VERSION-tf2-tools ros-$ROS_VERSION-tf-transformations \
  libeigen3-dev \
  libpcl-dev \
  libbluetooth-dev \
  ros-$ROS_VERSION-geodesy ros-$ROS_VERSION-pcl-ros ros-$ROS_VERSION-nmea-msgs \
  ros-$ROS_VERSION-sensor-msgs \
  ros-$ROS_VERSION-sensor-msgs-py \
  ros-$ROS_VERSION-rosbridge-suite \
  ros-$ROS_VERSION-laser-assembler \
  python3-pip python3-dev python3-setuptools python3-wheel

RUN pip3 install RPi.GPIO transforms3d bluedot
RUN mkdir -p /opt/$USERNAME/src

RUN apt-get clean \
  && rm -rf /var/lib/apt/lists/*

FROM dep-install as dep-compile
ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG ROS_VERSION
ARG BRANCH


WORKDIR /opt/$USERNAME/src

#TODO(PriestOfAdanos): should take local folder istead of some branch from git
RUN git clone https://github.com/PriestOfAdanos/lc_sensor_grabber.git -b $BRANCH 
RUN git clone https://github.com/Slamtec/rplidar_ros.git -b ros2 
RUN git clone https://github.com/ros-perception/laser_geometry.git -b ros2

COPY . /opt/$USERNAME/src/ndt_omp/

WORKDIR /opt/$USERNAME
RUN /bin/bash -c '. /opt/ros/$ROS_VERSION/setup.bash; colcon build'

FROM  dep-install AS final
ARG USERNAME
ARG USER_UID
ARG USER_GID
# ARG ROS_VERSION
# ARG BRANCH

RUN curl -fsSL https://deb.nodesource.com/gpgkey/nodesource.gpg.key | apt-key add -

USER $USERNAME
RUN curl -sL https://deb.nodesource.com/setup_16.x -o /tmp/nodesource_setup.sh
RUN sudo bash /tmp/nodesource_setup.sh
RUN sudo apt install nodejs
COPY --from=dep-compile /opt/$USERNAME/install /opt/$USERNAME

# get this to config file
WORKDIR /
COPY lc_entrypoint_scripts/* /opt/$USERNAME/scripts/

COPY ./production_entrypoint.sh /