ARG USERNAME=lc
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG BRANCH


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
  ros-galactic-rosbridge-suite \
  python3-pip python3-dev python3-setuptools python3-wheel

RUN pip3 install RPi.GPIO transforms3d bluedot
RUN mkdir -p /opt/$USERNAME/src

RUN apt-get clean \
  && rm -rf /var/lib/apt/lists/*

FROM dep-install as dep-compile
ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG BRANCH


WORKDIR /opt/$USERNAME/src

RUN git clone https://github.com/PriestOfAdanos/lc_sensor_grabber.git -b $BRANCH
RUN git clone https://github.com/Slamtec/rplidar_ros.git -b ros2 
RUN git clone https://github.com/ros-perception/laser_geometry.git -b ros2

COPY . /opt/$USERNAME/src/ndt_omp/

WORKDIR /opt/$USERNAME
RUN /bin/bash -c '. /opt/ros/galactic/setup.bash; colcon build'

FROM  dep-install AS final
ARG USERNAME
ARG USER_UID
ARG USER_GID

RUN curl -fsSL https://deb.nodesource.com/gpgkey/nodesource.gpg.key | apt-key add -

USER $USERNAME
RUN curl -sL https://deb.nodesource.com/setup_16.x -o /tmp/nodesource_setup.sh
RUN sudo bash /tmp/nodesource_setup.sh
RUN sudo apt install nodejs
COPY --from=dep-compile /opt/$USERNAME/install /opt/$USERNAME

# get this to config file
WORKDIR /
COPY lc_entrypoint_scripts/* /opt/$USERNAME/scripts/



ENTRYPOINT . /opt/lc/setup.bash && ros2  launch process_launchers scanner_nodes.launch.py
