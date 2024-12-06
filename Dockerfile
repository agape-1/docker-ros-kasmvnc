# Modified from https://github.com/brean/gz-sim-docker/blob/main/Dockerfile https://github.com/UNF-Robotics/docker-ros2-jazzy/blob/master/Dockerfile https://github.com/Tiryoh/docker-ros2-desktop-vnc/blob/master/humble/Dockerfile
ARG UBUNTU_DISTRO=jammy
FROM ghcr.io/linuxserver/baseimage-kasmvnc:ubuntu${UBUNTU_DISTRO}
ARG ROS_DISTRO=humble

RUN apt-get update -q && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-ros-base \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep update

ENV COLCON_WS=/opt/ros_ws
ENV COLCON_WS_SRC=/opt/ros_ws/src
ENV PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"

ENV DEBIAN_FRONTEND=noninteractive

# update base system
RUN apt-get update && apt-get upgrade -y --no-install-recommends

# TODO: Remove and replace `$SLAM_TOOLBOX_PKG` with `ros-${ROS_DISTRO}-slam-toolbox` once `ros-rolling-slam-toolbox` is available in ROS index.
# Defaults to latest available slam-toolbox (jazzy) in ROS repository
RUN export SLAM_TOOLBOX_PKG=$([ '${ROS_DISTRO}' != 'rolling' ] && echo ros-${ROS_DISTRO}-slam-toolbox || echo ros-jazzy-slam-toolbox)

# install ros2 packages
RUN apt-get update && apt-get install -y -m --no-install-recommends \
	libusb-1.0-0-dev \
	python3-colcon-devtools \
	python3-colcon-package-selection \
	python3-colcon-ros \
	ros-dev-tools \
	ros-${ROS_DISTRO}-ament-lint-auto \
	ros-${ROS_DISTRO}-controller-manager \
	ros-${ROS_DISTRO}-joint-limits \
	ros-${ROS_DISTRO}-joint-state-publisher \
	ros-${ROS_DISTRO}-joy-linux \
	ros-${ROS_DISTRO}-joy-teleop \
	ros-${ROS_DISTRO}-robot-state-publisher \
	ros-${ROS_DISTRO}-ros2-controllers \
	ros-${ROS_DISTRO}-ros2launch \
	ros-${ROS_DISTRO}-rplidar-ros \
 	$SLAM_TOOLBOX_PKG  \ 
	ros-${ROS_DISTRO}-teleop-twist-keyboard \
	ros-${ROS_DISTRO}-xacro \
	&& rm -rf /var/lib/apt/lists/*

# ros 2 env
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "[[ -d /opt/ros_ws/install ]] && source /opt/ros_ws/install/setup.sh" \
	>> /root/.bashrc

# ros 2 workspace
RUN mkdir -p /opt/ros_ws/src

# common commands added to history
RUN echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard" \
        >> /root/.bash_history
RUN echo "source /opt/ros_ws/install/setup.sh" \
        >> /root/.bash_history
RUN echo "cd /opt/ros_ws" \
        >> /root/.bash_history

# Add dynamic arch determination from https://stackoverflow.com/questions/53048942/is-it-possible-to-get-the-architecture-of-the-docker-engine-in-a-dockerfile
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg\
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

ARG GZ_VERSION=harmonic

RUN apt-get update -qq \
    && apt-get install -y \
        gz-${GZ_VERSION} \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${COLCON_WS_SRC}\
    && cd ${COLCON_WS}\
    && . /opt/ros/${ROS_DISTRO}/setup.sh\
    && colcon build

# install packages for the following:
# dynamic websocket configuration
# Windows compatibility helper
# py-XDG fix: https://github.com/gfjardim/docker-containers/issues/51
RUN apt-get update && apt-get install -y --no-install-recommends \
	xmlstarlet \
	dos2unix \
	python3-xdg \
	&& rm -rf /var/lib/apt/lists/*

ARG ENTRYPOINT=docker-entrypoint.sh
# Fix: Allow `${ENTRYPOINT}` var accessible in `ENTRYPOINT` layer
ENV ENTRYPOINT=$ENTRYPOINT
ARG WEBSOCKET_GZLAUNCH_FILE=websocket.gzlaunch
ENV GZ_SIM_OPTIONS="-s --headless-rendering"
ENV WEBSOCKET_PORT=9002
ENV PUID=1000
ENV GUID=1000
ENV WEBSOCKET_GZLAUNCH_PATH=/${WEBSOCKET_GZLAUNCH_FILE}

COPY ${ENTRYPOINT} /

COPY ${WEBSOCKET_GZLAUNCH_FILE} ${WEBSOCKET_GZLAUNCH_PATH}

RUN chown ${PUID}:${GUID} ${WEBSOCKET_GZLAUNCH_PATH} \
	&& dos2unix /${ENTRYPOINT} \
	&& chmod +x /${ENTRYPOINT}
	
COPY /root /

EXPOSE ${WEBSOCKET_PORT}
