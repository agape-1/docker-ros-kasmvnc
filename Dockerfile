# Modified from https://github.com/brean/gz-sim-docker/blob/main/Dockerfile https://github.com/UNF-Robotics/docker-ros2-jazzy/blob/master/Dockerfile
# TODO: Integrate gzweb
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-core-noble
ENV COLCON_WS=/opt/ros_ws
ENV COLCON_WS_SRC=/opt/ros_ws/src
ENV PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"

ENV DEBIAN_FRONTEND=noninteractive

# update base system
RUN apt-get update && apt-get upgrade -y --no-install-recommends

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
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
	ros-${ROS_DISTRO}-slam-toolbox \
	ros-${ROS_DISTRO}-teleop-twist-keyboard \
	ros-${ROS_DISTRO}-xacro \
	&& rm -rf /var/lib/apt/lists/*

# install packages for dynamic websocket configuration
RUN apt-get update && apt-get install -y --no-install-recommends \
	xmlstarlet
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

ARG ENTRYPOINT=docker-entrypoint.sh
ARG WEBSOCKET_GZLAUNCH_FILE=websocket.gzlaunch
ARG GZ_SIM_OPTIONS=-s --headless-rendering
ARG WEBSOCKET_PORT=9002

COPY ${ENTRYPOINT} ${WEBSOCKET_GZLAUNCH_FILE} ./

RUN chmod +x ./${ENTRYPOINT} && xmlstarlet edit -L --update "//port" --value ${WEBSOCKET_PORT} ${WEBSOCKET_GZLAUNCH_FILE}

EXPOSE ${WEBSOCKET_PORT}

ENTRYPOINT ./${ENTRYPOINT} ${GZ_SIM_OPTIONS} ${WEBSOCKET_GZLAUNCH_FILE}
