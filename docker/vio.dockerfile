ARG BASE_IMAGE=ros:jazzy

FROM --platform=linux/amd64 ${BASE_IMAGE} AS base

# ARG = ${}... ENV = $
ARG NEW_USER=vio_dev
ENV WS_DIR=/home/${NEW_USER}/vio_workspace
# ENV RMW_IMPLEMENTATION==rmw_cyclonedds_cpp

USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
    && apt-get install -y \
    # iproute2 \
    # build-essential \
    # ca-certificates \
    # gnupg \
    # software-properties-common \
    # gcc \
    # g++ \
    # clang \
    # cmake \
    python3 \
    # wget \
    tree \
    # x11-xserver-utils \
    libopencv-dev \
    libeigen3-dev \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-cv-bridge \
    # ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    # ros-${ROS_DISTRO}-tf2-geometry-msgs \
    # This remembers to clean the apt cache after a run for size reduction
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Add the new_user to listed groups. And remove need for password
RUN adduser --disabled-password --gecos '' ${NEW_USER} \
&& echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
&& adduser ${NEW_USER} sudo


FROM base AS dev-ws

USER ${NEW_USER}
WORKDIR $WS_DIR
# RUN mkdir src
# RUN cd $WS_DIR/src && git clone https://github.com/ros2/rmw_cyclonedds ros2/rmw_cyclonedds -b ${ROS_DISTRO} \
# && git clone https://github.com/eclipse-cyclonedds/cyclonedds eclipse-cyclonedds/cyclonedds \
# && cd .. && rosdep update && rosdep install --from src -i &&  colcon build --symlink-install


# Setup ROS2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$NEW_USER/.bashrc

CMD ["/bin/bash"]