ARG BASE_IMAGE=ros:jazzy

FROM ${BASE_IMAGE} AS base

# ARG = ${}... ENV = $
ARG NEW_USER=vio_dev
ENV WS_DIR=/home/${NEW_USER}/vio_workspace
ENV LIBGL_ALWAYS_SOFTWARE=1

USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
    && apt-get install -y \
    python3 \
    tree \
    libopencv-dev \
    libeigen3-dev \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-cv-bridge \
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


# Setup ROS2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$NEW_USER/.bashrc

CMD ["/bin/bash"]