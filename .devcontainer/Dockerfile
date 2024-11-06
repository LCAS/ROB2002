ARG BASE_IMAGE=ros:humble

FROM ${BASE_IMAGE} as base

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    git \
    python3-pip \
    python3-rosdep \
    python3-matplotlib \
    python3-pandas \
    ros-humble-rviz2 \
    ros-humble-gazebo-plugins \
    ros-humble-nav2-* \
    ros-humble-rqt* \
    zenoh-bridge-ros2dds

# get the source tree and analyse it for its package.xml only
FROM base as sourcefilter
COPY ./src /tmp/src
# remove everything that isn't package.xml
RUN find /tmp/src -type f \! -name "package.xml" -print | xargs rm -rf

# install all dependencies listed in the package.xml
FROM base as depbuilder
# copy the reduced source tree (only package.xml) from previous stage
COPY --from=sourcefilter /tmp/src /tmp/src
RUN rosdep update && apt-get update
RUN cd /tmp/src && rosdep install --from-paths . --ignore-src -r -y && cd && rm -rf /tmp/src

FROM depbuilder as final
# add user ros with sudo rights if it doesn't exist
RUN if ! id ros; then \
        useradd -ms /bin/bash ros && echo "ros:ros" | chpasswd && adduser ros sudo; \
        echo "ALL ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers; \
        echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc; \
    else echo "user ros already exists"; \
    fi

# add sudo without password
ENV DEBIAN_FRONTEND=noninteractive

# add commit history to bust cache if and when needed.
ADD "https://api.github.com/repos/LCAS/limo_ros2/commits?per_page=1&sha=humble" .lcas_addons.limo_ros2.commits

COPY .devcontainer/*repos .devcontainer/*.sh /tmp/.devcontainer/
RUN bash /tmp/.devcontainer/install.sh

USER ros
