FROM arm64v8/ros:kinetic-ros-core-xenial

MAINTAINER "Andriy Petlovanyy"

LABEL Description="This ROS Kinetic image for Carmen for Raspberry PI" Vendor="Andriy Petlovanyy" Version="1.0"

ENV DISPLAY :1.0

ENV QT_X11_NO_MITSHM 1

COPY qemu-aarch64-static /usr/bin/

RUN set -x && \
    \
    echo "Changing shell to bash" && \
    ln -sf /bin/bash /bin/sh

RUN set -x && \
    \
    echo "Installing utilities" && \
    apt-get update && \
    apt-get install -y \
          wget \
          sudo \
          git && \
    \
    echo "Setting up ROS workspace" && \
    mkdir -p /root/workspace/carmen/base/src && \
    cd /root/workspace/carmen/base && \
    source /opt/ros/kinetic/setup.bash && catkin_init_workspace src && \
    source /opt/ros/kinetic/setup.bash && catkin_make && \
    \
    echo "Getting and compiling source code" && \
    pushd /root/workspace/carmen/base/src && \
    git clone https://github.com/AndriyPt/carmen.git && \
    popd && \
    source /opt/ros/kinetic/setup.bash && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y && \
    source /opt/ros/kinetic/setup.bash && catkin_make && \
    echo 'source /root/workspace/carmen/base/devel/setup.bash' >> /root/.bashrc && \     
    \
    echo "Clean up" && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /root/.ansible /root/.gitconfig /root/.cache
