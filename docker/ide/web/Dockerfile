FROM andriyp/carmen:latest

MAINTAINER "Andriy Petlovanyy"

LABEL Description="This ROS Kinetic image for Carmen development with Cloud 9 Web IDE" Vendor="Andriy Petlovanyy" Version="1.0"

ARG carmen_aurora_branch

ENV carmen_aurora_branch ${carmen_aurora_branch:-"master"}

ENV carmen_aurora_script "https://raw.githubusercontent.com/ros-ukraine/aurora/$carmen_aurora_branch/bin/run-ansible.sh"

ENV LOCAL_USER_ID 1000

ENV MY_USERNAME user

ENV DISPLAY :1.0

ENV QT_X11_NO_MITSHM 1

USER user

RUN set -x && \
    \
    echo "Downloading one-liner" && \
    wget -O /tmp/oneliner "$( echo "$carmen_aurora_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/oneliner && \
    \
    echo "Installing IDEs and GzWeb" && \
    /tmp/oneliner install_software --debug-branch $carmen_aurora_branch software=[cloud9,gzweb] && \
    \
    echo "Link models to gzweb folders" && \
    pushd /home/user/gzweb/http/client/assets && \
    ln -s /home/user/workspace/carmen/base/src/carmen/carmen_description ./carmen_description && \
    ln -s /home/user/workspace/carmen/base/src/carmen/carmen_gazebo ./carmen_gazebo && \
    popd && \
    \
    echo "Generating models for visualization" && \
    pushd /home/user/gzweb && \
    npm run deploy --- -m local && \
    popd && \
    \
    echo "Starting virtual X server" && \
    (nohup Xvfb :1 -screen 0 1024x768x16 &> /home/user/gzweb/xvfb.log &) && \
    \
    echo "Clean up" && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /root/.ansible /root/.gitconfig /root/.cache

USER root

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

EXPOSE 8080 8181 8282 8090 9090

CMD node /home/user/c9sdk/server.js --listen 0.0.0.0 --port 8181 -w /home/user/workspace/carmen/base/src/carmen/ -a :
