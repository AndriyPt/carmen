# carmen

Carmen Robot Car project

[![License](https://img.shields.io/github/license/AndriyPt/carmen.svg)](https://github.com/AndriyPt/carmen/blob/kinetic-devel/LICENSE)

# Table of Content
- [Linux](#linux)
- [Windows](#windows)
- [Raspberry Pi](#raspberry-pi)

# Linux

## Docker

For convenience it is recommended to use Docker containers.
Please follow these steps to run Docker container on your machine.

 1. Install Desktop OS Ubuntu Trusty or Xenial on your machine or in virtual machine
 2. Install Docker-CE using these [instructions](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)
 3. In order to executed Docker without sudo please execute
```bash
sudo usermod -aG docker $USER
```
 4. Logout and login to your machine again :)
 5. In case if you have NVidia graphic card customized Docker could be installed which will utilize your GPU. Please follow [these extra steps](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-1.0)).
 6. For development [the following](https://hub.docker.com/repository/docker/andriyp/carmen) docker image will be used for NVidia Docker [this one](https://hub.docker.com/repository/docker/andriyp/carmen-dev-nvidia).
 7. Use the following command to start ordinary Docker container
```bash
docker run -it --name carmen_dev -p 8080:8080 -p 8090:8090 -p 9090:9090 -e DISPLAY -e LOCAL_USER_ID=$(id -u) -v /tmp/.X11-unix:/tmp/.X11-unix:rw andriyp/carmen:latest
```
for NVidia Docker please use
```bash
nvidia-docker run -it --name carmen_dev -p 8080:8080 -p 8090:8090 -p 9090:9090 -e DISPLAY -e LOCAL_USER_ID=$(id -u) -v /tmp/.X11-unix:/tmp/.X11-unix:rw andriyp/carmen-dev-nvidia:latest
```
 8. Black window of [Terminator](https://gnometerminator.blogspot.com/p/introduction.html) UI console will appear after some time.
 9. You can use it's features to [split terminal window](https://linux.die.net/man/1/terminator) into smaller terminals and run few commands in parallel (Ctrl+Shift+E).
 10. If you want to run real robot add user to dialout group and restart Docker container
```bash
sudo usermod -a -G dialout user
```

In order to relaunch docker container after you closed Terminator window or rebooted machine please run
```bash
docker start carmen_dev
```
and for NVidia Docker
```bash
nvidia-docker start carmen_dev
```

After some time Terminator window will reappear.

## IDEs

In case if you want to run PyCharm in Docker container please run

```bash
pycharm
```

To launch QtCreator please run

```bash
qtcreator
```

For VSCode type

```bash
vscode
```

## URDF and RViz
In order to debug URDF please launch

```bash
roslaunch carmen_launch view_urdf.launch
```

To have a look on the state of the robot in RViz run

```bash
roslaunch carmen_launch rviz.launch
```

## Hardware

In order to launch ROS Serial node which will be communicating with MCU run the following command
```bash
roslaunch carmen_launch hardware.launch
```
Default port is */dev/ttyACM0* and baud is *115200*.

In order to run command with custom port and\or baud please use parameters to pass these values e.g.
```bash
roslaunch carmen_launch hardware.launch port:=/dev/ttyUSB0 baud:=57600
```

Messages from MCU could be received using [rostopic](http://wiki.ros.org/rostopic#rostopic_command-line_tool) command.

In case if sensor reading need to checked from MCU.
You need to know in which topic there readings are published.
Let's assume it is */mcu_sensor_data*.
Please start hardware nodes as described above.
After that please run this command
```bash
rostopic echo /mcu_sensor_data
```

In order to publish messages to MCU please use **rostopic pub** command.

Header files for messages could be found in [this](https://github.com/AndriyPt/carmen/tree/kinetic-devel/carmen_hardware/firmware/rosserial) folder.

The following command will regenerate these headers with most recent changes in message files
```bash
rosrun carmen_hardware generate_messages.sh
```


# Windows

## Docker Desktop

For OS Windows it is recommended to use Docker Desktop containers.
Please follow these steps to run Docker container on your machine.

 1. Install Windows 10 on your machine or in virtual machine
 2. Install Docker Desktop using these [instructions](https://docs.docker.com/docker-for-windows/install/)
 3. For development [the following](https://hub.docker.com/repository/docker/andriyp/carmen-dev-web) docker image will be used.
 4. Use the following command to start ordinary Docker container
```powershell
docker run -d --name carmen_dev -p 8080:8080 -p 8181:8181 -p 8282:8282 -p 8090:8090 -p 9090:9090 andriyp/carmen-dev-web:latest
```
 5. Command will spawn Docker container and exit.

In order to relaunch docker container please run
```bash
docker start carmen_dev
```

## IDEs

In Docker Desktop only Cloud9 web IDE is available.
Open http://localhost:8181 in your browser.

## Visualization

Run the following command in Cloud9 web IDE terminal windows
```bash
roslaunch carmen_launch web_server.launch
```
Open web browser and go to [WebViz application page](https://webviz.io/app/).
It will connect to your local web socket server running in Docker container.
You will be able to plot data, view rosout and more.

# Raspberry PI

## Docker

Please follow these steps to run Docker container on your machine.

 1. Install Raspbian on your machine
 2. Install Docker using these [instructions](https://docs.docker.com/install/linux/docker-ce/debian/#install-docker-engine---community-1)
 3. For development [the following](https://hub.docker.com/r/andriyp/carmen-dev-rpi) docker image will be used.
 4. Use the following command to start ordinary Docker container
 for ARM 32 bit
```bash
docker run -d --name carmen_dev -p 8080:8080 -p 8181:8181 -p 8282:8282 -p 8090:8090 -p 9090:9090 andriyp/carmen-dev-rpi:armhf
```
 for ARM 64 bit
```bash
docker run -d --name carmen_dev -p 8080:8080 -p 8181:8181 -p 8282:8282 -p 8090:8090 -p 9090:9090 andriyp/carmen-dev-rpi:arm64
```
 5. Command will spawn Docker container and exit.

In order to relaunch docker container please run
```bash
docker start carmen_dev
```

## QEMU Docker

In case if you want to test Docker containers on Linux using QEMU install Docker as it is described in Linux section steps (1-4).
After that run the following steps:
 1. Execute command
 ```bash
 docker run --rm --privileged multiarch/qemu-user-static:register --reset
 ```
  2. Run container
  for ARM 32 bit
```bash
 docker run -it --name carmen_dev -p 8080:8080 -p 8181:8181 -p 8282:8282 -p 8090:8090 -p 9090:9090 --entrypoint /usr/bin/qemu-arm-static andriyp/carmen-dev-rpi:armhf /bin/bash 
```
  for ARM 64 bit
```bash
 docker run -it --name carmen_dev -p 8080:8080 -p 8181:8181 -p 8282:8282 -p 8090:8090 -p 9090:9090 --entrypoint /usr/bin/qemu-aarch64-static andriyp/carmen-dev-rpi:arm64 /bin/bash 
```

## IDEs

In Docker Desktop only Cloud9 web IDE is available.
Open http://localhost:8181 in your browser.

## Visualization

Run the following command in Cloud9 web IDE terminal windows
```bash
roslaunch carmen_launch web_server.launch
```
Open web browser and go to [WebViz application page](https://webviz.io/app/).
It will connect to your local web socket server running in Docker container.
You will be able to plot data, view rosout and more.
In case if you want to see data running on remote Raspberry PI please click on Help sign in WebViz page to see how to change WebViz URL.
