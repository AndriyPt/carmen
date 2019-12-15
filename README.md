# carmen

Carmen Robot Car project

[![License](https://img.shields.io/github/license/AndriyPt/carment.svg)](https://github.com/AndriyPt/carmen/blob/kinetic-devel/LICENSE)


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

