# Turtlebot stack Installation guide

## Context

The Turtlebot2 has been around for years. Most tutorials online use very outdated versions of Ubuntu or Ros1. At the time, it was meant to hold a netbook, as Arm devices were not yet popular and accessible. Nowadays, we wish to use lighter and simpler Raspberry Pis. The easiest solution found was to install a docker image found on github and run it using QEMU, to translate the x86 instructions to Arm. We had to madify a little the dockerfile for it to run. 

## Stack
Materials:
* Turtlebot2
* Raspberry Pi 3 (1GiB)

Stack:
* Ubuntu 24.04 LTS on the Raspberry
* Ros2 iron
* [docker image](https://github.com/Felipe-noob/turtlebot2_ros2_rasp3)
* QEMU 

## Installation

Ros 2: Follow the official documentation.

QEMU:
```bash
sudo apt-get install qemu-system-x86 binfmt-support qemu-user-static
```

Docker: follow the official instruction or these:
```bash
sudo apt install docker.io docker-buildx
```

Docker image:
Beware this might take up to an hour to run. You could try running it on your desktop as a sort of "cross-compilation".
```bash
docker build -t ingot/turtlebot2-ros-iron:desktop -f turtlebot2_ros2.dockerfile --build-arg from_image=osrf/ros:iron-desktop --build-arg parallel_jobs=4 --platform linux/amd64 .
```
If you decide to compile on the desktop, you can transfer the image to the raspberry through a file.
```bash
docker save -o image.tar <image id>
```

After [transfering](https://stackoverflow.com/a/26226261/15000482) it to the Raspberry, load it. I recommend putting it on a USB drive instead ou the SD itself, so that you can do this in just 16GiB.
```bash
docker load -i image.tar
```
If you mess up, [this answer](https://stackoverflow.com/questions/45798076/how-to-clean-up-docker) tells you how to clean up docker and free up some space. 
