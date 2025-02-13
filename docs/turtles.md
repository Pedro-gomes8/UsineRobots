# Setup guide for the turtle bots

## Hardware:

* TurtleBot2
* Raspberry Pi 3
* Portable battery
* WiFi

## Raspberry

* Ubuntu 24.04 LTS, ARM server edition
* [Connect to WiFi](https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line)

## Computer

* Ros2 - jazzy
```bash
docker run --name <felipe-ros2> osrf/ros:jazzy-desktop
docker ps --all
docker commit <houno8nq0hw89nh0> <test-ros-2>
docker run -it <test-ros-2>
docker ps --all
docker start -ia <quirky_sammet>
```

To come back to the last "VM", repeat the last step.
