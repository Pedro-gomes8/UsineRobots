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
docker run osrf/ros:jazzy-desktop

docker ps --all
# Copy the ID from the container you have just created
docker commit <houno8nq0hw89nh0> <my-ros-2>

docker run -it <my-ros-2>
docker ps --all
# Copy the name from the new container
docker rename <quirky_sommet> ros

docker start -ia ros

docker ps --all
# Remove the unused container
docker rm <gracious_golick>
```

To come back to the last "VM", `docker start -ia ros`

Another option:
docker start ros 
docker exec -it ros bash # TO go back to it
