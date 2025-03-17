# UsineRobots



## Robotic Production Line Simulation

This repository contains a ROS-based setup of a robotic production line featuring:

Two Niryo Ned 2 robots
TurtleBot mobile robots
Intel RealSense D405 depth camera
OpenCV for object detection and color-based classification

The system simulates an end-to-end workflow where objects of different colors are deposited, sorted, and transported between two robotic arms via TurtleBots, culminating in color-based placement.

## Overview

### First Niryo Ned 2 (Robot 1)

Picks objects from a deposit spot/input tray
Classifies them by color
Places each object on a TurtleBot.

## TurtleBot

The TurtleBot carries the object through a corridor to the second robotic arm’s station.
Arrives at the delivery spot near the second arm.

### Second Niryo Ned 2 (Robot 2) + Intel RealSense D405

A fixed, mounted D405 camera identifies the object’s precise position and color.
Robot 2 picks the object from the TurtleBot, then places it at the designated color location.

### ROS Coordinator
Manages communication between Robot 1, Robot 2, and the TurtleBot.
Ensures tasks execute in correct order.
System Architecture

