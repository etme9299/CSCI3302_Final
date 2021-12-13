# CSCI3302 Introduction to Robotics Final Project
Final Project for CSCI 3302 - Fall 2021

## Project Description

We have created a robotic system that leverages color detection, Webots camera features on a Drone, shortest path Algorithms, and robot odometry to allow a ground robot to navigate from point A to point B on uncharted land.
The Mavic robot has the responsibility of scanning (flyover) and mapping the built out maze as well as identifying both obstacles and target objects. The Mavic took an image of our map, and we used this pixel representation to perform matrix manipulations to better understand the world. We implemented color detection on an aerial image in order to transform our image into a workable matrix to perform path planning on. The ground robot moves based on the path planned and  has this ability based on its odometry and given waypoint calcuations from the Webots Mavic quadcopter. The ground robot is in a waiting state until prompted by the Mavik. Therefore we are utilizing the ability for robots to pass data from one to another.
 Applications to this final project could include autonomous robot driving based on real life obstacle data. In unsafe areas we can minimize human interaction by allowing robots to communicate and traverse. This project started with us pitching a completely different idea, but with some creativity we are proud to deliver this Drone/Ground-Robot system implementation!

## Software Prerequisites

 Python 3
 Webots
 

## List of Relevant Python Packages

```bash
import pdb
import pickle
import random
import copy
import cv2  
import numpy as np  
from simple_pid import PID
```

## Project Demonstration
