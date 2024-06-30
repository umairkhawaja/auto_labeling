#!/bin/bash

cd catkin_ws/src/
git clone https://github.com/Livox-SDK/livox_ros_driver.git
git clone https://github.com/ibrahimhroob/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ..