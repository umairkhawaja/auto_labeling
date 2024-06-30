#!/bin/bash

# Remove the build directory if it exists
rm -r build

# Create a new build directory and navigate into it
mkdir build && cd build

# Generate build files using CMake
cmake ..

# Compile the project and install the binaries
make && make install

# Remove the build directory as it is not required 
cd .. && rm -r build

# Navigate to the catkin workspace
cd catkin_ws

# Clean the catkin workspace
catkin clean -y

# Build the catkin workspace
catkin build

# Source the setup script to update the environment
source devel/setup.bash