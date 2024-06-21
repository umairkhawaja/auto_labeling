# Spatio-Temporal Auto Labelling System

We introduce a point cloud labelling system that generates spatio-temporal stability labels by analyzing point-to-point distance correspondence over time. This system requires at least two observations of the same environment taken at different times. The map labelling pipeline is illustrated in the figure below:

![ch2_labelling](https://github.com/ibrahimhroob/auto_labeling/assets/47870260/40f0c054-ac67-48cb-8c99-dbac7009a44c)

## Dependencies

To streamline the process, the system should ideally be packaged in a container. Meanwhile, follow these steps to install the necessary dependencies:

**Mapping System**: This system is compatible with any system capable of generating point cloud maps. In this work, [FAST-LIO](https://github.com/hku-mars/FAST_LIO) was used. To set it up, follow these instructions:
```bash
cd <your_cw>/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git
git clone https://github.com/ibrahimhroob/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
catkin build
```

**Required Libraries**: The system requires the OctoMap and PCL (>=1.8) libraries.
```bash
sudo apt install ros-$ROS_DISTRO-pcl-*
sudo apt install ros-$ROS_DISTRO-octomap-*
sudo apt install ros-$ROS_DISTRO-octovis   # Useful tool to visualize octomaps
```

**CloudCompare**: This tool is highly useful for visualizing point cloud data. It is also used for segmenting the ground plane using CSF. Ensure you have CloudCompare version >= 2.12 installed.

## Setup

To build the labelling system, execute the following commands:
```bash
git clone https://github.com/ibrahimhroob/auto_labeling.git
cd auto_labeling
mkdir build && cd build
cmake ..
make && make install
```

## Usage

Instructions for using the system will be added here.

## Publication

If you utilize our code in your academic work, please cite the following [paper](https://arxiv.org/pdf/2301.03426):

```bibtex
@inproceedings{hroob2024ias,
  author = {Hroob, I and Molina, S and Polvara, R and Cielniak, G and Hanheide, M},
  title = {{LTS-NET: End-to-End Unsupervised Learning of Long-Term 3D Stable Objects}},
  journal = {Proc. of Int. Conf. on Intelligent Autonomous Systems (IAS)},
  year = {2023},
}

@inproceedings{hroob2023ecmr,
  author    = {Hroob, I and Molina, S and Polvara, R and Cielniak, G and Hanheide, M},
  title     = {Learned Long-Term Stability Scan Filtering for Robust Robot Localisation in Continuously Changing Environments}, 
  booktitle = {Proc. of the Europ. Conf. on Mobile Robotics (ECMR)}, 
  year      = {2023},
}
```
