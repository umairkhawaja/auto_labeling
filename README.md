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

To build the labelling system, simply use the provided build script:
```bash
git clone https://github.com/ibrahimhroob/auto_labeling.git
cd auto_labeling
bash build.sh
```

## Usage

The system accepts `rosbags` of the environment located in the `bags` directory within your dataset folder:

```bash
DATASET/
├── bags
│   ├── 0.bag
│   ├── 1.bag
```

To streamline the entire process, we provide a single bash script that includes all the steps for the labelling pipeline along with the necessary hyperparameters. You can find this script in the [pipeline.bash](https://github.com/ibrahimhroob/auto_labeling/blob/main/scripts/pipeline.bash) file inside the scripts folder. The most important parameters to set are [DATASET_DIR](https://github.com/ibrahimhroob/auto_labeling/blob/0241328d264e696441a6fa223c2bd7228f51ead4/scripts/pipeline.bash#L20) and [FASTLIO_CW_PTH](https://github.com/ibrahimhroob/auto_labeling/blob/0241328d264e696441a6fa223c2bd7228f51ead4/scripts/pipeline.bash#L21C1-L21C15). Please update these parameters based on your dataset and `catkin_ws` directories. Here are the main steps in sequence, which you can comment or uncomment based on your testing and requirements:

```bash
# Main code
bash create_maps.bash -D $DATASET_DIR \
                      -C $FASTLIO_CW_PTH \
                      -o $OCTO_RESOLUTION \
                      -r $ROSBAG_PLAY_RATE \
                      -m $MAPPING_FILTER_SIZE_MAP \
                      -s $MAPPING_FILTER_SIZE_SURF \
                      -p $MAPPING_FILTER_POINTS \
                      -f $LIDAR_CFG_FILE \
                      -I $IMU_TOPIC \
                      -P $POINTS_TOPIC

# bash extract_scans.bash -D $DATASET_DIR

# bash filter_ground.bash -D $DATASET_DIR \
#                         -r $CLOTH_RESOLUTION \
#                         -t $CLASS_THRESHOLD

# bash tum_to_matrices.bash -D $DATASET_DIR

# bash registration.bash -D $DATASET_DIR -l $DOWN_SAMPLE_FILTER_LEAF_SIZE=0.1

# bash features.bash -D $DATASET_DIR -o $OCTO_DEPTH_QUERY -f $FEATURES_TYPE

# bash label_scans.bash -D $DATASET_DIR -k $KNN_DIS_SCAN_TO_MAP
```


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
