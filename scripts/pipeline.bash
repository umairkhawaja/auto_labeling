#!/bin/bash

######### BASH TERMINAL COLORS ################################################
# Black        0;30     Dark Gray     1;30
# Red          0;31     Light Red     1;31
# Green        0;32     Light Green   1;32
# Brown/Orange 0;33     Yellow        1;33
# Blue         0;34     Light Blue    1;34
# Purple       0;35     Light Purple  1;35
# Cyan         0;36     Light Cyan    1;36
# Light Gray   0;37     White         1;37

RED='\033[0;31m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

######### DIRECTORIES & FILES #################################################
DATASET_DIR=/home/ibrahim/ktima
FASTLIO_CW_PTH=/home/ibrahim/neptune/catkin_ws

######### ENVIRONMENTAL VARIABLES  ############################################
#mapping params
OCTO_RESOLUTION=0.1
MAPPING_FILTER_SIZE_MAP=0.15
MAPPING_FILTER_SIZE_SURF=0.15
MAPPING_FILTER_POINTS=2
ROSBAG_PLAY_RATE=1 
LIDAR_CFG_FILE=ouster16
IMU_TOPIC=/os_cloud_node/imu 
POINTS_TOPIC=/os_cloud_node/points

#ground filter params
CLOTH_RESOLUTION=0.7  #Cloth resolution refers to the grid size (the unit is same as the unit of pointclouds) of cloth which is used to cover the terrain. The bigger cloth resolution you have set, the coarser DTM  you will get.
CLASS_THRESHOLD=0.2   #Classification threshold refers to a threshold (the unit is same as the unit of pointclouds) to classify the pointclouds into ground and non-ground parts based on the distances between points and the simulated terrain. 0.5 is adapted to most of scenes

#maps alignment params
DOWN_SAMPLE_FILTER_LEAF_SIZE=0.1

#ground truth features generator
OCTO_DEPTH_QUERY=13
FEATURES_TYPE=octo  #options: octo, raw {raw is not fully functional yet!}

#knn_dist_scan_to_map
KNN_DIS_SCAN_TO_MAP=0.3 #this distance should be equal or greater the MAPPING_FILTER_SIZE_*

######### LOG DIRECTORY #######################################################
# create log dir
LOG_DIR=$(pwd)/log
mkdir -p $LOG_DIR
printf "log dir: ${BLUE}${LOG_DIR}${NC}\n"

###############################################################################
#Main code is here
bash create_maps.bash  -D $DATASET_DIR \
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

# bash filter_ground.bash -D $DATASET_DIR  \
#                         -r $CLOTH_RESOLUTION \
#                         -t $CLASS_THRESHOLD

# bash tum_to_matrices.bash -D $DATASET_DIR

# bash registration.bash    -D $DATASET_DIR -l $DOWN_SAMPLE_FILTER_LEAF_SIZE=0.1

# bash features.bash        -D $DATASET_DIR -o $OCTO_DEPTH_QUERY -f $FEATURES_TYPE

# bash label_scans.bash     -D $DATASET_DIR -k $KNN_DIS_SCAN_TO_MAP

printf "\n${GREEN}Pipeline Finished!${NC} \n"
