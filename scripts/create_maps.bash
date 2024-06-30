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
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

printf "\n${PURPLE}***Script ID: create maps***${NC}\n"

######### DIRECTORIES & FILES #################################################
# main dataset dir, initially will have only the bags folder
DATASET_DIR=/workspace/datasets/BLT/
CW_PTH=catkin_ws

######### ENVIRONMENTAL VARIABLES  ############################################
OCTO_RESOLUTION=0.1 

MAPPING_FILTER_SIZE_MAP=0.3
MAPPING_FILTER_SIZE_SURF=0.3

MAPPING_FILTER_POINTS=4

ROSBAG_PLAY_RATE=2 #rosbag play rate

MAP_ODOM_TOPIC=/Odometry
IMU_TOPIC=/os_cloud_node/imu 
POINTS_TOPIC=/os_cloud_node/points

LIDAR_CFG_FILE=ouster16

######### LOG DIRECTORY #######################################################
# create log dir
LOG_DIR=$(pwd)/log
mkdir -p $LOG_DIR
#printf "log dir: ${BLUE}${LOG_DIR}${NC}\n"

###############################################################################
#parse bash args of they exists to override the default params 
while getopts D:C:r:o:m:s:L:p:f:I:P: flag
do
    case "${flag}" in
        D) DATASET_DIR=${OPTARG};;
        C) CW_PTH=${OPTARG};;
        r) ROSBAG_PLAY_RATE=${OPTARG};;
        o) OCTO_RESOLUTION=${OPTARG};;
        m) MAPPING_FILTER_SIZE_MAP=${OPTARG};;
        s) MAPPING_FILTER_SIZE_SURF=${OPTARG};;
        p) MAPPING_FILTER_POINTS=${OPTARG};;
        L) LOG_DIR=${OPTARG};;
        f) LIDAR_CFG_FILE=${OPTARG};;
        I) IMU_TOPIC=${OPTARG};;
        P) POINTS_TOPIC=${OPTARG};;        
    esac
done

printf "Args:\n"
printf "(D)ataset dir:        ${CYAN}${DATASET_DIR}${NC}\n"
printf "(C)atkin_ws dir:      ${CYAN}${CW_PTH}${NC}\n"
printf "(r)osbag play rate:   ${CYAN}${ROSBAG_PLAY_RATE}${NC}\n"
printf "(o)ctomap resolution: ${CYAN}${OCTO_RESOLUTION}${NC}\n"
printf "(m)ap filter size:    ${CYAN}${MAPPING_FILTER_SIZE_MAP}${NC}\n"
printf "(s)urf filter size:   ${CYAN}${MAPPING_FILTER_SIZE_SURF}${NC}\n"
printf "(p)oints filter num:  ${CYAN}${MAPPING_FILTER_POINTS}${NC}\n"
printf "(L)og dir:            ${CYAN}${LOG_DIR}${NC}\n"
printf "Lidar c(f)g file:     ${CYAN}${LIDAR_CFG_FILE}${NC}\n"
printf "(I)mu topic:          ${CYAN}${IMU_TOPIC}${NC}\n"
printf "(P)oints topic:       ${CYAN}${POINTS_TOPIC}${NC}\n"
printf "\n"

###############################################################################
# source the catkin_ws 
source $CW_PTH/devel/setup.bash
fastlio_pcd_dir=${CW_PTH}/src/FAST_LIO/PCD

# get rosbags list from the bags dir
ros_bags=($(ls ${DATASET_DIR}/bags))  

raw_data_dir=$DATASET_DIR/raw

pcd_dir=$raw_data_dir/pcd
octo_dir=$raw_data_dir/octo
map_odom_dir=$raw_data_dir/map_odom

mkdir -p $pcd_dir
mkdir -p $octo_dir
mkdir -p $map_odom_dir

# for every bag file, create pcd map, octo map, save the mapping trajectory 
for b in ${ros_bags[@]}; do
    printf "Processing: ${RED}${b}${NC} ...\n"
    bag_id=${b::-4}
    bag_dir=$DATASET_DIR/bags/$b

    printf "${CYAN}Running fast-lio + octomap server in background ... ${NC}\n"
    roslaunch fast_lio mapping_ktima_octo.launch \
                        filter_size_map:=${MAPPING_FILTER_SIZE_MAP}\
                        filter_size_surf:=${MAPPING_FILTER_SIZE_SURF}\
                        point_filter_num:=${MAPPING_FILTER_POINTS}\
                        lidar_cfg_file:=${LIDAR_CFG_FILE}\
                        points_topic:=${POINTS_TOPIC}\
                        octo_res:=${OCTO_RESOLUTION}&>$LOG_DIR/mapping_log.txt& 
    sleep 5   

    odom_bag=$map_odom_dir/$bag_id
    printf "${CYAN}Recording ${MAP_ODOM_TOPIC} to ${odom_bag}.bag${NC}\n"
    rosbag record -O $odom_bag $MAP_ODOM_TOPIC &>$LOG_DIR/rosbag_record.txt& 

    printf "${CYAN}Playing ${b} at rate of ${ROSBAG_PLAY_RATE}${NC}\n"
    rosbag play $bag_dir --clock -r $ROSBAG_PLAY_RATE \
                --topics $IMU_TOPIC $POINTS_TOPIC

    octo_map=$octo_dir/$bag_id.ot 
    printf "${CYAN}Requesting octomap to ${octo_map}${NC}\n"
    rosrun octomap_server octomap_saver -f ${octo_map} 

    rosnode kill -a # kill all the nodes 
    killall -9 roscore
    killall -9 rosmaster
    sleep 5

    printf "${CYAN}Saving ${MAP_ODOM_TOPIC} as .tum format${NC}\n"
    evo_traj bag $odom_bag.bag $MAP_ODOM_TOPIC --save_as_tum
    mv Odometry.tum $map_odom_dir/$bag_id.tum
    rm -r $odom_bag.bag

    pcd_map=$pcd_dir/$bag_id.pcd
    printf "${CYAN}Moving pcd map to ${pcd_map}${NC}\n"
    mv ${fastlio_pcd_dir}/scans.pcd ${pcd_map} #move map to the raw pcd folder 

    printf "======================================================\n"
done

printf "${GREEN}Done!${NC} \n"

