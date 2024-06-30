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

printf "\n${PURPLE}***Script ID: extract scans***${NC}\n"

######### DIRECTORIES & FILES #################################################
DATASET_DIR=/home/ibrahim/ktima

######### ENVIRONMENTAL VARIABLES  ############################################
POINTS_TOPIC=/os_cloud_node/points

######### LOG DIRECTORY #######################################################
# create log dir
LOG_DIR=$(pwd)/log
mkdir -p $LOG_DIR
# printf "log dir: ${BLUE}${LOG_DIR}${NC}"

###############################################################################
#parse bash args of they exists to override the default params 
while getopts D:L: flag
do
    case "${flag}" in
        D) DATASET_DIR=${OPTARG};;
        L) LOG_DIR=${OPTARG};;
    esac
done

printf "Args:\n"
printf "(D)ataset dir:        ${CYAN}${DATASET_DIR}${NC}\n"
printf "(L)og dir:            ${CYAN}${LOG_DIR}${NC}\n"
printf "\n"

###############################################################################
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
CW_DIR="$(dirname "$SCRIPT_DIR")"/../catkin_ws
source $CW_DIR/devel/setup.bash

echo "catkin ws dir: $CW_DIR"

# Function to clean up and exit
cleanup() {
    echo -e "\n${RED}Ctrl+C captured! Cleaning up...${NC}"
    rosnode kill -a # kill all the nodes 
    killall -9 roscore
    killall -9 rosmaster
    printf "${GREEN}Cleanup done!${NC}\n"
    exit 1
}

# Trap SIGINT (Ctrl+C)
trap cleanup SIGINT

# get rosbags list from the bags dir
ros_bags=($(ls ${DATASET_DIR}/bags))  

raw_data_dir=$DATASET_DIR/raw
mkdir -p $raw_data_dir

# for every bag file, create pcd map, octo map, save the mapping trajectory and
# save the individual scans 
for b in ${ros_bags[@]}; do
    printf "Processing: ${RED}${b}${NC} ...\n"
    bag_id=${b::-4}
    bag_dir=$DATASET_DIR/bags/$b

    raw_scans_dir=$raw_data_dir/scans/$bag_id
    rm -r $raw_scans_dir
    mkdir -p $raw_scans_dir
    printf "${CYAN}Extracting raw scans to ${raw_scans_dir}${NC}\n"
    roscore&>$LOG_DIR/roscore.txt& 
    sleep 5
    rosrun bag_to_pcl_pcd bag_to_pcl_pcd $bag_dir $POINTS_TOPIC $raw_scans_dir
    rosnode kill -a # kill all the nodes 
    killall -9 roscore
    killall -9 rosmaster
    printf "======================================================\n"
done

printf "${GREEN}Done!${NC} \n"
