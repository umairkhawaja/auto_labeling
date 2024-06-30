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

printf "\n${PURPLE}***Script ID: filter ground***${NC}\n"

######### DIRECTORIES & FILES #################################################
DATASET_DIR=~/workspace/datasets/auto_labeler/

######### ENVIRONMENTAL VARIABLES  ############################################
CLOTH_RESOLUTION=0.7
CLASS_THRESHOLD=0.2

POINTS_TOPIC=/os_cloud_node/points

######### LOG DIRECTORY #######################################################
# create log dir
LOG_DIR=$(pwd)/log
mkdir -p $LOG_DIR
# printf "log dir: ${BLUE}${LOG_DIR}${NC}"

###############################################################################
#parse bash args of they exists to override the default params 
while getopts D:r:t:L: flag
do
    case "${flag}" in
        D) DATASET_DIR=${OPTARG};;
        r) CLOTH_RESOLUTION=${OPTARG};;
        t) CLASS_THRESHOLD=${OPTARG};;
        L) LOG_DIR=${OPTARG};;
    esac
done

printf "Args:\n"
printf "(D)ataset dir:        ${CYAN}${DATASET_DIR}${NC}\n"
printf "cloth (r)esolution:   ${CYAN}${CLOTH_RESOLUTION}${NC}\n"
printf "class (t)hreshold:    ${CYAN}${CLASS_THRESHOLD}${NC}\n"
printf "(L)og dir:            ${CYAN}${LOG_DIR}${NC}\n"
printf "\n"

###############################################################################
raw_data_dir=$DATASET_DIR/raw
pcd_data_dir=$raw_data_dir/pcd

ground_dir=$raw_data_dir/ground
off_ground_dir=$raw_data_dir/off_ground

mkdir -p $ground_dir
mkdir -p $off_ground_dir

# get pcd maps list from the pcd dir
pcd_maps=($(ls ${pcd_data_dir}/*.pcd))

for pcd_map_path in ${pcd_maps[@]}; do
    pcd_map=$(basename "$pcd_map_path")
    printf "Processing: ${RED}${pcd_map}${NC} ...\n"
    map_id=${pcd_map%.pcd}
    ground_bin=${pcd_data_dir}/${map_id}_ground_points.bin
    offground_bin=${pcd_data_dir}/${map_id}_offground_points.bin

    # Check if ground and offground bin files already exist
    if [ -f "$ground_bin" ] && [ -f "$offground_bin" ]; then
        printf "Skipping ${RED}${pcd_map}${NC} as it has already been processed.\n"
    else
        flatpak run org.cloudcompare.CloudCompare -SILENT -O $pcd_map_path -CSF -SCENES SLOPE -CLOTH_RESOLUTION $CLOTH_RESOLUTION -CLASS_THRESHOLD $CLASS_THRESHOLD -EXPORT_GROUND -EXPORT_OFFGROUND
    fi

    ground_pcd=$ground_dir/$pcd_map
    off_ground_pcd=$off_ground_dir/$pcd_map

    flatpak run org.cloudcompare.CloudCompare -SILENT -O ${ground_bin} -C_EXPORT_FMT PCD -SAVE_CLOUDS FILE $ground_pcd
    flatpak run org.cloudcompare.CloudCompare -SILENT -O ${offground_bin} -C_EXPORT_FMT PCD -SAVE_CLOUDS FILE $off_ground_pcd
    rm ${ground_bin}
    rm ${offground_bin}
    printf "======================================================\n"
done

#remove the full pcd maps to save space
#rm -r $pcd_data_dir

printf "${GREEN}Done!${NC} \n"