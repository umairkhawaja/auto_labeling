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

printf "\n${PURPLE}***Script ID: tum to matrices***${NC}\n"

######### DIRECTORIES & FILES #################################################
DATASET_DIR=/home/ibrahim/ktima

###############################################################################
#parse bash args of they exists to override the default params 
while getopts D: flag
do
    case "${flag}" in
        D) DATASET_DIR=${OPTARG};;
    esac
done

printf "Args:\n"
printf "(D)ataset dir:        ${CYAN}${DATASET_DIR}${NC}\n"
printf "\n"

###############################################################################
conda init bash
conda activate pn2_env

MAP_ODOM_DIR=$DATASET_DIR/raw/map_odom
RESULTS_DIR=$DATASET_DIR/raw/trajectories

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PY_SCRIPT_DIR="$(dirname "$SCRIPT_DIR")"/python_scripts


printf "${RED}Generating transformation matrices ... ${NC}\n"

# get rosbags list from the bags dir
trajectories=($(ls ${MAP_ODOM_DIR}))  

for t in ${trajectories[@]}; do
    python3 $PY_SCRIPT_DIR/tum_to_matrices.py --trajectory $MAP_ODOM_DIR/$t --results_dir $RESULTS_DIR
    printf "======================================================\n"
done

printf "${GREEN}Done!${NC} \n"
