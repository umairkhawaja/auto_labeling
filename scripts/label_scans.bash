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

printf "\n${PURPLE}***Script ID: labelling raw scans***${NC}\n"

######### DIRECTORIES & FILES #################################################
DATASET_DIR=/home/ibrahim/ktima

######### ENVIRONMENTAL VARIABLES  ############################################
KNN_DIS_SCAN_TO_MAP=0.3

######### LOG DIRECTORY #######################################################
# create log dir
LOG_DIR=$(pwd)/log
mkdir -p $LOG_DIR

###############################################################################
#parse bash args of they exists to override the default params 
while getopts D:L:k: flag
do
    case "${flag}" in
        D) DATASET_DIR=${OPTARG};;
        L) LOG_DIR=${OPTARG};;
        k) KNN_DIS_SCAN_TO_MAP=${OPTARG};;
    esac
done

printf "Args:\n"
printf "(D)ataset dir:        ${CYAN}${DATASET_DIR}${NC}\n"
printf "(L)og dir:            ${CYAN}${LOG_DIR}${NC}\n"
printf "(k)nn dis scan to map:${CYAN}${LOG_DIR}${NC}\n"
printf "\n"

###############################################################################
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
BINARY_DIR="$(dirname "$SCRIPT_DIR")"/bin

printf "Binary dir: ${BINARY_DIR}\n"

printf "${RED}Creating labelled scans for training ... ${NC}\n"
$BINARY_DIR/auto_labeling --dataset $DATASET_DIR -s --knn_dist_scan_to_map $KNN_DIS_SCAN_TO_MAP

printf "\n${GREEN}Done!${NC} \n"
