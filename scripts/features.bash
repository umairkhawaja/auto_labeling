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

printf "\n${PURPLE}***Script ID: ground truth features generator***${NC}\n"

######### DIRECTORIES & FILES #################################################
DATASET_DIR=/home/ibrahim/ktima

######### ENVIRONMENTAL VARIABLES  ############################################
OCTO_DEPTH_QUERY=15
FEATURES_TYPE=octo  #options: octo, raw

######### LOG DIRECTORY #######################################################
# create log dir
LOG_DIR=$(pwd)/log
mkdir -p $LOG_DIR

###############################################################################
#parse bash args of they exists to override the default params 
while getopts D:L:o:f: flag
do
    case "${flag}" in
        D) DATASET_DIR=${OPTARG};;
        L) LOG_DIR=${OPTARG};;
        o) OCTO_DEPTH_QUERY=${OPTARG};;
        f) FEATURES_TYPE=${OPTARG};;
    esac
done

printf "Args:\n"
printf "(D)ataset dir:        ${CYAN}${DATASET_DIR}${NC}\n"
printf "(L)og dir:            ${CYAN}${LOG_DIR}${NC}\n"
printf "(o)ctomap depth query:${CYAN}${OCTO_DEPTH_QUERY}${NC}\n"
printf "(f)eatures type      :${CYAN}${FEATURES_TYPE}${NC}\n"
printf "\n"

###############################################################################
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
BINARY_DIR="$(dirname "$SCRIPT_DIR")"/bin

printf "Binary dir: ${BINARY_DIR}\n"

printf "${RED}Creating features maps ... ${NC}\n"
# $BINARY_DIR/auto_labeling --dataset $DATASET_DIR --depth_query 15 --features_raw --features_octo
$BINARY_DIR/auto_labeling --dataset $DATASET_DIR --depth_query $OCTO_DEPTH_QUERY --features_$FEATURES_TYPE

printf "\n${GREEN}Done!${NC} \n"
