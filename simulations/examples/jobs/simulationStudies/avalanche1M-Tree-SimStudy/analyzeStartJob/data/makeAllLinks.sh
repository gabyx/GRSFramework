#!/bin/bash

#make all links of the studies!
# get dir of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Directors of this script: $DIR"

# Configuration Settings================================================
# Folder to the Simfiles of the Simulation Study
simFolder="${AVALANCHE1M_TREES_STUDY_SIMFOLDER}"
experimentSettingsFile="${CHUTE_EXPERIMENTS_REPO_DIR}/data/ExperimentSettings.json"
# ======================================================================

ln -s $experimentSettingsFile $DIR/

# find all SimFiles .sim file in simFolder and link them into this folder
find $simFolder -type f -iname "*.sim" | \
  sed -re 'p;s@.*?Study-P-([[:digit:]]*)\.([[:digit:]]).*@\1 \2@' | \
  xargs -n3 printf "ln -s %s $DIR/SimState-P-%i-%i.sim ; " | \
  sh

# scene file (take P-9, it does not matter)
sf="$(dirname $(readlink "$DIR/SimState-P-9-0.sim"))/SceneFile.xml"
cp $sf $DIR/SceneFile.xml