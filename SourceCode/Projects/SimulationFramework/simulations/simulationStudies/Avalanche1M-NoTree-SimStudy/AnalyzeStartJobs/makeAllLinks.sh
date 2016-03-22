#!/bin/bash

#make all links of the studies!
# get dir of this script

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Directors of this script: $DIR"

# Configuration Settings================================================
# Folder to the Simfiles of the Simulation Study
studyFolderTreeAnalyzeStartJob="$DIR/../../Avalanche1M-Tree-SimStudy/AnalyzeStartJob"
# ======================================================================

ln -s $studyFolderTreeAnalyzeStartJob/analyzerLogic $DIR/analyzerLogic
ln -s $studyFolderTreeAnalyzeStartJob/templates $DIR/templates
ln -s $studyFolderTreeAnalyzeStartJob/scripts $DIR/scripts
ln -s $studyFolderTreeAnalyzeStartJob/JobAnalyzeStart.ini $DIR/JobAnalyzeStart.ini
cp $studyFolderTreeAnalyzeStartJob/results/Plots.ipynb $DIR/results/Plots.ipynb

# execute data links
$DIR/data/makeAllLinks.sh


