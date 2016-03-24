#!/bin/bash

TEMP=$(pwd)
cd ~/Desktop/Simulations/
shopt -s expand_aliases
source ~/.bash_aliases
source ./InitVariables.sh
echo $TEMP
cd $TEMP

activate3Delight
SimFileFolder="/media/zfmgpu/Data1/GabrielNuetzi/SimFiles/SimFilesBrutus/Avalanche1M-FillUp3/1"

SimFile="$SimFileFolder/SimStateTestRender.sim"
SceneFile="$SimFileFolder/SceneFile.xml"

run_simconv renderer \
-i \
$SimFile \
-r renderman \
-s $SceneFile \
-c ./RendermanMaterialCollection-Gap.xml -o ./output/FrameGap  
2>&1 | asanMangler
