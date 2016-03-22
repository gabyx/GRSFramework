#!/bin/bash
set -x
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

outputFolder="$DIR/../nuetzig/global"

simFolder="/media/zfmgpu/MyDataRaid/GabrielNuetzi/SimFilesEulerBrutus/SimulationStudies/SimFiles/Avalanche1M-Trees-Study"

echo "Find all xmls in $outputFolder"
find $outputFolder -type f -iname "*SimState*" -iname "*.xml" -exec bash -c "mv {} $DIR/" \;


# find all CombinedSimData.dat files in simFolder and link them into this folder
# sed: match only lines /../ , !d  delete if no match restart, otherwise p: print then s@...@...@ replace 
echo "Find all CombinedSimData.dat in $simFolder"
find $simFolder \( -type f -or -type l \) -and -iname "*CombinedSimData.dat.gz" \
  | sed -re '/.*?Study-P-([[:digit:]]*)\/\w*\.dat.gz/!d;p;s@@\1@'  \
  | xargs -n2 printf "ln -s %s CombinedSimData-P-%i.dat.gz ;\n" \
  | sh