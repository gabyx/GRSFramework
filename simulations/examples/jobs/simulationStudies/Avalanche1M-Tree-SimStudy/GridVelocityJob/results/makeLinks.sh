#!/bin/bash
set -x
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

outputFolder="$DIR/../nuetzig/global"


echo "Find all folder with h5 files in $outputFolder"
mkdir -p "gridVelocities/all"
find "$outputFolder/GridVelocityAll.0" -type d -iname "*GridVelocity-P*" -exec bash -c "ln -s {} $DIR/gridVelocities/all" \;
find "$outputFolder/GridVelocity.0" -type d -iname "*GridVelocity-P*" -exec bash -c "ln -s {} $DIR/gridVelocities/rough" \;



images="/media/zfmgpu/MyDataRaid/GabrielNuetzi/ChuteExperimentsAdd/data/experiment5"
ln -s  "$images" "$DIR/experiment"

ln -s "$images/exp5-0002.jpg" $DIR/experimentBackground.jpg

treeMaskFile="/home/zfmgpu/Desktop/Repository/ChuteExperiments/data/experiment5/treemask.jpg"
ln -s $treeMaskFile $DIR

# make link to CIV images and grid files
# we match experiment 5
CIVResults="/media/zfmgpu/MyDataRaid/GabrielNuetzi/CIVExperiments/CIVExperiments-3Passes/CIV-Exp5-4Frames"
echo "Make links to CIV folder"
ln -s "$CIVResults/gridFiles" "$DIR/civGridFiles"
ln -s "$CIVResults/imageFiles" "$DIR/civImageFiles"