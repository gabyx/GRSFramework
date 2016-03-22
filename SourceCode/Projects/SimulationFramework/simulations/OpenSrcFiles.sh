#!/bin/bash



#define all files which to open

folderpaths=("$SFSRC_DIR/SimulationFramework/CommonSource/" "$SFSRC_DIR/SimulationFramework/Projects/GeneralRigidBodySimulation" "$SFSRC_DIR/SimulationFramework/Projects/GeneralRigidBodySimulationMPI")

files=""

for i in "${folderpaths[@]}" ; do    
    echo "Adding all files in folder $i "    
    files+=" "$(find $i -type f -regextype posix-extended -regex '.*\.(hpp|icc)')
done
#echo $files

vim $files


