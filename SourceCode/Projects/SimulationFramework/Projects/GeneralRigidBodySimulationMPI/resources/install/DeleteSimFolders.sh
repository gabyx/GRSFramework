#!/bin/bash


# Clean all Sim Folders
for dir in ./*
do
        dir=${dir%*/}
        dirName=${dir##*/}
        #echo ${dirName}
        if [[ "${dirName}" = Process* ]] 
        then
           #echo "Removing: " ${dirName}
           rm -r ${dirName}
        else
            if [[ "${dirName}" = SimFiles* ]]
            then
                #echo "Removing: " ${dir##*/}
                rm -r ${dirName}
            fi     
        fi
done

