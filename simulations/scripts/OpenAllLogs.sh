#!/bin/bash

# Open all Logs in geany

SearchFolder=$1                                  # the search folder
RenameREGEX="/^.*ProcessMPI_\([[:digit:]]*\).*/!d;s//\1/"      #  get the number of the folder in a found path, /patter/d!;s//\1/,  delete all lines which do not mathc pattern and if they match extract particulat element 

#add all files to this list!
FileList=""


function addFiles(){
        filename=$1
        
        foundFiles=$(find $SearchFolder -name $filename)
        echo "Found files: " $foundFiles

        if [[ -z "$foundFiles" ]]; then
            echo "No file with name: $filename found in serach folder:" "$SearchFolder"
            return -1
        fi    


        for file in $foundFiles ; do
                echo "Found file: " $file
                #s=$(echo $file | sed -e "$RenameREGEX")
                #echo "Rename String: $s"
                #if [ -z $s ]; then
                #    continue
                #fi
                #dir=$(dirname $file)
                #newfile="$dir/${filename%.*}$s.log" #${filename%.*} extracts only the filename without suffix
                #if [[ $file != $newfile ]]; then
                #    echo "Copy file to: $newfile"
                #    cp $file $newfile
                #fi
                FileList+=" "$file
        done
}

addFiles "SimulationLog.log" && addFiles "MPISerializer.log"
killall -s 15 geany
 ##echo $FileList
geany $FileList
