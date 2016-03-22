#!/bin/bash

# Delete all Process Folders
Prefix="Process_"

for i in $Prefix* ; do
	echo "Removing Folder : "$i
	rm -r ./$i
done


for i in "SimFiles" ; do
	echo "Removing Folder : "$i
	rm -r ./$i
done
