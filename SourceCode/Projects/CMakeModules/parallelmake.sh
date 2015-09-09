#!/bin/bash
cores=$(grep -c ^processor /proc/cpuinfo)

cores=$(($cores-4))

if [[ $cores -le 0 ]] ; then
    cores=1
fi

echo "Building with $cores cores!"
make -j$cores "$@" 
exit