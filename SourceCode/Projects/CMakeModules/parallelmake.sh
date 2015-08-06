#!/bin/bash
cores=$(grep -c ^processor /proc/cpuinfo)

if [[ $cores -ge 2 ]] ; then
    cores=$(($cores-1))
fi

echo "Building with $cores cores!"
make -j$cores "$@" 
exit