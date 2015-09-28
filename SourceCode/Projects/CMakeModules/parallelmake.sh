#!/bin/bash
cores=$(grep -c ^processor /proc/cpuinfo)

if [[ $cores -ge 8 ]] ; then
    cores=$(($cores-4))
fi

echo "Building with $cores cores!"
make -j$cores "$@" 
exit