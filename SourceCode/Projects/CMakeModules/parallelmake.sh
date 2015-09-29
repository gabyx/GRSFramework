#!/bin/bash

SS="parallelmake.sh"

coresToUse=1

ARGS="$@ --"
eval set -- "$ARGS" ;
while true ; do
    case "$1" in
        -j) coresToUse="$2"; shift 2 ;
        ;;
        --) shift; break ; 
        ;;
        *) arg_rest+=( "$1" ) ; shift ;
        ;;
    esac
done


# ignore cores (e.g. -j 12), set them according to the cpu information!
maxCores=$(grep -c ^processor /proc/cpuinfo)
coresToUse=$(bc <<< "((0.5*$maxCores) + 0.5)/1" ) # cheap hack to round to int

echo "Building with $coresToUse cores!"
echo "passing rest arguments to make: ${arg_rest[@]}"
make -j $coresToUse "${arg_rest[@]}"
exit