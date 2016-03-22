#!/bin/bash

# terminate as early as possible
set -e

out=$(h5ls $1) 

if echo $out | grep -q "checkpoint" ; then
    printf "recover"
else
    printf "finished"
fi