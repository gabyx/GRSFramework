#!/bin/bash

# terminate as early as possible
set -e

out=$(exrinfo $1) 

if echo $out | grep -q "checkpoint" ; then
    printf "recover"
else
    printf "finished"
fi