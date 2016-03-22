#!/bin/bash

#make all links of the studies!
# get dir of this script

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Directors of this script: $DIR"

# execute data links
$DIR/data/makeAllLinks.sh


