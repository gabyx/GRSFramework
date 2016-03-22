#!/bin/bash
set -x
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

outputFolder="$DIR/../nuetzig/global"
echo "Find all xmls in $outputFolder"

find $outputFolder -type f -iname "*SimState*" -iname "*.xml" -exec bash -c "mv {} $DIR/" \;
