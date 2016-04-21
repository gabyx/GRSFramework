#!/bin/bash
shopt -s expand_aliases
source ~/.bash_aliases

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

fileRegexReplace -R ".*\.(svg)"  \
-r 's@(<svg.*?) width="(.*?)" height="(.*?)"@\1 width="100%" height="100%" viewBox="0 0 \2 \3" preserveAspectRatio="xMaxYMax"@' -f $DIR

fileRegexReplace -R ".*\.(svg)"  \
-r 's@-configureJob@configureJob@g' -f $DIR



