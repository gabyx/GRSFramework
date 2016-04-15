#!/bin/bash

#make all links of the studies!
# get dir of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


ln -s $GRSF_REPO_DIR_ADD/examples/jobs/simulationStudies/initCondition/avalanche1M-FillUp5/SimState.sim $DIR
ln -s $GRSF_REPO_MEDIA_DIR $DIR/media

