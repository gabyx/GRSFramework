#!/bin/bash

#make all links of the studies!
# get dir of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


# join .sim file parts
cat $GRSF_REPO_ADD_DIR/examples/jobs/simulationStudies/initCondition/avalanche1M-FillUp5/SimState*  > $DIR/SimState.sim

ln -s $GRSF_REPO_MEDIA_DIR $DIR/media

