#!/bin/bash

#make all necessary links for this simple job
# get dir of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


ln -s $GRSF_REPO_DIR/additional/examples/jobs/simple/initCondition/SimState.sim $DIR
ln -s $DIR/../../../simple/SceneFileFunnel.xml $DIR/SceneFileFunnel.xml
ln -s $GRSF_REPO_MEDIA_DIR $DIR/

