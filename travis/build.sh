#!/bin/bash
# "BUILD ========================================================================"


cd $ROOT_PATH
if [ -z "$BUILD_TYPE" ]; then export BUILD_TYPE=Release; fi





if [ "$BUILD_GRSF" == "ON" ]; then

  echo "Build GRSFramework..."
  cd $CHECKOUT_PATH

  ### init submodules
  #git submodule init
  #git submodule update
  ## only for hig perf. tests
  ##- cd addtional/tests/files; cat Lucy* | tar xz

  if [ ! -d ${ROOT_PATH}/build ]; then mkdir ${ROOT_PATH}/build; fi
  cd ${ROOT_PATH}/build

  echo " ApproxMVBB Repo Dir: ${APPROXMVBB_REPO_DIR}"

  cmake $CHECKOUT_PATH -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
                       -DGRSF_BUILD_GUI=$BUILD_GRSF_SIMGUI \
                       -DGRSF_BUILD_MPI=$BUILD_GRSF_SIMMPI \
                       -DGRSF_BUILD_NOGUI=$BUILD_GRSF_SIM \
                       -DGRSF_BUILD_SIMCONVERTER=$BUILD_GRSF_CONVERTER \
                       -DApproxMVBB_SEARCH_PATH=${APPROXMVBB_REPO_DIR} \
                       -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"
  make VERBOSE=1
  cd $ROOT_PATH

else
  echo "Do not build GRSFramework!"
fi

# "BUILD COMPLETE ================================================================"
