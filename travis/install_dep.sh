#!/bin/bash

set -e # exit on errors

cd $ROOT_PATH

export GRSF_CACHE_SIGNATURE_FILE="$GRSF_CACHE_DIR/GRSF_DEPS_CACHE_SUCCESSFUL"

# Install OpenMPI  =====================================================
sudo apt-get -y install openmpi-bin libopenmpi-dev

# Install eigen3 =======================================================
hg clone https://bitbucket.org/eigen/eigen/ ${ROOT_PATH}/eigen3 > /dev/null
cd ${ROOT_PATH}/eigen3 && hg update default
mkdir ${ROOT_PATH}/eigen3Build
cd ${ROOT_PATH}/eigen3Build
cmake ../eigen3 -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"
sudo make -j${BUILD_CORES}
sudo make -j${BUILD_CORES} install > /dev/null

# Install meta =========================================================
git clone https://github.com/ericniebler/meta.git ${ROOT_PATH}/meta > /dev/null
sudo cp -r ${ROOT_PATH}/meta/include/* /usr/local/include/
#ls -a /usr/local/include/meta

# large libraries
# check if the cache build signature file is here
if [  ! -f "$GRSF_CACHE_SIGNATURE_FILE" ] ; then

  echo "GRSF Build: Build only dependencies! and CACHE them"

  export BUILD_GRSF="OFF"
  export BUILD_DEPS="ON"

  # Install HDF5  ========================================================
  HDF5_DOWNLOAD_URL=https://www.hdfgroup.org/ftp/HDF5/releases/hdf5-1.8.15/src/hdf5-1.8.15.tar.gz
  #http://www.hdfgroup.org/ftp/HDF5/current/src/hdf5-1.8.17.tar.gz
  HDF5_BUILD=${ROOT_PATH}/hdf5Build
  mkdir -p ${HDF5_BUILD}
  wget --no-verbose --output-document="${ROOT_PATH}/hdf5.tar.gz" "$HDF5_DOWNLOAD_URL" > /dev/null
  cd ${HDF5_BUILD}
  tar zxf "${ROOT_PATH}/hdf5.tar.gz" --strip-components=1 -C "${HDF5_BUILD}" > /dev/null
  sudo ./configure --prefix=$INSTALL_PREFIX/hdf5 --enable-cxx > /dev/null
  sudo make -j${BUILD_CORES}
  sudo make -j${BUILD_CORES} install > /dev/null

  # Install pugixml  =====================================================
  git clone https://github.com/zeux/pugixml.git ${ROOT_PATH}/pugixml > /dev/null
  perl -pi -e 's/\/\/\s*#define\s*PUGIXML_HAS_LONG_LONG/#define PUGIXML_HAS_LONG_LONG/g' ${ROOT_PATH}/pugixml/src/pugiconfig.hpp
  mkdir ${ROOT_PATH}/pugixmlBuild
  cd ${ROOT_PATH}/pugixmlBuild
  cmake ../pugixml/scripts/ -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX > /dev/null
  sudo make -j${BUILD_CORES}
  sudo make -j${BUILD_CORES} install > /dev/null

  # Install boost ========================================================
  # Install newer boost
  BOOST_DOWNLOAD_URL="http://sourceforge.net/projects/boost/files/boost/1.58.0/boost_1_58_0.tar.bz2/download"
  BOOST_BUILD=${ROOT_PATH}/boostBuild
  mkdir -p ${BOOST_BUILD}
  wget --no-verbose --output-document="${ROOT_PATH}/boost.tar.bz2" "$BOOST_DOWNLOAD_URL" > /dev/null
  cd ${BOOST_BUILD}
  tar jxf "${ROOT_PATH}/boost.tar.bz2" --strip-components=1 -C "${BOOST_BUILD}" > /dev/null
  ./bootstrap.sh --with-libraries=system,thread,serialization,filesystem,chrono,atomic,date_time --prefix=$INSTALL_PREFIX > /dev/null
  sudo ./b2 -j${BUILD_CORES} threading=multi link=shared release install


  # Install Assimp   =====================================================
  cd ${ROOT_PATH}
  git clone https://github.com/assimp/assimp.git assimp > /dev/null
  mkdir -p ${ROOT_PATH}/assimpBuild
  cd ${ROOT_PATH}/assimpBuild
  cmake ../assimp -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH" > /dev/null
  sudo make -j${BUILD_CORES}
  sudo make -j${BUILD_CORES} install > /dev/null



  # Install OGRE 3d  =====================================================
  cd ${ROOT_PATH}
  git clone https://github.com/wgois/OIS.git OIS > /dev/null
  cd ${ROOT_PATH}/OIS
  sudo ./bootstrap > /dev/null
  sudo ./configure --prefix=$INSTALL_PREFIX > /dev/null
  sudo make -j${BUILD_CORES}
  sudo make -j${BUILD_CORES} install > /dev/null

  sudo apt-get -y install libxrandr-dev
  hg clone http://bitbucket.org/sinbad/ogre -u v1-9 ${ROOT_PATH}/ogre > /dev/null
  mkdir ${ROOT_PATH}/ogreBuild
  cd ${ROOT_PATH}/ogreBuild
  cmake ../ogre -DCMAKE_BUILD_TYPE=Release -DOGRE_BUILD_SAMPLES=OFF -DOGRE_BUILD_TESTS=OFF -DOGRE_BUILD_TOOLS=OFF -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"
  sudo make -j${BUILD_CORES}
  sudo make -j${BUILD_CORES} install > /dev/null

  # alternative
  #sudo apt-get -y install libogre-1.9.0 libogre-1.9-dev libois-1.3.0 libois-dev

  # make a signature file which marks successful cache build to check
  # copy all system stuff in local cache
  #sudo cp -r /usr/local/* $GRSF_CACHE_DIR

  echo "successful" | sudo tee -a $GRSF_CACHE_SIGNATURE_FILE > /dev/null
  echo "content in $GRSF_CACHE_DIR :"
  ls -al $GRSF_CACHE_DIR

else
  echo "GRSF Build: Use cached dependencies..."
  echo "content in $GRSF_CACHE_DIR :"
  ls -al $GRSF_CACHE_DIR/{bin,include,lib,share}
  #sudo cp -r $GRSF_CACHE_DIR /usr/local

  export BUILD_GRSF="ON"
  export BUILD_DEPS="OFF"

  #alternative boost
  #sudo apt-get  -y install libboost1.55-all-dev
  #alternative assimp
  #sudo apt-get -y install libassimp-dev
fi

# Clone ApproxMVBB  ====================================================
cd ${ROOT_PATH}
git clone https://github.com/gabyx/ApproxMVBB.git ApproxMVBB > /dev/null
export APPROXMVBB_REPO_DIR=${ROOT_PATH}/ApproxMVBB
