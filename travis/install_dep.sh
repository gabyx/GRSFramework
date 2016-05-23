#!/bin/bash

set -e # exit on errors

cd $ROOT_PATH


# Install OpenMPI  =====================================================
sudo apt-get -y install openmpi-bin libopenmpi-dev

# Install HDF5  ========================================================
sudo apt-get -y install libhdf5-serial-dev hdf5-tools


# Install eigen3 =======================================================
hg clone https://bitbucket.org/eigen/eigen/ ${ROOT_PATH}/eigen3
cd ${ROOT_PATH}/eigen3 && hg update 3.2
mkdir ${ROOT_PATH}/eigen3Build
cd ${ROOT_PATH}/eigen3Build
cmake ../eigen3 -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX
sudo make VERBOSE=1 install
ls -al $INSTALL_PREFIX/lib/cmake

# Install meta =========================================================
git clone https://github.com/ericniebler/meta.git ${ROOT_PATH}/meta
sudo cp -r ${ROOT_PATH}/meta/include/* $INSTALL_PREFIX/include/
#ls -a /usr/local/include/meta

# Install pugixml  =====================================================
git clone https://github.com/zeux/pugixml.git ${ROOT_PATH}/pugixml
perl -pi -e 's/\/\/\s*#define\s*PUGIXML_HAS_LONG_LONG/#define PUGIXML_HAS_LONG_LONG/g' ${ROOT_PATH}/pugixml/src/pugiconfig.hpp 
mkdir ${ROOT_PATH}/pugixmlBuild
cd ${ROOT_PATH}/pugixmlBuild
cmake ../pugixml/scripts/ -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX
sudo make VERBOSE=1 install

# Install boost ========================================================
 # Install newer boost
BOOST_DOWNLOAD_URL="http://sourceforge.net/projects/boost/files/boost/1.58.0/boost_1_58_0.tar.bz2/download"
BOOST_BUILD=${ROOT_PATH}/boostBuild
mkdir -p ${BOOST_BUILD}
wget --no-verbose --output-document="${ROOT_PATH}/boost.tar.bz2" "$BOOST_DOWNLOAD_URL"
cd ${BOOST_BUILD}
tar jxf "${ROOT_PATH}/boost.tar.bz2" --strip-components=1 -C "${BOOST_BUILD}"
./bootstrap.sh --with-libraries=system,thread,serialization
sudo ./b2 threading=multi link=shared release install > /dev/null


# Install OGRE 3d
sudo apt-get -y install libxrandr-dev
hg clone http://bitbucket.org/sinbad/ogre -u v1-9 ${ROOT_PATH}/ogre
mkdir ${ROOT_PATH}/ogreBuild
cd ${ROOT_PATH}/ogreBuild
cmake ../ogre -DCMAKE_BUILD_TYPE=Release
sudo make VERBOSE=1 install > /dev/null



