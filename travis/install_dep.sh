#!/bin/bash

set -e # exit on errors

cd $ROOT_PATH


# Install OpenMPI  =====================================================
sudo apt-get -y install openmpi-bin libopenmpi-dev

# Install HDF5  ========================================================

HDF5_DOWNLOAD_URL=http://www.hdfgroup.org/ftp/HDF5/current/src/hdf5-1.8.17.tar.gz
HDF5_BUILD=${ROOT_PATH}/hdf5Build
mkdir -p ${HDF5_BUILD}
wget --no-verbose --output-document="${ROOT_PATH}/hdf5.tar.gz" "$HDF5_DOWNLOAD_URL"
cd ${HDF5_BUILD}
tar zxf "${ROOT_PATH}/hdf5.tar.gz" --strip-components=1 -C "${HDF5_BUILD}"
sudo ./configure --prefix=/usr/local/hdf5 --enable-cxx 
sudo make
sudo make install
export PATH="$PATH:/usr/local/hdf5"

# Install eigen3 =======================================================
hg clone https://bitbucket.org/eigen/eigen/ ${ROOT_PATH}/eigen3
cd ${ROOT_PATH}/eigen3 && hg update default
mkdir ${ROOT_PATH}/eigen3Build
cd ${ROOT_PATH}/eigen3Build
cmake ../eigen3 -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX
sudo make VERBOSE=1 install
ls $INSTALL_PREFIX/lib/cmake

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
./bootstrap.sh --with-libraries=system,thread,serialization,filesystem,chrono,atomic,date_time
sudo ./b2 threading=multi link=shared release install

#alternative
#sudo apt-get  -y install libboost1.54-all-dev

# Install Assimp   =====================================================
cd ${ROOT_PATH}
git clone https://github.com/assimp/assimp.git assimp
mkdir -p ${ROOT_PATH}/assimpBuild
cd ${ROOT_PATH}/assimpBuild
cmake ../assimp 
sudo make VERBOSE=1 install

#alternative
#sudo apt-get -y install libassimp-dev

# Install OGRE 3d  =====================================================
#cd ${ROOT_PATH}
#git clone https://github.com/wgois/OIS.git OIS
#cd ${ROOT_PATH}/OIS
#sudo ./bootstrap
#sudo ./configure
#sudo make && sudo make install

#sudo apt-get -y install libxrandr-dev
#hg clone http://bitbucket.org/sinbad/ogre -u v1-9 ${ROOT_PATH}/ogre
#mkdir ${ROOT_PATH}/ogreBuild
#cd ${ROOT_PATH}/ogreBuild
#cmake ../ogre -DCMAKE_BUILD_TYPE=Release -DOGRE_BUILD_SAMPLES=OFF -DOGRE_BUILD_TESTS=OFF -DOGRE_BUILD_TOOLS=OFF
#sudo make VERBOSE=1 install 

# alternative
sudo apt-get -y install libogre-1.9.0 libogre-1.9-dev libois-1.3.0


# Clone ApproxMVBB  ====================================================
cd ${ROOT_PATH}
git clone https://github.com/gabyx/ApproxMVBB.git ApproxMVBB
export APPROXMVBB_REPO_DIR=${ROOT_PATH}/ApproxMVBB


