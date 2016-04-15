#!/bin/bash
# first  argument = path to where the virtual env is installed ./pythonenv3.4 for example
# second argument = number of cores to build


if [ -z "$1" ]; then
  echo "no install path given"
  exit -1
else
  installPath=$1
  echo "install path: $installPath"
fi

if [ -z "$2" ]; then
  nCores=1
else
  if [ "$2" -eq "$2" ] 2>/dev/null; then
    nCores=$2
    echo "number of cores: $nCores"
  else
    echo "number of cores invalid"
    exit -1
  fi

fi


if [ -z "${HDF5_ROOT}" ]; then
  echo "HDF5_ROOT variable is unset";
  exit -1
else
  echo "HDF5 root dir: $HDF5_ROOT"
fi

set -e

mkdir -p "$installPath"
mkdir -p "$installPath/tempDir"

echo "make virtual env in $installPath"
virtualenv --system-site-packages $installPath

echo "install modules:"
cd $installPathcd
pip=$installPath/bin/pip
python=$installPath/bin/python

source $installPath/bin/activate

cd $installPath/tempDir

$pip install cython

#numpy
git clone https://github.com/numpy/numpy.git -b maintenance/1.11.x --single-branch
cd numpy
$python setup.py build -j $nCores install
cd ..

$pip install matplotlib
$pip install scipy
$pip install scikit-image
$pip install ConfigArgParse
$pip install git+https://github.com/jsonpickle/jsonpickle.git
$pip install jsonschema
$pip install demjson
$pip install glob2
$pip install transforms3d
$pip install git+https://github.com/bcj/AttrDict.git
$pip install git+https://github.com/lxml/lxml.git@lxml-3.4

$pip install --install-option="--jobs=$nCores" pyside
$pip install graphviz
$pip install pyyaml
$pip install psutil


$pip install pycore

name="sip-4.17"
wget http://sourceforge.net/projects/pyqt/files/sip/sip-4.17/$name.tar.gz
tar xvf $name.tar.gz
rm $name.tar.gz
cd $name
$python configure.py
make -j$nCores install

name="PyQt-x11-gpl-4.11.4"
wget http://sourceforge.net/projects/pyqt/files/PyQt4/PyQt-4.11.4/$name.tar.gz
tar xvf $name.tar.gz
rm $name.tar.gz
cd $name
export QT_SELECT=qt4
$python configure.py
make -j$nCores
make -j$nCores install
cd ..

name="PyQt-gpl-5.5.1"
wget http://sourceforge.net/projects/pyqt/files/PyQt5/PyQt-5.5.1/$name.tar.gz
tar xvf $name.tar.gz
rm  $name.tar.gz
cd $name
export QT_SELECT=qt5
$python configure.py
make -j$nCores
make -j$nCores install
cd ..


$pip install PyOpenGL
$pip install git+https://github.com/vispy/vispy.git

$pip install -U ipython
$pip install -U jupyter
$pip install mpldatacursor


# Install h5py
git clone https://github.com/h5py/h5py.git
cd h5py
git checkout tags/2.6.0
$python setup.py configure --hdf5="$HDF5_ROOT"
$python setup.py install
cd ..

cd $installPath
rm -r $installPath/tempDir
echo "finished virtualenv install"
