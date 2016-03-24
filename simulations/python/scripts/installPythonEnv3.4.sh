#!/bin/bash
# first  argument = path to where the virtual env is installed ./pythonenv3.4 for example


if [ -z "$1" ]; then
  echo "No install path given"
  exit -1
fi

installPath=$(readlink -e $1)


if [ -z "$HDF5_ROOT" ]; then 
  echo "HDF5_ROOT variable is unset"; 
  exit -1
fi

mkdir -p $installPath
mkdir -p $installPath/tempDir

echo "Make virtual env in $installPath"
virtualenv $installPath

echo "Install modules:"
cd $installPath
pip=$installPath/bin/pip
python=$installPath/bin/python
source $installPath/bin/activate

$pip install cython
$pip install numpy 
$pip install matplotlib 
$pip install scipy
$pip install scikit-image 
$pip install ConfigArgParse 
$pip install git+https://github.com/jsonpickle/jsonpickle.git 
$pip install jsonschema 
$pip install glob2 
$pip install transforms3d
$pip install git+https://github.com/bcj/AttrDict.git
$pip install git+https://github.com/lxml/lxml.git@lxml-3.4
$pip install pyside
$pip install graphviz
$pip install pyyaml
$pip install psutil

cd $installPath/tempDir

name="sip-4.17"
wget http://sourceforge.net/projects/pyqt/files/sip/sip-4.17/$name.tar.gz
tar xvf $name.tar.gz
rm $name.tar.gz
cd $name
$python configure.py
make -j12 install

name="PyQt-4.11.4/PyQt-x11-gpl-4.11.4"
wget http://sourceforge.net/projects/pyqt/files/PyQt4/PyQt-4.11.4/$name.tar.gz
tar xvf $name.tar.gz
rm $name.tar.gz
cd $name
QT_SELECT=qt4
$python configure.py
make -j12 install

name="PyQt-gpl-5.5.1"
wget http://sourceforge.net/projects/pyqt/files/PyQt5/PyQt-5.5.1/$name.tar.gz
tar xvf $name.tar.gz
rm  $name.tar.gz
cd $name
QT_SELECT=qt5
$python configure.py
make -j12 install


$pip install PyOpenGL
$pip install git+https://github.com/vispy/vispy.git

$pip install jupyter
$pip install mpldatacursor


# Install h5py
wget "https://pypi.python.org/packages/source/h/h5py/h5py-2.5.0.tar.gz#md5=6e4301b5ad5da0d51b0a1e5ac19e3b74"
tar -xvzf h5py-2.5.0.tar.gz
cd h5py-2.5.0
$python setup.py configure --hdf5="$HDF5_ROOT"
$python setup.py install

cd $installPath
rm -r $installPath/tempDir
echo "Finished virtualenv install"