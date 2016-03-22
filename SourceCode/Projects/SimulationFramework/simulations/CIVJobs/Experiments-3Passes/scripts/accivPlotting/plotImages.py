#!/usr/bin/python

import sys
if sys.version_info[0] != 3:
    print("This script is only python3 compatible!")
    exit(1)


import numpy
import h5py
from optparse import OptionParser
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import os.path

parser = OptionParser()
parser.add_option("--folder", type="string", default='.', dest="folder", help="folder where the images are")
parser.add_option("--imageCount", type="int", default=100, dest="imageCount", help="the number of images")


options, args = parser.parse_args()

folder = options.folder

maxImageStd = 3.0 # Image data is clamped to be within 3 std. dev. from the mean.
                  # Decrease this number to increase image contrast.


for imageIndex in range(options.imageCount):
  imageFileName = '%s/image%03i.h5'%(folder,imageIndex)
  if not(os.path.exists(imageFileName)):
    continue
  print imageFileName
  h5File = h5py.File(imageFileName, 'r')
  bounds = h5File["bounds"][...]
  imageData = h5File["data"][...]
  imageMask = numpy.array(h5File["mask"][...],bool)
  h5File.close()

  gx = numpy.linspace(bounds[0],bounds[1],imageData.shape[1])
  gy = numpy.linspace(bounds[2],bounds[3],imageData.shape[0])
    
  imageMean = numpy.mean(imageData[imageMask])
  imageStd = numpy.std(imageData[imageMask])
  
  imageData *= imageMask
  vmin = imageMean-maxImageStd*imageStd
  vmax = imageMean+maxImageStd*imageStd
  
  plt.imsave('%s/image%03i.png'%(folder,imageIndex), imageData[::-1,:],
             vmin=vmin, vmax=vmax, cmap=cm.get_cmap('gray'))


