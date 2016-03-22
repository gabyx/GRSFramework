#!/usr/bin/python
import numpy
import h5py
from optparse import OptionParser
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import os.path
parser = OptionParser()
parser.add_option("--inFolder", type="string", default='.', dest="inFolder", help="folder where the images are")
parser.add_option("--outFolder", type="string", default='.', dest="outFolder", help="folder to write to")
parser.add_option("--imageCount", type="int", default=34, dest="imageCount", help="the number of images")


options, args = parser.parse_args()

maxImageStd = 3.0 # Image data is clamped to be within 3 std. dev. from the mean.
                  # Decrease this number to increase image contrast.

for prefix in ['earlier','later']:
  for imageIndex in range(options.imageCount):
    imageFileName = '%s/%sAdvectedImage_%i.h5'%(options.inFolder,prefix, imageIndex)
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
    
    plt.imsave('%s/%sAdvectedImage_%i.png'%(options.outFolder,prefix,imageIndex), imageData[::-1,:],
               vmin=vmin, vmax=vmax, cmap=cm.get_cmap('gray'))



