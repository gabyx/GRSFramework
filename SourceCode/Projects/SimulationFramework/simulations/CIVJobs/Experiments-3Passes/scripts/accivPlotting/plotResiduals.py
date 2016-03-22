#!/usr/bin/python

import sys,traceback
if sys.version_info[0] != 3:
    print("This script is only python3 compatible!")
    exit(1)


import numpy
import h5py
from optparse import OptionParser
import matplotlib
from attrdict import AttrMap
from argparse import ArgumentParser

defaultArguments = { 
                    "savePlots": True, 
                    "scatterFileName": "outScatteredVelocity.h5", 
                    "figurePrefix":"fig" , 
                    "figureExt": ".jpg"
                    }

def plotCmd():
    
    parser = ArgumentParser()
    
    parser.add_argument("--folder", type=str, dest="folder", help="folder of the output data to be plotted")
    parser.add_argument("--savePlots", action="store_true", dest="savePlots", help="include this flag save plots to files instead of displaying them")
    parser.add_argument("--scatterFileName", type=str, dest="scatterFileName")
    parser.add_argument("--figureExt", type=str, default='.jpg', dest="figureExt")
    parser.add_argument("--figurePrefix", type=str, dest="figurePrefix")

    options = AttrMap(vars(parser.parse_args()))
    plot(options)

def plot(options):
    
    try:
        plotImpl(options)
    except Exception as e:
        print("====================================================================")
        print("Exception occured: " + str(e))
        print("====================================================================")
        traceback.print_exc(file=sys.stdout)
        return 1



def plotImpl(options):
    
    # overwrite options with default values if not existing!
    if defaultArguments is not None:
        for k in defaultArguments.keys():
            if k not in options or options[k] is None:
                options[k] = defaultArguments[k]
    
    if options.savePlots:
      matplotlib.use('Agg')

    import matplotlib.pyplot as plt

    folder = options.folder
    scatterFileName = '%s/%s'%(folder,options.scatterFileName)

    # width and height of each figure (inches)
    width = 12
    height = 10


    h5File = h5py.File(scatterFileName, 'r')
    x = h5File["x"][...]
    y = h5File["y"][...]
    resX = h5File["residualX"][...]
    resY = h5File["residualY"][...]
    h5File.close()


    fig = plt.figure(1, figsize=[width,height])
    fig.subplots_adjust(left=0.075, right=0.975, bottom=0.05, top=0.95, wspace=0.2, hspace=0.25)
    ax = fig.add_subplot(111, aspect='equal')
    plt.plot(x, resX, '.k')
    plt.xlabel('x')
    plt.ylabel('resX')
    plt.axis('tight')

    fig = plt.figure(2, figsize=[width,height])
    fig.subplots_adjust(left=0.075, right=0.975, bottom=0.05, top=0.95, wspace=0.2, hspace=0.25)
    ax = fig.add_subplot(111, aspect='equal')
    plt.plot(y, resX, '.k')
    plt.xlabel('y')
    plt.ylabel('resX')
    plt.axis('tight')

    fig = plt.figure(3, figsize=[width,height])
    fig.subplots_adjust(left=0.075, right=0.975, bottom=0.05, top=0.95, wspace=0.2, hspace=0.25)
    ax = fig.add_subplot(111, aspect='equal')
    plt.plot(x, resY, '.k')
    plt.xlabel('x')
    plt.ylabel('resY')
    plt.axis('tight')

    fig = plt.figure(4, figsize=[width,height])
    fig.subplots_adjust(left=0.075, right=0.975, bottom=0.05, top=0.95, wspace=0.2, hspace=0.25)
    ax = fig.add_subplot(111, aspect='equal')
    plt.plot(y, resY, '.k')
    plt.xlabel('y')
    plt.ylabel('resY')
    plt.axis('tight')


    plt.draw()
    if options.savePlots:
      outFileName = '%s/%s_x_resX.png'%(folder,options.figurePrefix)
      plt.figure(1)
      plt.savefig(outFileName)
      outFileName = '%s/%s_y_resX.png'%(folder,options.figurePrefix)
      plt.figure(2)
      plt.savefig(outFileName)
      outFileName = '%s/%s_x_resY.png'%(folder,options.figurePrefix)
      plt.figure(3)
      plt.savefig(outFileName)
      outFileName = '%s/%s_y_resY.png'%(folder,options.figurePrefix)
      plt.figure(4)
      plt.savefig(outFileName)
    else:
      plt.show()




if __name__ == "__main__":
    

   sys.exit(plotCmd());
