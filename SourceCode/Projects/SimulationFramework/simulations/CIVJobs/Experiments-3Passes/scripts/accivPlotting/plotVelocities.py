#!/usr/bin/python

import sys,traceback
if sys.version_info[0] != 3:
    print("This script is only python3 compatible!")
    exit(1)


import numpy as np
import h5py
from attrdict import AttrMap,AttrDict
from argparse import ArgumentParser
import os.path
import json

# Import Plot Settings
sys.path.append( os.environ["PYTHONSCRIPT_DIR"] )
import PlotSettings as pS
import PlotSettings.generalSettings as gPS
lineSettings = gPS.defaultLineSettings 
markerSize   = gPS.defaultMarkerSize 


defaultPlotSettings ={
      "savePlots": True, 
      "figurePrefix":"fig",
      "figureExt": ".pdf",
      "showTitles": True,
      "x0" : -0.05,
      "y0" : 0.1
}

defaultOptions = {      "imageFileName": "image001.h5" , 
                        "gridFileName": "outGridVelocity.h5", 
                        "scatterFileName": "outScatteredVelocity.h5", 
                        "tiePointsFolder" : "accivWorkFolder",
                        "plotSettings" : defaultPlotSettings
                 }


def plotCmd():
    
    parser = ArgumentParser()
    
    parser.add_argument("--folder", type=str, dest="folder", help="folder of the output data to be plotted")
    parser.add_argument("--imageFileName", type=str, dest="imageFileName", help="image file path")
    parser.add_argument("--gridFileName", type=str, dest="gridFileName")
    parser.add_argument("--scatterFileName", type=str,  dest="scatterFileName")
    parser.add_argument("--tiePointsFolder", type=str, dest="tiePointsFolder")
    parser.add_argument("--plotSettings", type=str, dest="plotSettings")
    
    options = AttrMap(vars(parser.parse_args()))
    
    # load plotSettings if defined
    if "plotSettings" in options:
       f = open(options["plotSettings"],"r")
       options["plotSettings"] = AttrMap(json.load(f))

    plot(options)

    
def plot(options):
    
    if isinstance(options,dict):
        options = AttrMap(options)
    
    # overwrite options with default values if not existing!
    def makeComplete(d,defaults):
      if isinstance(d,dict) or isinstance(d,AttrMap) or isinstance(d,AttrDict):
        for k in defaults.keys():
          if k not in d:
            d[k] = defaults[k]
          else:
            makeComplete(d[k],defaults[k]) # goes on if d[k] is a dict
        
    
    makeComplete(options,defaultOptions)
            
    print("arguments:" , options)
    plotSettings = options.plotSettings
    
    import matplotlib as mpl
    # update matplotlib settings =======================================
    pS.loadPlotSettings(mpl)
    # ==================================================================
    
    import matplotlib.cm as cm
    import matplotlib.ticker as ticker
    import matplotlib.pyplot as plt
    import matplotlib.colors as colors
    from   mpl_toolkits.axes_grid1 import make_axes_locatable
    from matplotlib.offsetbox import AnchoredOffsetbox, TextArea
    

    """ Add an anchored text to the axis """
    def addText(ax, t, loc=2, textp = dict(size=12), pad = 0.1, borderpad = 0.5 ,
                frameon =True, 
                fc="white", ec="black" ,lw=1):
        #box = TextArea(t, textprops=textp)
        
        #a = AnchoredOffsetbox(loc=loc,
                             #child=box, pad=pad,
                             #frameon=frameon,
                             #borderpad=borderpad,
                             #)
        #a.patch.set_boxstyle("square",pad=0.1)
        #a.patch.set_facecolor(fc)
        #a.patch.set_edgecolor(ec)
        #a.patch.set_linewidth(lw)

        #ax.add_artist(a)
        
        a = ax.annotate(t, (1,0), (0, -20), xycoords='axes fraction', textcoords='offset points', va='top', ha='right', **textp)
        
        return a
    
    
    plt.close("all")
    
    noScatterFile = False
    scatterFileName = os.path.join(options.folder,options.scatterFileName)
    if(not os.path.exists(scatterFileName)):
      noScatterFile = True

    gridFileName = os.path.join(options.folder,options.gridFileName)
    if(not os.path.exists(gridFileName)):
      raise ValueError("not found:", gridFileName)

      
    
    tiePointsFileName = os.path.join(options.folder,options.tiePointsFolder,"combinedCorrelationTiePoints.h5")
    if(not os.path.exists(tiePointsFileName)):
      print("not found:", tiePointsFileName)
      exit()
      
    imageFileName = options.imageFileName
    if(not os.path.exists(imageFileName)):
      raise ValueError("not found:", imageFileName)
  
    # Plot Options =====================================================
    # figure titles
    figureSize= (pS.cm2inch(16),pS.cm2inch(12))
    figures={}
    figureTitles=dict()
    
    # background
    axisBG = [1,1,1]

    # velocity norm range ( [xmin,ymin],[xmax,ymax])
    velocityRange = np.array([[0,-0.4],[-2.0,0.4]])
    velocityMaxNorm =  np.around(np.linalg.norm(velocityRange[1] - velocityRange[0]),decimals=1)
    pixelShiftRange = [-10,10]
    # plot a velocity vector every "skip" pixels from the gridded velocity data
    skip = 6
    
    # location uncertainty
    corrLocationUncertaintyMax = 0.002
    corrVelocityUncertaintyMax = velocityMaxNorm
    tiePointFractionMax = 0.1
    
    # number of points to be sampled from the scattered data
    maxPoints = 10000

    # the locations of the major and minor axes to plot
    
    x0 = plotSettings.x0
    y0 = plotSettings.y0

        
    # the width around each axis to take points from when plotting axes
    dx = 0.01
    dy = 0.01
    
    # location of time and frame idx
    locFrame = 4
    propFrameText = dict(color="black", size=9)

    maxImageStd = 3.0 # Image data is clamped to be within 3 std. dev. from the mean.
                      # Decrease this number to increase image contrast.

    # decrease these numbers to increase the length of vectors, and visa versa
    scatterVectorScale = 200.0
    gridVectorScale = 200.0

    quiverOpts = {'headwidth': 2, 'headlength':4}
    
    colormap = cm.Greys_r
    # ===================================================================



    h5File = h5py.File(imageFileName, 'r')
    time = h5File["time"][...]
    
    
    if not "frameIdx" in options:
        frameIdx = 0
    else:
        frameIdx = options.frameIdx
        
    print("Time: ", time)
    bounds = h5File["bounds"][...]
    imageData = h5File["data"][...]
    print("Bounds:" , bounds)
    #print("Index (0,0) (top-left):", imageData[0,0])
    #print("Index (1,0):", imageData[1,0])
        
    imageMask = np.array(h5File["mask"][...],bool)
    imageFinalMask = np.ma.masked_equal(h5File["finalMask"][...],0)
    h5File.close()

    if not noScatterFile:
        h5File = h5py.File(scatterFileName, 'r')
        x = h5File["x"][...]
        y = h5File["y"][...]
        vx = h5File["vx"][...]
        vy = h5File["vy"][...]
        pixelVx = h5File["dataX"][...]
        pixelVy = h5File["dataY"][...]
        h5File.close()
    
    
    h5File = h5py.File(tiePointsFileName, 'r')
    deltaTs = h5File["deltaTs"][...]
    maxDeltaT = np.amax(deltaTs)
    residualsFound = "correlationVelocityResiduals" in h5File
    if residualsFound:
      correlationVelocityResiduals = h5File["correlationVelocityResiduals"][...]
      correlationLocationResiduals = h5File["correlationLocationResiduals"][...]
      correlationTiePointsX1 = h5File["x1"][...]
      correlationTiePointsY1 = h5File["y1"][...]
      correlationTiePointsX2 = h5File["x2"][...]
      correlationTiePointsY2 = h5File["y2"][...]
    h5File.close()


    h5File = h5py.File(gridFileName, 'r')
    gridVx = h5File["vx"][...]
    gridVy = h5File["vy"][...]
    h5File.close()
    print("Gridded velocity shape: ", gridVx.shape)
    gx = np.linspace(bounds[0],bounds[1],gridVx.shape[1])
    gy = np.linspace(bounds[2],bounds[3],gridVx.shape[0])



    # File info: ===========================================================
    gridSize = gridVx.shape
    print( "Grid Data size: " + str(gridSize) )
    if not noScatterFile:
        print( "Scattered data size: " + str(vx.shape) )
        print( "Scattered pixelVx size: " + str(pixelVx.shape) )
    # ======================================================================

    # Setup Data============================================================
    [gridX, gridY] = np.meshgrid(gx,gy)
    vMagGrid = np.sqrt(gridVx**2 + gridVy**2)
    
    
    if not noScatterFile:
        dLon = gx[1]-gx[0]
        dLat = gy[1]-gy[0]
        pixelVx = pixelVx/dLon*maxDeltaT
        pixelVy = pixelVy/dLat*maxDeltaT
        vMag = np.sqrt(vx**2+vy**2)

    #imageMean = np.mean(imageData[imageMask])
    #imageStd = np.std(imageData[imageMask])

    #imageData *= imageMask
    #imageData = np.maximum(imageMean-maxImageStd*imageStd,
       #np.minimum(imageMean+maxImageStd*imageStd,imageData))


    if not noScatterFile:
        if(x.size > maxPoints):
          indices = np.array(np.random.rand(maxPoints)*x.size,int)
        else:
          indices = np.array(np.linspace(0,x.size-1,x.size),int)
        maskXAxis = np.abs(y-y0) < dy
        maskYAxis = np.abs(x-x0) < dx
        xAxisIndices = indices[maskXAxis[indices]]
        yAxisIndices = indices[maskYAxis[indices]]

    xAxisGridIndex = np.argmin(np.abs(gy-y0))
    yAxisGridIndex = np.argmin(np.abs(gx-x0))
    
    # Mask the correlation tie points, to plot the uncertainty 
    # histogram only for the tie points (x1,y1 in image 1 to x2,y2 in image 2)
    # in the mask ============================================================
    if residualsFound: 
      # acciv grid is (y,x) indexed with left-bottom corner as origin
      dim = np.array([gridSize[1],gridSize[0]],dtype=int) 
      minPoint = np.array([bounds[0],bounds[2]],dtype=float)
      maxPoint = np.array([bounds[0+1],bounds[2+1]],dtype=float)
      dxInv = 1.0/((maxPoint - minPoint) / dim)
      points1 = np.vstack([correlationTiePointsX1,correlationTiePointsY1]).T # [[x,y],[x,y]]
      points2 = np.vstack([correlationTiePointsX2,correlationTiePointsY2]).T
      
      def getValidIndices(dim,minP,dxInv,points):
        indices = ((points - minP) * dxInv).astype(int)
        validIndices = np.logical_not(
                     ((indices[:,0] < 0) | (indices[:,0] >= dim[0])) |
                     ((indices[:,1] < 0) | (indices[:,1] >= dim[1]))
                    )
        
        return indices, validIndices
        
      indices1, validMask1 = getValidIndices(dim,minPoint,dxInv,points1)
      indices2, validMask2 = getValidIndices(dim,minPoint,dxInv,points2) 
      totalMask = validMask1 & validMask2 # both tie points need to be in bounds
      indices1 = indices1[totalMask]
      indices2 = indices2[totalMask]
      mask = imageFinalMask.mask # take care (y,x) indexes
      
      correlationTiePointMask = np.logical_not(
                                   mask[indices1[:,1],indices1[:,0]] |
                                   mask[indices2[:,1],indices2[:,0]])  
      #either first point masked or the second points results in neglecting the tiepoint
      
      print("Correlation tie points in mask: %f" % (np.sum(correlationTiePointMask) / correlationVelocityResiduals.size *100 ) + "%")
      
      # mask correlation vel/loc residuals for further processing
      correlationVelocityResiduals = correlationVelocityResiduals[correlationTiePointMask]
      correlationLocationResiduals = correlationLocationResiduals[correlationTiePointMask]

    # ========================================================================
    
    

    # Plot Data ============================================================
    figCount = 1
    
    if not noScatterFile:
        figureTitles[figCount] = 'a sample of %i scattered velocity vectors'%(indices.size)
        fig = plt.figure(figCount,figsize=figureSize)
        figures[figCount] = fig
        ax = fig.add_subplot(111, aspect='equal')
        ax.imshow(imageData, origin='lower', extent=(bounds[0],bounds[1],bounds[2],bounds[3]), cmap=colormap)
        ax.quiver(x[indices], y[indices], vx[indices], vy[indices], color='g', pivot='tail',  scale_units='xy', scale=scatterVectorScale, **quiverOpts)
        ax.quiver(x[xAxisIndices], y[xAxisIndices], vx[xAxisIndices], vy[xAxisIndices], color='r', pivot='mid',  scale_units='xy', scale=scatterVectorScale, **quiverOpts)
        ax.quiver(x[yAxisIndices], y[yAxisIndices], vx[yAxisIndices], vy[yAxisIndices], color='b', pivot='mid',  scale_units='xy', scale=scatterVectorScale, **quiverOpts)
        
        if plotSettings.showTitles:
            ax.set_title(figureTitles[figCount])
            
        ax.set_xlabel("$x$ [m]")
        ax.set_ylabel("$y$ [m]")
        addText(ax,"t: %.04f s, frame: %i" % (time,frameIdx), loc=locFrame, textp = propFrameText)
        fig.tight_layout(pad=0.1)
        pS.defaultFormatAxes(ax)
        
    ##fig = plt.figure(12, figsize=[width,height])
    ###fig.subplots_adjust(left=0.075, right=0.975, bottom=0.05, top=0.95, wspace=0.2, hspace=0.25)
    ##ax = fig.add_subplot(111, aspect='equal')
    ##plt.imshow(imageData, extent=(bounds[0],bounds[1],bounds[3],bounds[2]), cmap=colormap)
    ##ax.set_ylim(ax.get_ylim()[::-1])
    ##plt.axis('tight')

    figCount+=1
    figureTitles[figCount] = 'gridded velocity vector (skip = %i)'% (skip)
    fig = plt.figure(figCount,figsize=figureSize)
    figures[figCount] = fig
    ax = fig.add_subplot(111, aspect='equal')
    ax.imshow(imageData, origin='lower', extent=(bounds[0],bounds[1],bounds[2],bounds[3]), cmap=colormap)
    ax.quiver(gridX[::skip,::skip], gridY[::skip,::skip], gridVx[::skip,::skip], gridVy[::skip,::skip], color='g', 
              pivot='tail',  scale_units='xy', scale=gridVectorScale)
    if plotSettings.showTitles:
        ax.set_title(figureTitles[figCount])

    ax.set_xlabel("$x$ [m]")
    ax.set_ylabel("$y$ [m]")
    addText(ax,"t: %.04f s, frame: %i" % (time,frameIdx), loc=locFrame, textp = propFrameText)
    fig.tight_layout(pad=0.1)
    pS.defaultFormatAxes(ax)
    
    figCount+=1
    #figureTitles[figCount] = '$v_x$'
    #fig = plt.figure(figCount,figsize=figureSize)
    #figures[figCount] = fig
    #ax = fig.add_subplot(111, aspect='equal')
    #im = ax.imshow(gridVx, origin='lower', extent=(bounds[0],bounds[1],bounds[2],bounds[3]), cmap=plt.get_cmap('jet'))
    #divider = make_axes_locatable(ax)
    #cax1 = divider.append_axes("right", size="2%", pad=0.2)
    #fig.colorbar(im, cax = cax1)
    #im.set_clim(0,velocityMaxNorm)
    #ax.set_aspect('equal')
    #im.set_clim(velocityRange[0][0],velocityRange[1][0])
    #if plotSettings.showTitles:
        #ax.set_title(figureTitles[figCount])
    #ax.set_xlabel("$x$ $[m]$")
    #ax.set_ylabel("$y$ $[m]$")
    #addText(ax,"t: %.04f s, frame: %i" % (time,frameIdx), loc=locFrame, textp = propFrameText)
    #fig.tight_layout(pad=0.1)
    #pS.defaultFormatAxes(ax)
    
    #cax1.set_ylabel(r'$[m/s]$')

    figCount+=1
    #figureTitles[figCount] = '$v_y$'
    #fig = plt.figure(figCount,figsize=figureSize)
    #figures[figCount] = fig
    #ax = fig.add_subplot(111, aspect='equal')
    #im = ax.imshow(gridVy, origin='lower', extent=(bounds[0],bounds[1],bounds[2],bounds[3]), cmap=plt.get_cmap('jet'))
    #divider = make_axes_locatable(ax)
    #cax1 = divider.append_axes("right", size="2%", pad=0.2)
    #fig.colorbar(im, cax = cax1)
    #im.set_clim(0,velocityMaxNorm)
    #ax.set_aspect('equal')
    #im.set_clim(velocityRange[0][1],velocityRange[1][1])
    #if plotSettings.showTitles:
        #ax.set_title(figureTitles[figCount])
        
    #ax.set_xlabel("$x$ $[m]$")
    #ax.set_ylabel("$y$ $[m]$")
    #addText(ax,"t: %.04f s, frame: %i" % (time,frameIdx), loc=locFrame, textp = propFrameText)
    #fig.tight_layout(pad=0.1)
    #pS.defaultFormatAxes(ax)
    #cax1.set_ylabel(r'$[m/s]$')


    figCount+=1
    figureTitles[figCount] = r'$||\mathbf{v}||$'
    fig = plt.figure(figCount,figsize=figureSize)
    figures[figCount] = fig
    ax = fig.add_subplot(111, aspect='equal')
    im = ax.imshow(vMagGrid, origin='lower', extent=(bounds[0],bounds[1],bounds[2],bounds[3]), cmap=plt.get_cmap('jet') , interpolation='none')
    divider = make_axes_locatable(ax)
    cax1 = divider.append_axes("right", size="3%", pad=0.2)
    c=fig.colorbar(im, cax = cax1)
    im.set_clim(0,velocityMaxNorm)
    ax.set_aspect('equal')
    if plotSettings.showTitles:
        ax.set_title(figureTitles[figCount])
        
    ax.set_xlabel("$x$ [m]")
    ax.set_ylabel("$y$ [m]")
    cax1.set_ylabel(r'[m/s]')
    
    addText(ax,"t: %.04f s, frame: %i" % (time,frameIdx), loc=locFrame, textp = propFrameText)
    fig.tight_layout(pad=0.1)
    pS.defaultFormatAxes(ax)
    pS.defaultFormatColorbar(c)
    

    if not noScatterFile:
        weights = np.ones(vx.shape)/vx.size
        figCount+=1
        figureTitles[figCount] = 'velocity histograms'
        fig = plt.figure(figCount,figsize=figureSize)
        figures[figCount] = fig
        #fig.subplots_adjust(left=0.075, right=0.975, bottom=0.05, top=0.95, wspace=0.2, hspace=0.25)
        ax = fig.add_subplot(111)
        ax.hist(vMag,100,weights=weights,histtype='step',color='k')
        ax.hist(vx,100,weights=weights,histtype='step',color=[0.4]*3)
        ax.hist(vy,100,weights=weights,histtype='step',color=[0.6]*3)
        ax.set_xlabel('velocity')
        ax.set_xlim(pixelShiftRange)
        ax.set_ylabel('tie point fraction')
        if plotSettings.showTitles:
            ax.set_title(figureTitles[figCount])
            
        ax.legend([r'$||\mathbf{v}||$','$v_x$','$v_y$'])
        fig.tight_layout(pad=0.1)
        pS.defaultFormatAxes(ax)

        figCount+=1
        figureTitles[figCount] = 'pixel offset histograms (search range)'
        fig = plt.figure(figCount,figsize=figureSize)
        figures[figCount] = fig
        ax = fig.add_subplot(111)
        ax.hist(np.sqrt(pixelVx**2+pixelVy**2),100,weights=weights,histtype='step',color='k')
        ax.hist(pixelVx,100,weights=weights,histtype='step',color=[0.4]*3)
        ax.hist(pixelVy,100,weights=weights,histtype='step',color=[0.6]*3)
        ax.set_xlabel(r'pixel vel. $\cdot$maxDeltaT [px])')
        ax.set_xlim(pixelShiftRange)
        ax.set_ylabel('tie point fraction')
        if plotSettings.showTitles:
            ax.set_title(figureTitles[figCount])
        ax.legend([r'$||\mathbf{v}||$','$v_x$','$v_y$'])
        addText(ax,"t: %.04f s, frame: %i" % (time,frameIdx), loc=locFrame, textp = propFrameText)
        fig.tight_layout(pad=0.1)
        pS.defaultFormatAxes(ax)

    #figCount+=1
    #fig = plt.figure(figCount,figsize=figureSize)
    #ax = fig.add_subplot(111, aspect='equal')
    #plt.plot(x[maskXAxis], vy[maskXAxis], '.k',gx,gridVy[xAxisGridIndex,:],'r')
    #plt.set_title('$v_y$ along $x$ axis within $dy = %.1f$ of $y = %.1f$'%(dy,y0))
    #plt.set_xlabel('$x$ $[m]$')
    #plt.set_ylabel('$v_y$ $[m/s]$')
    #plt.axis('tight')

    figCount+=1
    figureTitles[figCount] = r'$||\mathbf{v}||$ along $y$ axis within $dx = %.2f$ of $x = %.2f$ '%(dx,x0)
    fig = plt.figure(figCount,figsize=figureSize)
    figures[figCount] = fig
    ax = fig.add_subplot(111)
    if not noScatterFile:
        ax.plot(vMag[maskYAxis], y[maskYAxis], '.k',ms=markerSize)
    ax.plot(vMagGrid[:,yAxisGridIndex] * imageFinalMask[:,yAxisGridIndex],gy, color=[0.5]*3, lw=lineSettings["thick"])
    if plotSettings.showTitles:
        ax.set_title(figureTitles[figCount])
    
    ax.set_xlim([0,velocityMaxNorm])
    ax.set_ylim(bounds[2:4])
    ax.set_xlabel('$||\mathbf{v}||$ [m/s]')
    ax.set_ylabel('$y$ [m]')
    addText(ax,"t: %.04f s, frame: %i" % (time,frameIdx), loc=locFrame, textp = propFrameText)
    fig.tight_layout(pad=0.1)
    pS.defaultFormatAxes(ax)



    figCount+=1
    figureTitles[figCount] = r'$||\mathbf{v}||$ along $x$ axis within $dy = %.2f$ of $y = %.2f$' % (dy,y0)
    fig = plt.figure(figCount,figsize=figureSize)
    figures[figCount] = fig
    ax = fig.add_subplot(111)
    if not noScatterFile:
        ax.plot(x[maskXAxis], vMag[maskXAxis], '.k', ms=markerSize)
    ax.plot(gx,vMagGrid[xAxisGridIndex,:] * imageFinalMask[xAxisGridIndex,:] ,color=[0.5]*3 ,lw=lineSettings["thick"])
    if plotSettings.showTitles:
        ax.set_title(figureTitles[figCount])

    ax.set_xlim(bounds[0:2])
    ax.set_ylim([0,velocityMaxNorm])
    ax.set_ylabel('$||\mathbf{v}||$ [m/s]')
    ax.set_xlabel('$x$ $[m]$')
    addText(ax,"t: %.04f s, frame: %i" % (time,frameIdx), loc=locFrame, textp = propFrameText)
    fig.tight_layout(pad=0.1)
    pS.defaultFormatAxes(ax)


    #plot masked velocity with velocityMask
    figCount+=1
    figureTitles[figCount] = r'Velocity $||\mathbf{v}||$ , masked grid'
    fig = plt.figure(figCount,figsize=figureSize)
    figures[figCount] = fig
    ax = fig.add_subplot(111, aspect='equal', axisbg=axisBG)
    absVelMasked = vMagGrid * imageFinalMask;
    if plotSettings.showTitles:
        ax.set_title(figureTitles[figCount])
    ax.set_xlabel('$x$ [m]')
    ax.set_ylabel('$y$ [m]')
    ax.set_axisbelow(True)
    #image
    im = ax.imshow(absVelMasked, origin='lower', 
                    extent=(bounds[0],bounds[1],bounds[2],bounds[3]), 
                    cmap=plt.get_cmap('jet') , interpolation='none', zorder=1.2, clim=[0,velocityMaxNorm])
    # colorbar
    divider = make_axes_locatable(ax)
    cax1 = divider.append_axes("right", size="3%", pad=0.2)
    cax1.tick_params(length=3)
    c=fig.colorbar(im, cax = cax1, format=ticker.FuncFormatter(lambda x,pos: "%0.1f"%x))
    for l  in cax1.yaxis.get_ticklabels()[1::2]:
          l.set_visible(False)
    cax1.set_title(r'[m/s]',fontsize=10)
    # grid
    ax.grid(linestyle="-",linewidth=lineSettings["extra-thin"], color=[0.5]*3)
    
    # remove every second label
    for label in ax.xaxis.get_ticklabels()[::2]:
        label.set_visible(False)
    # ticks
    ax.tick_params(top='off',right='off',bottom='off',left='off')
    addText(ax,"t: %.04f s, frame: %i" % (time,frameIdx), loc=locFrame, textp = propFrameText)
    
    pS.defaultFormatAxes(ax)
    pS.defaultFormatColorbar(c)
    fig.tight_layout(pad=0.1)



    if residualsFound:
      mean = np.mean(correlationVelocityResiduals)
      med=np.median(correlationVelocityResiduals)
      maxVal = 6.0*med
      figCount+=1
      figureTitles[figCount] = r'correlation velocity uncertainty'
      fig = plt.figure(figCount,figsize=figureSize)
      figures[figCount] = fig
      ax = fig.add_subplot(111)
      weights = np.ones(correlationVelocityResiduals.shape)/correlationVelocityResiduals.size
      ax.hist(correlationVelocityResiduals,100,range=[0.0,maxVal], weights=weights,histtype='step',color='k')
      ax.axvline(mean,color="gray",ls="dashed")
      ax.set_xlabel('correlation velocity uncertainty [m/s]')
      ax.set_ylabel('tie point fraction')
      ax.set_xlim([0,corrVelocityUncertaintyMax])
      ax.set_ylim([0,tiePointFractionMax])
      if plotSettings.showTitles:
        ax.set_title(figureTitles[figCount])
      addText(ax,"t: %.04f s, frame: %i" % (time,frameIdx), loc=locFrame, textp = propFrameText)
      fig.tight_layout(pad=0.1)
      pS.defaultFormatAxes(ax)
      
      mean = np.mean(correlationLocationResiduals)
      med =np.median(correlationLocationResiduals)
      maxVal = 6.0*med
      figCount+=1
      figureTitles[figCount] = r'correlation location uncertainty'
      fig = plt.figure(figCount,figsize=figureSize)
      figures[figCount] = fig
      ax = fig.add_subplot(111)
      weights = np.ones(correlationLocationResiduals.shape)/correlationLocationResiduals.size
      ax.hist(correlationLocationResiduals,100,range=[0.0,maxVal], weights=weights,histtype='step', color='k')
      ax.axvline(mean,color="gray",ls="dashed")
      ax.set_xlabel('correlation location uncertainty [m]')
      ax.set_ylabel('tie point fraction')
      ax.set_xlim([0,corrLocationUncertaintyMax])
      ax.set_ylim([0,tiePointFractionMax])
      if plotSettings.showTitles:
        ax.set_title(figureTitles[figCount])
      addText(ax,"t: %.04f s, frame: %i" % (time,frameIdx), loc=locFrame, textp = propFrameText)
      fig.tight_layout(pad=0.1)
      pS.defaultFormatAxes(ax)
    

    plt.draw()
    
    if plotSettings.savePlots:
        for figIdx, fig in figures.items():
            outFileName = os.path.join( options.folder, "%s%03i%s" % (plotSettings.figurePrefix, figIdx,plotSettings.figureExt) )
            fig.savefig(outFileName)
            plt.close(fig)
        with open(os.path.join( options.folder, "figureTitles.json") ,"w" ) as f:
            json.dump(figureTitles,f)
    
    else:
      plt.show()

    
    plt.close("all")
    
    

if __name__ == "__main__":
   try:
        sys.exit(plotCmd())
   except Exception as e:
        print("====================================================================")
        print("Exception occured: " + str(e))
        print("====================================================================")
        traceback.print_exc(file=sys.stdout)
        sys.exit(111)