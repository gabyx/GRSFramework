import sys,os,inspect

from .generalSettings import *

# use the matpllotlibrc settings in this folder 
def loadPlotSettings(mplModule):
    
    def getScriptPath():
      return os.path.dirname(inspect.getfile(inspect.currentframe()))
  
    matplotlibrcFile = os.path.join(getScriptPath(),"matplotlibrc")
    
    # update matplotlib settings =======================================
    if matplotlibrcFile is not None:
        print("Setting matplotlib settings from file: %s" % matplotlibrcFile)
        mplModule.rcParams = mplModule.rc_params_from_file(matplotlibrcFile, fail_on_error=True)

    defaults = { 
      'lines.linewidth': defaultLineSettings["thin"] , 
      'axes.linewidth': defaultLineSettings["thick"],
      "lines.marker"      : None,         # the default marker
      "lines.markeredgewidth"  : cm2inch(defaultLineSettings["extra-thin"]),     # the line width around the marker symbol
      "lines.markersize"  : cm2pt(0.12),            # markersize, in points
      "figure.figsize": (cm2inch(16),cm2inch(12)) 
    }
    mplModule.rcParams.update(defaults)
    
    path = os.path.join(getScriptPath(),"MathMacros.sty")
    mplModule.rcParams["text.latex.preamble"] = [r"\def\dontLoadMathEnv{}",r"\input{%s}" % path]
    
    
    # ==================================================================
