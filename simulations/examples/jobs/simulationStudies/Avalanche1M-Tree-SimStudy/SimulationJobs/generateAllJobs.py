
import os
from os import path as path
import glob2
import subprocess
import sys

jobGeneratorDir =  os.environ["JOBGENERATOR_DIR"]
sys.path.append( jobGeneratorDir )

from HPCJobConfigurator.configureJob import submit


reply = input("WARNING: Generating all parameter studies! Continue [yes]? ")
if reply == "yes":
    a = dict(interact="False")
    submit(configFile ="Generator.ini",overwriteArgs=a)
    
    currrentFolder = os.getcwd()
    studyFolders = glob2.glob(path.join(currrentFolder,"studies","*"))
        
    for sF in studyFolders:
        os.chdir(path.realpath(sF))
        submit( overwriteArgs = a)
