import os
from os import path as path
import glob2
import subprocess
import sys

jobGeneratorDir =  os.environ["JOBGENERATOR_DIR"]
sys.path.append( jobGeneratorDir )

import JobGenerator

from JobGenerator import submit

# launch all studies

launchSequence = [ 0,3,5,7,9,10,14,1,2,4,6,8,11,12,13 ]

if len(set(launchSequence)) != len(launchSequence):
    raise ValueError("Launch Sequence indices appearing twice!")

currrentFolder = os.getcwd()



reply = input("WARNING: Launching  all parameter studies! Continue [yes]? ")
if reply == "yes":
    
    nr = input("Which job nr? , [0,1,2,3,...] :")
    
    for studyNr in launchSequence:
        studyFolder = path.join(currrentFolder,"studies","Avalanche1M-Tree-Study-P-%i" % studyNr)
        
        os.chdir(path.realpath(studyFolder))
        
        f = path.join(studyFolder, "cluster", "*.%s" % nr,"submit.sh")
        submitFiles = glob2.glob(f)
        
        for s in submitFiles:
            print("Launching %s" % s)
            out = subprocess.call(s, shell=True)
