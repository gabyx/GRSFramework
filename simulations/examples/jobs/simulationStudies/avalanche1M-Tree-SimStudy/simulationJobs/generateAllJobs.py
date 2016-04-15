#!/usr/bin/env python

import os
from os import path as path
import glob2
import subprocess
import sys

jobGeneratorDir =  os.environ["HPCJOBCONFIGURATOR_DIR"]
sys.path.append( jobGeneratorDir )

from HPCJobConfigurator.configureJob import configJob


reply = input("WARNING: Generating all parameter studies! Continue [yes]? ")
if reply == "yes":
    a = dict(interact="False")
    configJob(configFile ="Generator.ini",overwriteArgs=a, colorOutput=True)

    currrentFolder = os.getcwd()
    studyFolders = glob2.glob(path.join(currrentFolder,"studies","*"))

    for sF in studyFolders:
        os.chdir(path.realpath(sF))
        configJob( overwriteArgs = a, colorOutput=True)
