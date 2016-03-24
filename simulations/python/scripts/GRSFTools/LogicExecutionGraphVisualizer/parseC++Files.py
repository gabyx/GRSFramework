#!/usr/bin/env python3 

import sys,os, subprocess, ctypes
import graphviz as gv

if sys.version_info[0] != 3:
    print("This script is only python3 compatible!")
    exit(1)

from argparse import ArgumentParser

from attrdict import AttrMap
import xml.etree.ElementTree as ET

import commentjson
import jsonpickle
jsonpickle.load_backend('commentjson', 'dumps', 'loads', ValueError)

def jsonParse(s):
  return jsonpickle.decode(s) 
        
def jsonLoad(filePath):
  f = open(filePath)
  return jsonParse(f.read())
  

def main():
    parser = ArgumentParser()
    
    parser.add_argument("-f", "--files", dest="files", 
        help="The converter logic xml to visualize", metavar="<path>", action="append", required=True)
    
    
    opts= AttrMap(vars(parser.parse_args()))
    
    
    # parse all c++ files and pars line for line