import os

import xml.etree.ElementTree as ET

class ConverterProcessFileWriter:
    
    def __init__(self,pipelineSpecs,jobGenModules):
      
      self.cf = jobGenModules["commonFunctions"]
      self.iH = jobGenModules["importHelpers"]
      
      self.pipelineSpecs = pipelineSpecs
    
    def write(self,processFrames, processFile):
         
        for procIdx, fList in enumerate(processFrames):
          # for each process
  
            root = ET.Element("Converter")
            
            for frame in fList:
                    
                    converter = frame["tools"]["converter"]
                    
                    if converter["status"] =="finished":
                        continue
                    
                    
                    for simFileIdx,f in enumerate(converter["simFiles"]):
                      
                        fNode = ET.Element("File");
                        root.append(fNode)
                        fNode.attrib['simFile'] = f
                        fNode.attrib['useRange'] = "true"
                        
                        # for first file, set startIdx if we have one!
                        if simFileIdx == 0:
                          if "startIdx" in converter:
                            fNode.attrib["startIdx"] = str(converter["startIdx"])
                            fNode.attrib["mappedStartIdx"] = str(converter["mappedStartIdx"]) # no need to set, but make explicit!
                            
                        fNode.attrib['outputFile'] = converter["outputFile"]
                        fNode.attrib['uuid'] = str(self.cf.makeUUID(f));
                        
                    
                        
                   
            tree = ET.ElementTree(root);

            f = open(processFile.format(procIdx),"w+")
            f.write(self.cf.prettifyXML(root))
            f.close()
        print("Wrote converter process files for all ranks")