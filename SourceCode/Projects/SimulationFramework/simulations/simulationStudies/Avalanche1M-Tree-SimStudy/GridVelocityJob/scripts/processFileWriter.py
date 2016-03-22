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
                      
                        sIdx = converter["simInfos"][simFileIdx].resampleInfo.startIdx
                        eIdx = converter["simInfos"][simFileIdx].resampleInfo.endIdx
                        inc = converter["simInfos"][simFileIdx].resampleInfo.increment
                        fNode = ET.Element("File");
                        root.append(fNode)
                        fNode.attrib['simFile'] = f;
                        fNode.attrib['outputFile'] = converter["outputFile"]
                        fNode.attrib['useRange'] = "true"
                        fNode.attrib["startIdx"] = str(sIdx)
                        fNode.attrib["endIdx"] = str(eIdx)
                        fNode.attrib["increment"] = str(inc)
                        #fNode.attrib["mappedStartIdx"] = str(converter["mappedStartIdx"]) # no need to set, but make explicit!
                        
                    
                        
                   
            tree = ET.ElementTree(root);

            f = open(processFile.format(procIdx),"w+")
            f.write(self.cf.prettifyXML(root))
            f.close()
        print("Wrote converter process files for all ranks")