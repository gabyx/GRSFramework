import os,copy

class FrameGenerator:
    
    def __init__(self,pipelineSpecs, jobGenModules):
        
        self.cf = jobGenModules["commonFunctions"]
        self.iH = jobGenModules["importHelpers"]
        self.gSFI = jobGenModules["getSimFileInfos"]
        
        self.pipelineSpecs = pipelineSpecs
        self.pipelineTools = pipelineSpecs["pipelineTools"]
        
    def __call__(self,frameList):
      
        allFrames = []
        
        def updateDict(d,dd):
            for key in dd["copyToFrame"]:
                if key in d :
                    raise ValueError("Key %s exists already in dict: %s" %(key,str(d)))
                d[key] = copy.deepcopy(dd[key])
            return d
            
        for frame in frameList:
          
          f = {
            "uuid" : str(self.cf.makeUUID(frame["outputFile"])), 
            "status":"convert"
          }
          f.update(frame)

          allFrames.append({ 
                              "tools": {
                                  "converter" : 
                                     updateDict( f, self.pipelineTools["converter"] )
                              },
                          "fileMover" : list([])
                         })
          
        # assign stupid index for each frame
        framesPerIdx = {  i :  f for i,f in enumerate(allFrames) }
                 
        return (allFrames,framesPerIdx,framesPerIdx.values())
        # return all frames and frames per frameIdx, and a sortedFrames list which is used to distribute over the processes 