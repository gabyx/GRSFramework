import os,copy

class FrameGenerator:
    
    def __init__(self,pipelineSpecs, jobGenModules):
        
        self.cf = jobGenModules["commonFunctions"]
        self.iH = jobGenModules["importHelpers"]
        self.gSFI = jobGenModules["getSimFileInfos"]
        
        self.pipelineSpecs = pipelineSpecs
        self.pipelineTools = pipelineSpecs["pipelineTools"]
        
    def __call__(self,frameList,studyEntryDataFile,extractStateCount, simInfoApp):
      
        print("FrameGenerator GridVelocity ============================")
        # parse studyEntryDataFile
        studiesEntryData = self.cf.jsonLoad(studyEntryDataFile)
        studies = dict( (int(k),v) for k,v in studiesEntryData["studies"].items() )
        # take only relvant studies
        
        studyNrs = [ f["studyNr"] for f in frameList ]
        print("Relevant study nrs: " , studyNrs)
        
        # take only relevant study entry info
        for k in list(studies.keys()):
          if k not in studyNrs:
            del studies[k]
        
        # find minimal time 
        getRange = lambda study: study["finalStateIdx"] - study["entryStateIdx"]
        studyNrMinRange = min(studies, key = lambda nr : getRange(studies[nr]))
        
        minRange = getRange(studies[studyNrMinRange])
        print("---> minimal range [entryStateIdx;finalStateIdx] for studyNr : ", studyNrMinRange, "=" , minRange, "states")
        print("---> extract state count: ", extractStateCount)
        extractStepSize = int(getRange(studies[studyNrMinRange]) / extractStateCount)
        if extractStepSize <= 0:
          extractStepSize = 1
        print("---> extraction step size: ", extractStepSize)
        
        for frame in frameList:
          
          # iterate over all simfiles, and get info and especially state indices list!
          startIdx = studies[frame["studyNr"]]["entryStateIdx"]
          endIdx = startIdx + extractStateCount * extractStepSize
          print("---> Get sim info for startIdx:",startIdx,"and endIdx:",endIdx," and increment:",extractStepSize)
          infos = self.gSFI.getSimFileInfos(frame["simFiles"], 
                                            sIdx = startIdx, incr = extractStepSize, 
                                            eIdx = endIdx,
                                            skipFirstState = True,
                                            app=simInfoApp, returnList = True);
                                    
          frame["simInfos"] = infos
        
        # Build all frames
        
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
            "status":"convert",
            "entryData" : studies[frame["studyNr"]],
            "extractStepSize" : extractStepSize
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
        print("========================================================")

        return (allFrames,framesPerIdx,framesPerIdx.values())
        # return all frames and frames per frameIdx, and a sortedFrames list which is used to distribute over the processes 