import os,pickle, gzip,re,copy
import numpy as np
import numpy.lib.recfunctions as rfn
import csv


class SimDataConsolidate:
  
    def __init__(self):
      pass
      
    def load(self,
             folderPath, 
             simDataRelPath="./SimData.dat",  
             regExProcessFolder=".*ProcessMPI_(\d*).*"):
               
        processNumbers = [];
        # Loop through process folder
        simDataFiles = dict();
        for d in os.listdir(folderPath):
            p = os.path.normpath(os.path.join(folderPath,d));
            if os.path.isdir(p):
                res = re.match(regExProcessFolder,p)
                if(res):
                    #print("Adding folder: ", p)
                    processNumber = int(res.group(1));
                    processNumbers.append(processNumber)
                    simDataFiles[processNumber] = os.path.normpath(os.path.join(p, simDataRelPath));
        
        if not processNumbers:
            raise ValueError("No processes found")
        
        print("SimDataFiles from %i processes: " % max(processNumbers) )
        
        self.procIndices , self.simDataList =  self.combineSimData(simDataFiles);  
        # first is the consolidation, second are all others
        
    def loadCombined(self,filePaths, keys=["All","Total","Average","Min"], rowSlice=None):
        self.procIndices = None
        self.simDataList = None
        # stack all combined data under each other
        
        if not isinstance(filePaths,list):
          filePaths = [filePaths]
        
        for filePath in filePaths:
            print("loadCombined: stack %s" % filePath)
            f = gzip.open(filePath, 'rb')
            l=pickle.load(f)
            
            if self.simDataList is None:
                self.simDataList = l
            else:
                # stack together!
                for k in keys:
                    if rowSlice is None:
                        self.simDataList[k] = rfn.stack_arrays( [ self.simDataList[k], l[k] ] )
                    else:
                        self.simDataList[k] = rfn.stack_arrays( [ self.simDataList[k], l[k][rowSlice,:] ] )
        
    def saveCombined(self,path):
        f = gzip.open(path, 'wb+')
        pickle.dump(self.simDataList,f)
        return path
    

    def combineSimData(self,simDataFiles):

        # add all states from processes
        simDataList = dict()
        for processNr, simDataFile in simDataFiles.items():
            cNT = self.generateColumnNames(simDataFile)
            dt = [(s,np.float64) for s in cNT]
               
            # load structured unit        
            simDataList[processNr] = np.loadtxt(simDataFile,dtype=dt, delimiter="\t", comments='#');
            
            if processNr == 0:
                origcNT = cNT
                print("SimulationTime: max:" , np.max(simDataList[0]['SimulationTime']))
                
            if(cNT != origcNT):
                raise NameError(str(cNT) + " is not the same as " + str(origcNT))
            
            
            # add pot energy (if not already existing)
            if not "TotalPotEnergy" in simDataList[processNr].dtype.names:
                
                simDataList[processNr] = recf.append_fields(
                                  simDataList[processNr], 
                                  "TotalPotEnergy" , 
                                  simDataList[processNr] ["TotalStateEnergy"] -
                                    simDataList[processNr] ["TotalKinEnergy"] 
                                  )
            
            
            
        #determine common number of rows for SimDataTotal (not all processes might have written equal amount of rows... -> shutdown)
        minRows = np.shape(simDataList[0])[0];
        for simData in simDataList.values():
            minRows = min(minRows, np.shape(simData)[0]);
        print("common rows: " + str(minRows));
        
        #build data tensor for all time series
        procIndices = sorted(list(simDataList.keys()))
        dataAll = {"SimulationTime" : simDataList[0]["SimulationTime"][0:minRows]}
        for t in origcNT :
            if t != "SimulationTime":
                data = np.ndarray((len(simDataList),minRows),dtype=np.float64);
                for i,p in enumerate(procIndices):
                    data[i,:] = simDataList[p][t][0:minRows]
                dataAll[t] = data

        
        #make dictionary for total maximal values
        SimDataTotal = {"SimulationTime" : dataAll["SimulationTime"]}
        for key in ['TotalStateEnergy','TotalPotEnergy',
                    'TotalKinEnergy','nContacts']:
            SimDataTotal[key] = np.sum(dataAll[key],axis=0)
        
        for key in ['MaxOverlap']:
            SimDataTotal[key] = np.amax(dataAll[key],axis=0)
            
        # find indices of processes with maximal timestep time
        indices=np.argmax(dataAll["TimeStepTime"],axis=0)
        
        for key in ['TimeStepTime','CollisionTime',
                    'InclusionTime',
                    'BodyCommunicationTime']:
            SimDataTotal[key] = dataAll[key][indices,range(minRows)] 
            # select the values for the process with max timtestep time
        
        
        
        # make dictionary for average values over all processes
        SimDataAvg = {"SimulationTime" : dataAll["SimulationTime"]}
        for key in ['TimeStepTime','CollisionTime','InclusionTime',
                    'BodyCommunicationTime','AvgIterTime','MaxOverlap',
                    'TotalTimeProx','IterTimeProx']:
            SimDataAvg[key] = np.average(dataAll[key],axis=0);
        
        # make minimum values
        SimDataMin = {"SimulationTime" : dataAll["SimulationTime"]}
        for key in ['TimeStepTime','CollisionTime','InclusionTime',
                        'BodyCommunicationTime','AvgIterTime','MaxOverlap',
                        'TotalTimeProx','IterTimeProx']:
            SimDataMin[key]   = np.amin(dataAll[key],axis=0 )
        
        
        simDataList["All"]     = dataAll
        simDataList['Average'] = SimDataAvg;
        simDataList['Total']   = SimDataTotal;
        simDataList['Min']     = SimDataMin;
        
        # print some statistics
        summedTimeStepTime = np.sum(SimDataAvg['TimeStepTime'])/60
        summedMaxTimeStepTime = np.sum(SimDataTotal['TimeStepTime'])/60
        globalTotalTime = (simDataList[0]["GlobalTime"][-1] - simDataList[0]["GlobalTime"][0])/60
        print("Summed TimeStepTime  [min]: " + str(summedTimeStepTime))
        print("Summed max. TimeStepTime  [min]: " + str(summedMaxTimeStepTime))
        print("Global Time (proc 0) [min]: " + str(globalTotalTime))
  
        return procIndices, simDataList;
    
    def generateColumnNames(self,simDataFile):
        # open and read header
        with open(simDataFile, 'r') as f:
            reader = csv.reader(f, delimiter='\t', quoting=csv.QUOTE_NONE)
            header = next(reader)
        #print("Header is " , header)
        # Replace "#"
        header[0] = header[0].replace('#',"")
        header = [s.strip() for s in header]
        
        columnNames =[]
        for i,s in enumerate(header):
            #print(i,s)
            res = re.match('^([\w#]*\s?)*\w+',s)
            if(res):
                columnNames.append(res.group(0))
            else:
                raise NameError("No match in string " + str(s))
                
        return columnNames;
    