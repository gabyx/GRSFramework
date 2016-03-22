from contextlib import contextmanager
import sys, os,struct,numpy,re,io,csv
import collections

 
@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        try:  
            yield
        finally:
            sys.stdout = old_stdout


class SimFileException(Exception):
    def __init__(self,string):
        self.string = string
    def __str__(self):
        return self.string


class BinaryReaderEOFException(Exception):
    def __init__(self):
        pass
    def __str__(self):
        return 'Not enough bytes in file to satisfy read request'

class BinaryReader:
    # Map well-known type names into struct format characters.
    typeNames = {
        'int8'   :'b',
        'uint8'  :'B',
        'int16'  :'h',
        'uint16' :'H',
        'int32'  :'i',
        'uint32' :'I',
        'int64'  :'q',
        'uint64' :'Q',
        'float'  :'f',
        'double' :'d',
        'char'   :'s'}

    def __init__(self, fileName):
        self.file = open(fileName, 'rb')
        
    def getSize(self, typeName):
        typeFormat = BinaryReader.typeNames[typeName.lower()]
        return struct.calcsize(typeFormat)
        
    def read(self, typeName):
        typeFormat = BinaryReader.typeNames[typeName.lower()]
        typeSize = struct.calcsize(typeFormat)
        value = self.file.read(typeSize)
        if typeSize != len(value):
            raise BinaryReaderEOFException
        return struct.unpack(typeFormat, value)[0]

    def readS(self,typeName,n):
        typeFormat = BinaryReader.typeNames[typeName.lower()]
        typeSize = struct.calcsize(typeFormat)
        value = self.file.read(typeSize*n)
        return struct.unpack(('%d'%n)+typeFormat, value)

    
    def __del__(self):
        self.file.close()


#class BinaryWriter:
    ## Map well-known type names into struct format characters.
    #typeNames = {
        #'int8'   :'b',
        #'uint8'  :'B',
        #'int16'  :'h',
        #'uint16' :'H',
        #'int32'  :'i',
        #'uint32' :'I',
        #'int64'  :'q',
        #'uint64' :'Q',
        #'float'  :'f',
        #'double' :'d',
        #'char'   :'s'}

    #def __init__(self, fileName):
        #self.file = open(fileName, 'wb')
        
    #def getSize(self, typeName):
        #typeFormat = BinaryReader.typeNames[typeName.lower()]
        #return struct.calcsize(typeFormat)
        
    #def write(self, typeName):
        #typeFormat = BinaryReader.typeNames[typeName.lower()]
        #typeSize = struct.calcsize(typeFormat)
        #value = self.file.read(typeSize)
        #if typeSize != len(value):
            #raise BinaryReaderEOFException
        #return struct.unpack(typeFormat, value)[0]

    #def writeS(self,typeName,n):
        #typeFormat = BinaryReader.typeNames[typeName.lower()]
        #typeSize = struct.calcsize(typeFormat)
        #value = self.file.read(typeSize*n)
        #return struct.unpack(('%d'%n)+typeFormat, value)

    
    #def __del__(self):
        #self.file.close()
        
        
def index(l, f):
     return next((i for i in range(len(l)) if f(l[i])), None)        
def indexr(l, f):
     return next((i for i in reversed(range(len(l))) if f(l[i])), None)        
     
def getBodyNr(bodyId): #64bit 
#    return bodyId
    return bodyId & 0xffffffff
    
def getGroupNr(bodyId): #64bit     
    return bodyId >> 32

class RigidBodyState:
    def __init__(self,nStates,nDofqObj,nDofuObj, additionalBytesType=1):
        self.nDofqObj = nDofqObj;
        self.nDofuObj = nDofuObj;
        self.additionalBytesType =  additionalBytesType;   
        self.nStates = nStates;
        self.valid = numpy.zeros((nStates,1), dtype=numpy.uint8)
        
        self.q = numpy.zeros((nStates,self.nDofqObj), dtype=numpy.float64)
        self.u = numpy.zeros((nStates,self.nDofuObj), dtype=numpy.float64)

        self.addData = dict();

        if self.additionalBytesType == 1:
            rank = numpy.zeros((nStates,1), dtype=numpy.int32)
            self.addData['rank'] = rank;
        elif self.additionalBytesType == 2:
            rank = numpy.zeros((nStates,1), dtype=numpy.int32)
            selfaddData['rank'] = rank;
            material = numpy.zeros((nStates,1), dtype=numpy.int32)
            self.addData['meterial'] = material;
        elif self.additionalBytesType == 3:
            rank = numpy.zeros((nStates,1), dtype=numpy.int32)
            self.addData['rank'] = rank;
            material = numpy.zeros((nStates,1), dtype=numpy.int32)
            self.addData['material'] = material;
            totalOverlap = numpy.zeros((nStates,1), dtype=numpy.float64)
            self.addData['overlap'] = totalOverlap;
        elif self.additionalBytesType == 4:
            rank = numpy.zeros((nStates,1), dtype=numpy.int32)
            self.addData['rank'] = rank;
            totalOverlap = numpy.zeros((nStates,1), dtype=numpy.float64)
            self.addData['overlap'] = totalOverlap;
        elif self.additionalBytesType == 5:
            rank = numpy.zeros((nStates,1), dtype=numpy.int32)
            self.addData['rank'] = rank;
            material = numpy.zeros((nStates,1), dtype=numpy.int32)
            self.addData['material'] = material;
            totalOverlap = numpy.zeros((nStates,1), dtype=numpy.float64)
            self.addData['overlap'] = totalOverlap;
            geomId = numpy.zeros((nStates,1), dtype=numpy.int32)
            self.addData['geomId'] = geomId;
    
    def __getitem__(self,key):
        if(key==0):
            return self.q;
        elif(key==1):
            return self.u;
        elif(key==2):
            return self.addData;
    
class DynamicsState:

    def __init__(self):
        self.bodies = dict(); 
        self.bodiesLog = [] # for write access

    def reset(self,nStates):
        self.t = numpy.zeros(nStates, dtype=numpy.float64)
        return self

    def resetBody(self,bodyId, nStates, nDofqObj,nDofuObj, additionalBytesType=1):
        self.bodies[bodyId] = RigidBodyState(nStates,nDofqObj,nDofqObj,additionalBytesType);
        return self;
         
    def changeBodyId(self,bodyId, newBodyId):
        if( bodyId in self.bodies ):
            entry = self.bodies[bodyId];
            self.bodies.pop(bodyId);
            self.bodies[newBodyId] = entry
        
    def getIds(self):
        return list(self.bodies.keys());
        
    def getNBodies():
        return len(self.bodies);
    
   
    
    def __str__(self):
        return "DynamicsState id:" + str(id(self)) +  "\n" + \
        "nSimBodies: " + str(len(self.bodies))  + "\n" + \
        "minTime: " + str(min(self.t)) + "\n" + \
        "maxTime: " + str(max(self.t)) + "\n" + \
        "body ids: " + str(self.getIds())


class DynamicsStateIterator:
    def __init__(self,simFile, currStateId):
        self.sF = simFile;
        self.currStateId = currStateId;
    def __iter__(self):
        return self;
    def __next__(self):
        if(currStateId < simFile.readNStates):
            
            dynState = DynamicsState()
            dynState.reset(1)
            self._readStates(dynState,1);
            return dynState;
            
        else:
            raise StopIteration()

class SimFile:
    def __init__(self, path, timeRange, bodyRange):
        self.timeRange = timeRange;
        self.bodyRange = bodyRange;
        self.sF = BinaryReader(path);
        
        self.signature = b'MBSF';
        print("Read in SimFile at: " + path)
        self.sF = BinaryReader(path)
        
        self.sF.file.seek(0,2)
        self.nBytes = self.sF.file.tell()
        self.sF.file.seek(0,0)
        print("--> File has: " + str(self.nBytes/(1024*1024)) + " mb")
        
        _signature = self.sF.file.read(len( self.signature));
        
        if(_signature == self.signature):
            print("--> SimFile signature: " + str( self.signature) + " found")
        else:
            raise SimFileException("SimFile signature: " + str( self.signature) + " not found!")
        
        self.version = self.sF.read('uint32');
        print("--> SimFile version: " + str(self.version))
        
        self.nSimBodies, self.nDofqObj , self.nDofuObj = self.sF.readS('uint32',3)
        print("--> nSimBodies: " + str(self.nSimBodies))
        print("--> nDofqObj: " + str(self.nDofqObj))
        print("--> nDofuObj: " + str(self.nDofuObj))
        self.additionalBytesType = self.sF.read('uint32')
        self.additionalBytesPerBody = self.sF.read('uint32')
        print("--> Add. Bytes Type: " + str(self.additionalBytesType))
        print("--> Add. Bytes Per Body: " + str(self.additionalBytesPerBody))
        
        self.nBytesHeader = int((len(_signature) + 1*self.sF.getSize('uint32') + 3*self.sF.getSize('uint32') + 2*self.sF.getSize('uint32')))
        
        self.nBytesBodyId = 1*self.sF.getSize('uint64')
        self.nBytesBodyState = int( (self.nDofqObj+self.nDofuObj)*self.sF.getSize('double') + self.nBytesBodyId + self.additionalBytesPerBody);
        
            
        self.nBytesState = int(   self.nSimBodies*(self.nBytesBodyState) +  1*self.sF.getSize('double') );
        self.nStates = int((self.nBytes - self.nBytesHeader) / (self.nBytesState))
        self.nAdditionalWrongBytes = self.nBytes - self.nBytesHeader - self.nStates*self.nBytesState
        
        print("--> nBytesHeader: " + str(self.nBytesHeader))
        print("--> nBytesBodyState: " + str(self.nBytesBodyState))
        print("--> nBytesState: " + str(self.nBytesState))
        print("--> nStates: " + str(self.nStates))
        print("--> nAdditionalWrongBytes: " + str(self.nAdditionalWrongBytes))
        
        #Read in all times
        t=[]
        for i in range(self.nStates):
            t.append(self.sF.read('double'))
            self.sF.file.seek((self.nBytesBodyState)*self.nSimBodies,1)
        
        self.minTime = min(t)
        print("--> minTime: " + str(self.minTime))
        self.maxTime = max(t)
        print("--> maxTime: " + str(self.maxTime))
        
         # convert to numpy arrays
        self.t = numpy.array(t)
        
        if(self.timeRange[0] < self.minTime or self.timeRange[0] > self.maxTime):
            self.timeRange[0] = self.minTime;
        
        if(self.timeRange[1] < self.minTime or self.timeRange[1] > self.maxTime):
           self.timeRange[1] = self.maxTime
        print("Filtering Simfile: t = "+str(self.timeRange))

        self.startStateIdx = index(t,lambda t: t>=timeRange[0])
        self.endStateIdx = index(t,lambda t: t>=timeRange[1])
        self.readNStates = self.endStateIdx - self.startStateIdx + 1    
        print("--> readNStates: " + str(self.readNStates))

        if(self.bodyRange[0]  < 0 or self.bodyRange[1]  < 0 or self.bodyRange[1] - self.bodyRange[0] < 0):
            raise NameError("bodyRange: " + str(self.bodyRange) + " out of bounds!")
            
    def readAll(self):

        self.sF.file.seek(self.nBytesHeader,0);
        self.sF.file.seek(self.nBytesState*self.startStateIdx,1);
        
        dynState = DynamicsState()
        dynState.reset(self.readNStates)
        self._readStates(dynState,self.readNStates);
        
        ids = dynState.getIds();
        print("--> Time t:" , dynState.t)
        print("--> BodyIds: min/max ", min(ids), max(ids))
        print("--> Number of BodyIds: ", len(ids))

        return dynState;
        
    def _readStates(self,dynState,nStates):
        
        for j in range(nStates):
            
            readBodies = 0;
            time = self.sF.read('double')
            dynState.t[j] = time;
            
            #we have an id, we need to go through the whole state and add these bodies which belong to the range
            for i in range(self.nSimBodies):
                
                #read id
                id = getBodyNr(self.sF.read('uint64'))

                if( self.bodyRange[0] <= id and id <= self.bodyRange[1]):
                    
                    # add this body
                    if id not in dynState.bodies:
                        rbs = RigidBodyState(nStates,self.nDofqObj,self.nDofuObj,self.additionalBytesType);
                        dynState.bodies[id] = rbs;
                    else:
                        rbs = dynState.bodies[id];
                    
                    readBodies += 1;       
                    #read values
                    rbs.valid[j] = 1;
                    
                    rbs.q[j,:]= self.sF.readS('double',self.nDofqObj)   # q
                    rbs.u[j,:]= self.sF.readS('double',self.nDofuObj)   # u
                    
                    if self.additionalBytesType == 1:
                        rbs.addData['rank'][j,:] = self.sF.readS('uint32',1)      # rank
                    elif self.additionalBytesType == 2:
                        rbs.addData['rank'][j,:] = self.sF.readS('uint32',1)      # rank
                        rbs.addData['material'][j,:] = sF.readS('uint32',1)  # material
                    elif self.additionalBytesType == 3:
                        rbs.addData['rank'][j,:] = self.sF.readS('uint32',1)      # rank
                        rbs.addData['material'][j,:] = self.sF.readS('uint32',1)  # material
                        rbs.addData['overlap'][j,:] = self.sF.readS('double',1)   # overlap
                    elif self.additionalBytesType == 4:
                        rbs.addData['rank'][j,:] = self.sF.readS('uint32',1)      # rank
                        rbs.addData['overlap'][j,:] = self.sF.readS('double',1)   # overlap
                    elif self.additionalBytesType == 5:
                        rbs.addData['rank'][j,:] = self.sF.readS('uint32',1)      # rank
                        rbs.addData['material'][j,:] = self.sF.readS('uint32',1)  # material
                        rbs.addData['overlap'][j,:] = self.sF.readS('double',1)   # overlap
                        rbs.addData['geomId'][j,:] = self.sF.readS('uint32',1)   # overlap
                    else:
                        self.sF.file.seek(self.additionalBytesPerBody,1)
                        
                    # Calculate actual sum, for all other states, 
                    if  readBodies == self.bodyRange[1] - self.bodyRange[0] + 1:
                        # All bodies added, in the range
                        # skip other bodies
                        self.sF.file.seek((self.nSimBodies - i -1)*self.nBytesBodyState ,1)
                        break;
                    
                else: # if body does not match the range
                    #print("Skip body id: " + str(id))
                    self.sF.file.seek(self.nBytesBodyState - self.nBytesBodyId ,1)
                
            #endfor bodies
            
        #endfor
        
    def __iter__(self):
        return DynamicsStateIterator(self,0);    
    

def ReadInSimFile(path, timeRange, bodyRange, supressOutput = True):
    if(supressOutput):  
        with suppress_stdout():
            return _ReadInSimFile(path, timeRange, bodyRange)
    else:
            return _ReadInSimFile(path, timeRange, bodyRange)


def _ReadInSimFile(path, timeRange, bodyRange):
    simFile = SimFile(path,timeRange, bodyRange);
    return simFile.readAll();



    

#Usage =================================================================
#bodyRange = [0,3]
#timeRange = [0,50]
#folderNumber=0
#dynState1 = ReadInSimFile("../SimFiles/SimDataRECORDGUI_%d/SimulationState.sim" % folderNumber,timeRange,bodyRange,False)
#print(dynState1)
#dynState2 = ReadInSimFile("../GlobalFolder/SimFiles/SimDataRECORDMPI_%d/SimDataMPIUnordered.simmpi" % folderNumber,timeRange,bodyRange,False)
#print(dynState2)
# ======================================================================

#Reads in a SimData.dat file and returns a numpy array!
def ReadInSimDataFile(simDataFile):
    # open and read header
    with open(simDataFile, 'r') as f:
        reader = csv.reader(f, delimiter='\t', quoting=csv.QUOTE_NONE)
        header = next(reader)
    print("Header is " , header)
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
    
    # build dtype array        
    dt = [(s,numpy.float64) for s in columnNames];
    print("columnNames: " ,columnNames)
    # load structured unit        
    SimData = numpy.atleast_1d(numpy.loadtxt(simDataFile,dtype=dt, delimiter="\t", comments='#'))
    return SimData;

#Reads in a SimDataIteration.dat file and returns a list of numpy arrays!
def ReadInSimDataIterationFile(simDataIterationFile):
    data=[]
    with open(simDataIterationFile, 'r') as f:
        for line in f:
            data.append(numpy.loadtxt(io.StringIO(line)))
    return data;
