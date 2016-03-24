from contextlib import contextmanager
import xml.etree.ElementTree as ET;
import numpy as np;

from . import TopologyInfo as tpI
from . import Utilities as ut

@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        try:  
            yield
        finally:
            sys.stdout = old_stdout


class NearestDistances:
     def __init__(self,distances,mean,stdDev):
        self.distances = distances
        self.mean = mean
        self.stdDev = stdDev

class TopologyInfo:
     def __init__(self,filePath,time,buildMode, aabbs, topo, points, nearestDists):
        self.filePath = filePath;
        self.time = time;
        self.buildMode = buildMode;
        self.aabbs = aabbs;
        self.topo = topo;
        self.points = points;
        self.nearestDists = nearestDists;


def parseTopologyInfoXML(file, supressOutput = False, **kargs):
    if(supressOutput):  
        with suppress_stdout():
            return parseTopologyInfoXML_imp(file, **kargs)
    else:
            return parseTopologyInfoXML_imp(file, **kargs)


def parseTopologyInfoXML_imp(file, 
                            loadPoints = True, 
                            loadHistogram = True):

    tree = ET.parse(file)
    root = tree.getroot()
    
    descriptions ={};
    
    #Parse Descriptions ==========================
    for child in root.find("./Description"):
        descriptions[child.tag] = child.text;
    
    topoType = root.attrib["type"];
    print("Topo type: %s " % topoType)
    buildMode = root.attrib["buildMode"];
    
    #Parse Time
    time = float(root.attrib["time"])
    print("Time t:", time)

    #Parse AABB List ==================================
    aabbList = root.find("./AABBList")
    aabbs = np.array((0,0));
    if aabbList is not None:
        dt = np.dtype([('rank', np.uint), ('min', (float,3) ), ('max',(float,3) )]);
        aabbs = ut.parseNDArray(aabbList.text,dt)
        print("AABBs: " ,aabbs);
    else:
        print("No AABBs found!");

    if(topoType == "Grid"): 
        g = root.find("./Grid")
        topo = tpI.Grid.parseFromXML(g)
    elif (topoType == "KdTree"):
        g = root.find("./KdTree")
        topo = tpI.KdTree.parseFromXML(g)

    else:
        raise NameError("Topo type: " + topoType + " not implemented");

     # Parse points
    pointNode = root.find("./Points")
    points = None
    if loadPoints:
        if pointNode is not None:
            dt = float;
            points = ut.parseNDArray(pointNode.text,dt)
            print("Points: " ,np.shape(points))
        else:
            print("No points!")
    else:
        print("Loading points supressed")
        
    # Nearest Distance Histogram
    ndNode = root.find("./NearestDistanceHistogram")
    
    nearestDists = None
    if loadHistogram:
        if ndNode is not None:
            dt = float;
            distances = ut.parseNDArray(ndNode.text,dt)
            mean = float(ndNode.attrib["mean"])
            stdDev = float(ndNode.attrib["stdDeviation"])
            nearestDists = NearestDistances(distances,mean,stdDev)
        
        else:
            print("No nearest distances !")
    else:
        print("Nearest distances supressed")
        
    return TopologyInfo(file,time,buildMode,aabbs,topo,points,nearestDists)