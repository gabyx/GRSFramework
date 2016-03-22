import io
import numpy as np;
import xml.etree.ElementTree as ET
from transforms3d.quaternions import mat2quat   

from . import Utilities as ut

class AABB:
    def __init__(self,min,max):
        self.min =  min.flatten();
        self.max =  max.flatten();
    
    def center(self):
        return 0.5*(self.max + self.min);

class Grid:
    def __init__(self,processDim, aligned, aabb, A_IK):
        self.processDim = processDim;
        self.aabb = aabb;
        self.aligned = aligned;
        self.A_IK = A_IK;
        self.q_KI = mat2quat(A_IK); # A_IK = R_KI
        
    @staticmethod
    def parseFromXML(g):
        aligned = g.attrib["aligned"] in ["true","True"]
        print("aligned:",aligned)
        
        #Parse processDIm min/max A_IK
        s = g.find("./Dimension").text
        dt = np.dtype( np.uint  );
        processDim = ut.parseNDArray(s,dt);
        print("processDim:" , processDim)
        
        s = g.find("./MinPoint").text
        dt = np.dtype( float );
        minP = ut.parseNDArray(s,dt);
        print("min:" , minP)
        
        s = g.find("./MaxPoint").text
        maxP = ut.parseNDArray(s,dt);
        print("max:" , maxP)
        
        s = g.find("./A_IK").text
        dt = np.dtype( float  );
        A_IK = np.reshape(ut.parseNDArray(s,dt),(3,3));
        print("A_IK: \n", A_IK)
        
        # Parse points
      
        aabb = AABB(minP,maxP);
        return Grid(processDim,aligned,aabb,A_IK);

class KdTreeLeaf:
      def __init__(self,level,idx,aabb, points):
        self.level = level;
        self.idx = idx;
        self.points = points;
        self.aabb = aabb;  

class KdTree:
    def __init__(self, rootAABB, aligned, A_IK, leafs, aabbTree):
        self.aabb = rootAABB;
        self.aligned = aligned;
        self.leafs = leafs;
        self.aabbTree = aabbTree;
        self.A_IK = A_IK;
        self.q_KI = mat2quat(A_IK); # A_IK = R_KI
        
    def parseFromXML(g):
        dtAABB = np.dtype([('min', (float,3) ), ('max',(float,3) )]);
        dtPoints = float;
        
        aligned = g.attrib["aligned"] in ["true","True"]
        print("aligned:",aligned)
        
        # parse root
        s = g.find("./Root/AABB");
        if(s.text):
            rootAABB = ut.parseNDArray(s.text,dtAABB)
            rootAABB = AABB(rootAABB['min'],rootAABB['max'])
            print("Root AABB: " ,rootAABB);
        else:
            raise NameError("No Root AABB found!");
        
        s = g.find("./A_IK").text
        dt = np.dtype( float  );
        A_IK = np.reshape(ut.parseNDArray(s,dt),(3,3));
        print("A_IK: \n", A_IK)
        
        # parse all leafs
        leafs = g.find("./Leafs");
        leafsDict ={};
        for leaf in leafs.iter("Leaf"):
            
            level = int(leaf.attrib['level']);
            idx = int(leaf.attrib['idx']);
            
            s = leaf.find("AABB");
            if(s.text):
                leafAABB = ut.parseNDArray(s.text,dtAABB)
            else:
                raise NameError("No Leaf AABB found!");
            
            points = None
            s = leaf.find("Points");
            if s:
                points = ut.parseNDArray(s.text,dtPoints);
            
            if level not in leafsDict.keys():
                l = leafsDict[level] = [];
            else:
                l = leafsDict[level];
                   
            l.append( KdTreeLeaf(level,idx, AABB(leafAABB['min'],leafAABB['max']) ,points) )
        
        # parse all AABBSubTrees
        aabbSubTreeDict ={};
        for subtree in g.find("AABBTree").iter("AABBSubTree"):
            level = int(subtree.attrib['level']);
            aabbs = ut.parseNDArray(subtree.text,dtAABB);
            aabbSubTreeDict[level]= aabbs;
        
        
        
        return KdTree(rootAABB,aligned,A_IK,leafsDict,aabbSubTreeDict);
        