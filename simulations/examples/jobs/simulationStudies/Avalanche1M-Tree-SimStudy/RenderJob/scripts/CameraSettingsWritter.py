#!/usr/bin/env python 
import numpy as np
import os;

def toTransform(pos,quat):
    T = np.zeros((4,4),dtype=numpy.float64);
    
    fTx  = 2.0*quat[1];
    fTy  = 2.0*quat[2];
    fTz  = 2.0*quat[3];
    fTwx = fTx*quat[0];
    fTwy = fTy*quat[0];
    fTwz = fTz*quat[0];
    fTxx = fTx*quat[1];
    fTxy = fTy*quat[1];
    fTxz = fTz*quat[1];
    fTyy = fTy*quat[2];
    fTyz = fTz*quat[2];
    fTzz = fTz*quat[3];
    
    T[0,0] = 1.0-(fTyy+fTzz);
    T[0,1] = fTxy-fTwz;
    T[0,2] = fTxz+fTwy;
    T[1,0] = fTxy+fTwz;
    T[1,1] = 1.0-(fTxx+fTzz);
    T[1,2] = fTyz-fTwx;
    T[2,0] = fTxz-fTwy;
    T[2,1] = fTyz+fTwx;
    T[2,2] = 1.0-(fTxx+fTyy);
    
    T[0:3,3] = pos; 
    T[3,3] = 1.0;
    return T;

# Rendermand saves matrices in row-major order!
def toRendermanTransform(T):
    s = 'ConcatTransform  ['
    for x in np.nditer(T, order='C'):
        s+= str(x) + " ";
    s+= "]" + os.linesep;
    return s;

def normalize(v):
    norm=np.linalg.norm(v)
    if norm==0: 
       return v
    return v/norm

def calculateLookAt( fromPos, toPos, upAxis):
    zAxis = normalize(toPos - fromPos);
    yAxis = normalize(np.cross(zAxis, upAxis))
    xAxis = np.cross(yAxis,zAxis)
    A_IC = np.reshape(np.concatenate((xAxis,yAxis,zAxis), axis=0),(3,3)).T # Rotate Camera with respect to World Frame
    #print(A_IC)
    A_CR = np.zeros((3,3),dtype=np.float64); 
    A_CR[1,0]=1;
    A_CR[0,1]=1;
    A_CR[2,2]=1;
    
    R_RI = A_IR = A_IC.dot(A_CR)  # A_IR = R_RI (rotation from frame I to renderman camera frame)
    
    TR = np.eye(4);
    # We want y = [ R_IR ,   R_t_RI    ; 
    #                  0   ,   1      ]   * y
    # Renderman needs the transpose of this because it is left mulitplying by a row!
    TR[0:3,0:3] = R_RI.T ;
    TR[0:3,3]   = A_IR.T.dot( - fromPos );
    return TR.T



cameraFile = "./ribarchives/CameraTransform.rib"


#Close Position
#pos = np.array([0.2,-0.4,0.1])
#at  = np.array([0.2,0.0,0.1])
#up = np.array([0,0,1])

##Good Position
#pos = np.array([0.1,-0.4,0.4])
#at  = np.array([0.2,0.0,0.15])
#up = np.array([0,0,1])

#Far Position
pos = np.array([-0.5,-0.8,0.6])
at  = np.array([-0.25,-0.15,0])
up = np.array([0,0,1])

# pos = np.array([0.0,-0.6,0.0])
# at  = np.array([0.0,0,0.0])
# up = np.array([0,0,1])

T = calculateLookAt(pos,at, up );
print(T)
s = toRendermanTransform(T);
print(s)

f = open(cameraFile, mode='w',);
f.write(s);
f.close();
