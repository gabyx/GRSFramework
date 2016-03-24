import numpy as np;

#do the loading of the obj file
def loadObj(filename) :
 V = [] #vertex
 T = [] #texcoords
 N = [] #normals
 F = [] #face indexies

 fh = open(filename)
 for line in fh :
  if line[0] == '#' : continue
 
  line = line.strip().split(' ')
  if line[0] == 'v' : #vertex
   V.append(line[1:])
  elif line[0] == 'vt' : #tex-coord
   T.append(line[1:])
  elif line[0] == 'vn' : #normal vector
   N.append(line[1:])
  elif line[0] == 'f' : #face
   face = line[1:]
   if len(face) != 4 : 
    print(line)
    #raise Exception('not a quad!')
    continue
   for i in range(0, len(face)) :
    face[i] = face[i].split('/')
    # OBJ indexies are 1 based not 0 based hence the -1
    # convert indexies to integer
    for j in range(0, len(face[i])) : face[i][j] = int(face[i][j]) - 1
   F.append(face)

 return np.array(V,dtype=float), np.array(T,dtype=float), np.array(N,dtype=float), np.array(F,dtype=float)
