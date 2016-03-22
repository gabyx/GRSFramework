

import sys
import numpy as np

class Edge:
    def __init__(self,idx1,idx2):
        self.idx1 = idx1;
        self.idx2 = idx2;

    def __hash__(self):
        if(self.idx1<self.idx2):
            return hash((self.idx1,self.idx2));
        else:
            return hash((self.idx2,self.idx1));
    def __getitem__(self,a):
        if(a==0):
            return self.idx1;
        else:
            return self.idx2;

def findOBBEdge(edges, points):
    # shift the points such that the minimum x, y, z values
    # in the entire set of points is 0.
    shift = points.min(axis=0)
    points = points - shift

    min_volume = float("inf")

    # try every pair of edges (ordering is not important)
    for idx, edge1_idx in enumerate(edges):        
        #print("edge:", edge1_idx[0], edge1_idx[0])
        e1 = points[edge1_idx[0]] - points[edge1_idx[1]]
        for idx2, edge2_index in enumerate(edges[(idx+1):]):            
            e2 = points[edge2_index[0]] - points[edge2_index[1]]

            # transform the two edges into a orthogonal basis
            w = vec_cross(e1, e2) # returns normalized vector
            u = vec_cross(w, e1)
            v = vec_cross(u, w)

            # project all the points on to the basis u1, u2 u3
            p = calcProjections(points, w, u, v)

            volume, mins, maxes = calcVolume(p)

            # we are looking for the minimum volume box
            if volume < min_volume:
                min_volume = volume
                specs = u, v, w, mins, maxes, volume

    u, v, w, mins, maxes, volume = specs

    # get the corner by using our projections, then shift it to move
    # it back into the same origin as the original set of points
    corner = u * mins[0] + v * mins[1] + w * mins[2]
    corner += shift

    # create the sides which are vectors with the magnitude the length
    # of that side
    v1 = u * (maxes[0] - mins[0])
    v2 = v * (maxes[1] - mins[1])
    v3 = w * (maxes[2] - mins[2])

    return corner, v1, v2, v3, u,v,w

def calcVolume(p):
    """Calculates the volume of the box that would encompass the given
    points using the given projection. projection is sized (NxM) where
    N is the number of points and M is the number of vectors they were
    projected onto. Also return the minimum and maximum bounds along
    each of those vectors."""

    # the minimum and maximum projection of each basis vector
    mins = p.min(axis=0)
    maxes = p.max(axis=0)

    # the volume product of each difference between the maximum and
    # minimum values from the projection onto each basis vector
    volume = np.prod(maxes - mins)

    return volume, mins, maxes  

def calcProjections(points, *vectors):
    """Calculates the projection of points (NxD) onto the vectors 
    (MxD) and return the projections p which is a matrix sized (N, M) 
    where N is the number of points and M is the number of vectors.
    p[i][j], is the projection of points[i] onto vectors[j] (which is
    between 0 and 1)."""

    u = np.array(vectors)

    # project the points onto the vectors into on fell swoop
    d = np.dot(points, u.T)

    # this is the dot product of each vector with itself
    v2 = np.diag(np.inner(u, u))

    p = d / v2

    return p

def vec_cross(u, v):
    """Return the normalized cross product of u and v."""
    w = np.cross(u, v)
    w /= np.sqrt(np.sum(v**2))
    return w
