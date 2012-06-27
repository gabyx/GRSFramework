/*
 *  CollisionFunctions.hpp
 *
 *  Created by Gabriel N�tzi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef CollisionFunctions_hpp
#define CollisionFunctions_hpp

#include <vector>
#include "boost/tuple/tuple.hpp"
#include <Eigen/Dense>


#include "TypeDefs.hpp"

#include "MeshData.hpp"

/**
* @brief Collision helper functions for the Collider!
*/

template<typename TLayoutConfig>
class CollisionFunctions {

public:

    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

    typedef boost::tuple<double, Vector3 ,unsigned int,unsigned int> ClosestPoint;
    typedef std::vector< ClosestPoint > ClosestPointSet;


    inline static Vector3 getClosestPoint_PointTriangle(   const Vector3 & I_r_S,
            const MeshData<MeshPREC> & mesh,
            const Vector3 & I_r_M,
            const Matrix33 & A_IM,
            const unsigned int & indexFace,
            unsigned int & type,unsigned int & id) {

        // Assumes everything in the same frame!
        // Search the closest point to the sphere on the triangle A,B,C with vertex0,1,2 -->
        // This code only tests the closest point to the sphere center m_r_S
        // To understand this -> draw a triangle on the paper, and imagina a point in above the paper
        // somewhere in the space, this point is the center of the sphere...

        static Vector3 vertex0,vertex1,vertex2, M_r_MS;
        MeshData<MeshPREC>::MeshIndices & indices = mesh.m_Faces[indexFace];
        vertex0 = mesh.m_Vertices[indices(0)];
        vertex1 = mesh.m_Vertices[indices(1)];
        vertex2 = mesh.m_Vertices[indices(2)];
        M_r_MS = A_IM.transpose() * (I_r_S - I_r_M);

        return A_IM*getClosestPoint_PointTriangle(M_r_MS,vertex0,vertex1,vertex2, indices, indexFace, type, id);

    };



    inline static Vector3 getClosestPointsInRadius_PointMesh(  const Vector3 & I_r_S,
            const PREC & radius,
            const MeshData<MeshPREC> & mesh,
            const Vector3 & I_r_M,
            const Matrix33 & A_IM,
            ClosestPointSet & pointSet) {

        // Iterate over all faces
        MeshData<MeshPREC>::Faces::const_iterator faceIt;

        static Vector3 vertex0,vertex1,vertex2,M_r_MS, I_r_SC;
        unsigned int type; unsigned int id;
        bool validContact;
        int faceNr=0;

        for(faceIt = mesh.m_Faces.begin(); faceIt != mesh.m_Faces.end(); faceIt++) {
            // Check each face!
            vertex0 = mesh.m_Vertices[(*faceIt)(0)];
            vertex1 = mesh.m_Vertices[(*faceIt)(1)];
            vertex2 = mesh.m_Vertices[(*faceIt)(2)];
            M_r_MS = A_IM.transpose() * (I_r_S - I_r_M);
            I_r_SC = A_IM*getClosestPoint_PointTriangle(M_r_MS,vertex0,vertex1,vertex2,(*faceIt),faceNr,type,id);
            double overlap = radius - I_r_SC.norm();

            I_r_SC.normalize();
            //If closest point is in sphere, then add this to the set
            if(overlap >= 0){
                validContact = true;
//                for(unsigned int j=0; j<pointSet.size(); j++) {
//                    if( acos( pointSet[j].template get<1>().dot( I_r_SC )) < (5/180*M_PI)) {
//                        validContact=false;
//                        break;
//                    }
//                }
                if(validContact){
                    pointSet.push_back(ClosestPoint(overlap,I_r_SC,type,id));
                }
            }
            faceNr++;
        }

    };

    inline static Vector3 getClosestPoint_PointTriangle(    const Vector3 & I_r_S,
                                                            const Vector3 & vertex0,
                                                            const Vector3 & vertex1,
                                                            const Vector3 & vertex2,
                                                            const typename MeshData<MeshPREC>::MeshIndices & indices,
                                                            const unsigned int & indexFace,
                                                            unsigned int & type,unsigned int & id )
    {
        Vector3 ab = vertex1 - vertex0;
        Vector3 ac = vertex2 - vertex0;
        Vector3 bc = vertex2 - vertex1;

        // =============================================
        type = 3;
        double snom = (I_r_S - vertex0).dot(ab);
        double tnom = (I_r_S - vertex0).dot(ac);

        // Check if point lies in normal cone behind A
        id = indices(0);
        if (snom <= 0.0 && tnom <= 0.0) return vertex0;


        double sdenom = (I_r_S - vertex1).dot(-ab);
        double unom = (I_r_S - vertex1).dot(bc);
        // Checks if point lies in normal cone behind B
        id = indices(1);
        if (sdenom <= 0.0f && unom <= 0.0f) return vertex1;


        double tdenom = (I_r_S - vertex2).dot(-ac);
        double udenom = (I_r_S - vertex2).dot(-bc);
        // Checks if point lies in normal cone behind C
        id = indices(2);
        if (tdenom <= 0.0f && udenom <= 0.0f) return vertex2;

        // ==============================================
        type = 2;
        Vector3 n = ab.cross(ac);
        double vc = n.dot((vertex0 - I_r_S).cross(vertex1 - I_r_S));

        id = indices(0);
        // Check if point lies behind ab!
        if (vc <= 0.0 && snom >= 0.0 && sdenom >= 0.0)
            return vertex0 + (snom / (snom + sdenom)) * ab;

        // Check if point lies behind bc!
        id = indices(1);
        double va = n.dot((vertex1 - I_r_S).cross(vertex2 - I_r_S));
        if (va <= 0.0 && unom >= 0.0 && udenom >= 0.0)
            return vertex1 + (unom / (unom + udenom)) * bc;

        // Check if point lies behind ac!
        id = indices(2);
        double vb = n.dot((vertex2 - I_r_S).cross(vertex0 - I_r_S));
        if (vb <= 0.0 && tnom >= 0.0 && tdenom >= 0.0)
            return vertex0 + (tnom / (tnom + tdenom)) * ac;


        //Otherwise we know that the point closest is the projection onto the
        type = 1;
        id = indexFace;
        double u = va / (va + vb + vc);
        double v = vb / (va + vb + vc);
        double w = 1.0 - u - v;
        return u * vertex0 + v * vertex1 + w * vertex2;

   };

};


#endif


