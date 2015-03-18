/*
 *  GRSF/Dynamics/Collision/CollisionFunctions.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef GRSF_Dynamics_Collision_CollisionFunctions_hpp
#define GRSF_Dynamics_Collision_CollisionFunctions_hpp

#include <cmath>
#include <vector>
#include "boost/tuple/tuple.hpp"

#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Dynamics/General/MeshData.hpp"

/**
* @brief Collision helper functions for the Collider!
*/


namespace CollisionFunctions {

    DEFINE_MATRIX_TYPES


    ///< Halfspace/Point intersection
    template<typename Derived1, typename Derived2>
    inline bool collideHalfspacePointAndProx( const MatrixBase<Derived1> & I_pos,
                                              const MatrixBase<Derived2> & I_normal,
                                              Vector3 & I_p)
    {
        //t = n * r_PS ( P=point, S=center of plane ), if t>=0 intersect!
        PREC t = I_normal.dot(I_pos - I_p) /*+  halfspace->m_normal.dot(halfspace->m_pos)*/;
        if( t >= 0.0) {
            // project onto plane
            I_p += t*I_normal;
            return true;
        }
        return false;
    }

    ///< Sphere/Point intersection
    template<typename Derived>
    inline bool collideSpherePointAndProx( const MatrixBase<Derived> & I_center,
                                           PREC radius,
                                           Vector3 & I_p)
    {
        // d = c1 - p;
        // p = d - d/norm(d) * radius + p -->  is the projection onto the sphere

        Vector3  dist = (I_center - I_p);
        PREC dsqr = dist.squaredNorm();

        if(dsqr <= radius*radius){
            //we have a collision
            PREC d = sqrt(dsqr);
            //if the spheres are practically concentric just choose a random direction
            //to avoid division by zero
            if(dsqr < std::numeric_limits<PREC>::epsilon()) {
                I_p  += Vector3(0,0,radius);
            }else{
                I_p  += (1.0 - radius / sqrt(dsqr)) * dist; // project onto sphere
            }
            return true;
        }
        return false;
    }

    template<typename Derived1, typename Derived2>
    bool collideCapsulePointAndProx( const MatrixBase<Derived1> & I_pos,
                                     const MatrixBase<Derived2> & I_normal,
                                     PREC length,
                                     PREC radius,
                                     Vector3 & I_p)
    {
       const PREC halfL( 0.5*length );  // Half cylinder length

        // Calculating the component in the normal direction of the sphere in the frame K of the capsule
        // sphere->m_r_S - capsule->m_r_S  projecting onto the transformed normal in the I frame
        PREC z = I_normal.dot(I_p - I_pos);

        // Calculation the center of the sphere representing the capsule
        // limit the capsule representing sphere to the interval [-halfL, halfL]
        if( z > halfL ) {
            z = halfL;
        } else if( z < -halfL ) {
            z = -halfL;
        }

        // Performing a sphere-sphere collision between the colliding sphere and the
        // capsule representation
        return CollisionFunctions::collideSpherePointAndProx(I_pos + z*I_normal, radius, I_p);
    }



    using ClosestPoint = boost::tuple<double, Vector3 ,unsigned int,unsigned int>;
    using ClosestPointSet = std::vector< ClosestPoint >;



    inline Vector3 getClosestPoint_PointTriangle(   const Vector3 & I_r_S,
                                                    const Vector3 & vertex0,
                                                    const Vector3 & vertex1,
                                                    const Vector3 & vertex2,
                                                    const typename MeshData::MeshIndices & indices,
                                                    const unsigned int & indexFace,
                                                    unsigned int & type,unsigned int & id )
    {
//      ab * ( I_r_0S - I_n(I_n *(I_r_S - I_r_0)) ) == ab *  I_r_0S

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
        // Check if point lies behind ab! (normale mit vc muss negativ sein)
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


        //Otherwise we know that the point closest is the projection onto the triangle
        // Barycentric coordinates, area ratio is used to calculate these!
        type = 1;
        id = indexFace;
        double u = va / (va + vb + vc);
        double v = vb / (va + vb + vc);
        double w = 1.0 - u - v;
        return u * vertex0 + v * vertex1 + w * vertex2;

   };

     inline Vector3 getClosestPoint_PointTriangle(   const Vector3 & I_r_S,
            const MeshData & mesh,
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
        const MeshData::MeshIndices & indices = mesh.m_Faces[indexFace];
        vertex0 = mesh.m_Vertices[indices(0)];
        vertex1 = mesh.m_Vertices[indices(1)];
        vertex2 = mesh.m_Vertices[indices(2)];
        M_r_MS = A_IM.transpose() * (I_r_S - I_r_M);

        return A_IM*getClosestPoint_PointTriangle(M_r_MS,vertex0,vertex1,vertex2, indices, indexFace, type, id);

    };


    inline void getClosestPointsInRadius_PointMesh(  const Vector3 & I_r_S,
            const PREC & radius,
            const MeshData & mesh,
            const Vector3 & I_r_M,
            const Matrix33 & A_IM,
            ClosestPointSet & pointSet) {

        // Iterate over all faces
        MeshData::Faces::const_iterator faceIt;

        static Vector3 vertex0,vertex1,vertex2, M_r_MS, I_r_SC;
        unsigned int type; unsigned int id;
        bool validContact;
        int faceNr=0;

        for(faceIt = mesh.m_Faces.begin(); faceIt != mesh.m_Faces.end(); ++faceIt) {
            // Check each face!
            vertex0 = mesh.m_Vertices[(*faceIt)(0)];
            vertex1 = mesh.m_Vertices[(*faceIt)(1)];
            vertex2 = mesh.m_Vertices[(*faceIt)(2)];

//            std::cout <<"Check with faceNr" <<faceNr <<":" << vertex0.transpose() <<
//             ","<< vertex1.transpose() <<","<< vertex2.transpose() << std::endl;

            M_r_MS = A_IM.transpose() * (I_r_S - I_r_M);
            I_r_SC = A_IM*(getClosestPoint_PointTriangle(M_r_MS,vertex0,vertex1,vertex2,(*faceIt),faceNr,type,id) - M_r_MS);
            double normI_r_SC = I_r_SC.norm();
            double overlap = radius - normI_r_SC;

            //If closest point is in sphere, then add this to the set
            if(overlap >= 0.0 && normI_r_SC > 0.0){
//                std::cout <<"Collision with faceNr" <<faceNr <<":" << vertex0.transpose() <<std::endl;
                validContact = true;
                for(unsigned int j=0; j<pointSet.size(); j++) {
                    //Vector3 p1 = pointSet[j].get<1>();
                    //std::cout <<"Cos : "<< acos( pointSet[j].get<1>().dot( I_r_SC )) << "<" << (5.0/180.0*M_PI)<<std::endl;
//                    std::cout << "p1:" << p1 <<std::endl;
//                    std::cout << "I_r_SC:" << I_r_SC <<std::endl;
                    //double angle = std::acos( p1.dot( I_r_SC ) /(p1.norm() * normI_r_SC ));
//                                        std::cout << "Angle: " <<angle<< std::endl;
//                    if( angle  < (10.0/180.0*M_PI)) {
//                        validContact=false;
//                        break;
//                    }
                }
                if(validContact){
                    pointSet.push_back(ClosestPoint(overlap,I_r_SC,type,id));
                }
            }
            faceNr++;
        }
        //std::cout <<"Coll: ==========" <<  std::endl;
    };

    /* Finds the point with the maximum overlap */
     inline void getClosestPointInRadius_PointMesh(  const Vector3 & I_r_S,
            const PREC & radius,
            const MeshData & mesh,
            const Vector3 & I_r_M,
            const Matrix33 & A_IM,
            ClosestPointSet & pointSet) {

        // Iterate over all faces
        MeshData::Faces::const_iterator faceIt;

        static Vector3 vertex0,vertex1,vertex2, M_r_MS, I_r_SC;
        unsigned int type; unsigned int id;
        int faceNr=0;

        static ClosestPoint cp;
        cp.get<0>() = 0; // set overlap to zero

        for(faceIt = mesh.m_Faces.begin(); faceIt != mesh.m_Faces.end(); ++faceIt) {
            // Check each face!
            vertex0 = mesh.m_Vertices[(*faceIt)(0)];
            vertex1 = mesh.m_Vertices[(*faceIt)(1)];
            vertex2 = mesh.m_Vertices[(*faceIt)(2)];

            M_r_MS = A_IM.transpose() * (I_r_S - I_r_M);
            I_r_SC = A_IM*(getClosestPoint_PointTriangle(M_r_MS,vertex0,vertex1,vertex2,(*faceIt),faceNr,type,id) - M_r_MS);
            double overlap = radius - I_r_SC.norm();
            if(overlap>cp.get<0>()){
                //save this point
                cp.get<0>() = overlap;
                cp.get<1>() = I_r_SC;
                cp.get<2>() = type;
                cp.get<3>() = id;
            }
            faceNr++;
        }
        if(cp.get<0>()>0.0){
           pointSet.push_back(cp);
        }
        //std::cout <<"Coll: ==========" <<  std::endl;
    };



};


// do: http://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf However it's heavy on math and does not give the complete source code. Here is the complete source code:
//vector3 closesPointOnTriangle( const vector3 *triangle, const vector3 &sourcePosition )
//{
//    vector3 edge0 = triangle[1] - triangle[0];
//    vector3 edge1 = triangle[2] - triangle[0];
//    vector3 v0 = triangle[0] - sourcePosition;
//
//    float a = edge0.dot( edge0 );
//    float b = edge0.dot( edge1 );
//    float c = edge1.dot( edge1 );
//    float d = edge0.dot( v0 );
//    float e = edge1.dot( v0 );
//
//    float det = a*c - b*b;
//    float s = b*e - c*d;
//    float t = b*d - a*e;
//
//    if ( s + t < det )
//    {
//        if ( s < 0.f )
//        {
//            if ( t < 0.f )
//            {
//                if ( d < 0.f )
//                {
//                    s = clamp( -d/a, 0.f, 1.f );
//                    t = 0.f;
//                }
//                else
//                {
//                    s = 0.f;
//                    t = clamp( -e/c, 0.f, 1.f );
//                }
//            }
//            else
//            {
//                s = 0.f;
//                t = clamp( -e/c, 0.f, 1.f );
//            }
//        }
//        else if ( t < 0.f )
//        {
//            s = clamp( -d/a, 0.f, 1.f );
//            t = 0.f;
//        }
//        else
//        {
//            float invDet = 1.f / det;
//            s *= invDet;
//            t *= invDet;
//        }
//    }
//    else
//    {
//        if ( s < 0.f )
//        {
//            float tmp0 = b+d;
//            float tmp1 = c+e;
//            if ( tmp1 > tmp0 )
//            {
//                float numer = tmp1 - tmp0;
//                float denom = a-2*b+c;
//                s = clamp( numer/denom, 0.f, 1.f );
//                t = 1-s;
//            }
//            else
//            {
//                t = clamp( -e/c, 0.f, 1.f );
//                s = 0.f;
//            }
//        }
//        else if ( t < 0.f )
//        {
//            if ( a+d > b+e )
//            {
//                float numer = c+e-b-d;
//                float denom = a-2*b+c;
//                s = clamp( numer/denom, 0.f, 1.f );
//                t = 1-s;
//            }
//            else
//            {
//                s = clamp( -e/c, 0.f, 1.f );
//                t = 0.f;
//            }
//        }
//        else
//        {
//            float numer = c+e-b-d;
//            float denom = a-2*b+c;
//            s = clamp( numer/denom, 0.f, 1.f );
//            t = 1.f - s;
//        }
//    }
//
//    return triangle[0] + s * edge0 + t * edge1;
//}


#endif


