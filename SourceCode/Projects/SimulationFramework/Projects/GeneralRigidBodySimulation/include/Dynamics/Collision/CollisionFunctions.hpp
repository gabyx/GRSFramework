/*
 *  CollisionFunctions.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef CollisionFunctions_hpp
#define CollisionFunctions_hpp

#include <Eigen/Dense>

#include "TypeDefs.hpp"

#include "MeshData.hpp"

/**
* @brief Collision helper functions for the Collider!
*/

namespace CollisionFunctions{


   template<typename TLayoutConfig>
   typename TLayoutConfig::Vector3 getClosestPoint_PointTriangle(   const typename TLayoutConfig::Vector3 & I_r_S,  
                                                                     const MeshData<MeshPREC> & mesh,
                                                                     const typename TLayoutConfig::Vector3 & I_r_M,
                                                                     const typename TLayoutConfig::Matrix33 & A_IM,
                                                                     const unsigned int & indexFace,
                                                                     unsigned int & type,unsigned int & id)
   {
      DEFINE_LAYOUT_CONFIG_TYPES_OF_OUTSIDE_TEMPLATE(TLayoutConfig);

      // Assumes everything in the same frame!
      // Search the closest point to the sphere on the triangle A,B,C with vertex0,1,2 -->
      // This code only tests the closest point to the sphere center m_r_S
      // To understand this -> draw a triangle on the paper, and imagina a point in above the paper 
      // somewhere in the space, this point is the center of the sphere...
      static MeshData<MeshPREC>::TMeshIndices indices;
      static typename TLayoutConfig::Vector3 vertex0,vertex1,vertex2;
      indices = mesh.m_Faces[indexFace];
      vertex0 = A_IM * mesh.m_Vertices[indices(0)] + I_r_M;
      vertex1 = A_IM * mesh.m_Vertices[indices(1)] + I_r_M;
      vertex2 = A_IM * mesh.m_Vertices[indices(2)] + I_r_M;

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
   }

};


#endif