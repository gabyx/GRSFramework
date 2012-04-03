/*
*  MeshData.hpp
*
*  Created by Gabriel Nützi on 19.03.10.
*  Copyright 2010 -. All rights reserved.
*
*/

#ifndef MeshInformation_hpp
#define MeshInformation_hpp

/// Includes =================================
#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>
#include <Eigen/Dense>
#include <OGRE/Ogre.h>
#include <OgreMath.h>

#include "TypeDefs.hpp"

///==========================================

template<typename PREC> 
class MeshData
{
public:

   DEFINE_MATRIX_TYPES
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MeshData(){};
   ~MeshData(){};
   
   typedef Eigen::Matrix<unsigned int,3,1> TMeshIndices;

   //Mesh Information in 3D Coordinates in [m] --> scaled!
   std::vector< Vector3 > m_Vertices;
   std::vector< TMeshIndices > m_Faces;
   std::vector< Vector3 > m_Normals; 


   void print(){
      cout << "MeshData:: " <<endl<<"Vertices : "<<endl;
      for (int i=0;i<m_Vertices.size();i++){
         cout <<i<<": "<<m_Vertices[i] <<endl;
      }
      cout << "Indices & Normals : "<<endl;
      for (int i=0;i<m_Faces.size();i++){
         cout <<i<<": "<<m_Faces[i] <<  "\t n:"<<m_Normals[i] <<endl;
      }
   };


   void setup(size_t &vertex_count,
      Ogre::Vector3* vertices,
      size_t &index_count,
      unsigned long* indices, double scale)
   {
      // If we add more Objects we need an Offset for the Indices
      std::size_t indices_offset = m_Vertices.size();

      Vector3 temp;
      for (int i=0;i<vertex_count;i++){
         temp << vertices[i].x/(Ogre::Real)scale, vertices[i].y/(Ogre::Real)scale, vertices[i].z/(Ogre::Real)scale;
         m_Vertices.push_back(temp);
      }

      for (int i=0;i<index_count;i+=3){
         Eigen::Matrix<unsigned long,3,1> temp;
         temp << (unsigned long)(indices[i]+indices_offset), (unsigned long)(indices[i+1]+indices_offset), (unsigned long)(indices[i+2]+indices_offset);
         m_Faces.push_back(temp);
         //Calculate the normal to each triangle:
         //m_Normals.push_back(Ogre::Math::calculateBasicFaceNormal(m_Vertices[indices[i]],m_Vertices[indices[i+1]],m_Vertices[indices[i+2]]));
      }



   };

   void writeToLog(Ogre::Log * plog){
      std::stringstream logmessage;
      logmessage << "MeshData:: " <<endl<<"Vertices : "<<endl;
      for (int i=0;i<m_Vertices.size();i++){
         logmessage <<i<<": "<<m_Vertices[i] <<endl;
      }
      logmessage << "Indices & Normals : "<<endl;
      for (int i=0;i<m_Faces.size();i++){
         logmessage <<i<<": "<<m_Faces[i] <<  "\t n:"<<m_Normals[i] <<endl;
      }
      plog->logMessage(logmessage.str().c_str());
   };

   /* inline Vector3 getPointofTriangle(int IndicesIndex, int PointIndex){
   assert(IndicesIndex < m_Faces.size() && PointIndex <= 2 );
   return m_Vertices[m_Faces[IndicesIndex][PointIndex]];
   }
   */
};


#endif

