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

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>       // Output data structure
#include <assimp/postprocess.h> // Post processing flags

#include "TypeDefs.hpp"

#include "SimpleLogger.hpp"
#include "QuaternionHelpers.hpp"

///==========================================

class MeshData
{
public:

   DEFINE_MATRIX_TYPES_OF( MeshPREC )

   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MeshData(){};
   ~MeshData(){};

   typedef Eigen::Matrix<unsigned int,3,1> MeshIndices;
   typedef std::vector< MeshIndices > Faces;
   typedef std::vector< Vector3 > Vertices;
   typedef std::vector< Vector3 > Normals;

   //Mesh Information in 3D Coordinates in [m] --> scaled!
   Vertices m_Vertices;
   Faces m_Faces;
   Normals m_Normals;


   void print(){
      std::cout << "MeshData:: " <<std::endl<<"Vertices : "<<std::endl;
      for (int i=0;i<m_Vertices.size();i++){
         std::cout <<i<<": "<<m_Vertices[i] <<std::endl;
      }
      std::cout << "Faces (Indices) & Normals : "<<std::endl;
      for (int i=0;i<m_Faces.size();i++){
         std::cout <<i<<": "<<m_Faces[i] <<  "\t n:"<<m_Normals[i] <<std::endl;
      }
   };


//   void setup(size_t &vertex_count,
//      Ogre::Vector3* vertices,
//      size_t &index_count,
//      unsigned long* indices, double scale)
//   {
//      // If we add more Objects we need an Offset for the Indices
//      std::size_t indices_offset = m_Vertices.size();
//
//      Vector3 temp;
//      for (int i=0;i<vertex_count;i++){
//         temp << vertices[i].x/(Ogre::Real)scale, vertices[i].y/(Ogre::Real)scale, vertices[i].z/(Ogre::Real)scale;
//         m_Vertices.push_back(temp);
//      }
//
//      for (int i=0;i<index_count;i+=3){
//         Eigen::Matrix<unsigned long,3,1> temp;
//         temp << (unsigned long)(indices[i]+indices_offset), (unsigned long)(indices[i+1]+indices_offset), (unsigned long)(indices[i+2]+indices_offset);
//         m_Faces.push_back(temp);
//         //Calculate the normal to each triangle:
//         //m_Normals.push_back(Ogre::Math::calculateBasicFaceNormal(m_Vertices[indices[i]],m_Vertices[indices[i+1]],m_Vertices[indices[i+2]]));
//      }
//
//
//
//   };

   bool setup( Assimp::Importer & importer, const aiScene* scene, Vector3 scale_factor, Quaternion quat, Vector3 trans){
    Matrix33 Rot_KI = getRotFromQuaternion(quat);


      if(scene->mNumMeshes >=1){

         for(unsigned int j=0;j<scene->mNumMeshes;j++){
            aiMesh * mesh = scene->mMeshes[j];

            Vector3 temp;
            // Vertices
            for(unsigned int k=0;k<mesh->mNumVertices;k++){

               aiVector3D & vertice = mesh->mVertices[k];
               temp << vertice.x,vertice.y, vertice.z;

               //Apply transformation: Scale, then rotate,then translate all in I frame!
               temp(0) *= scale_factor(0);
               temp(1) *= scale_factor(1);
               temp(2) *= scale_factor(2);
               temp += trans;
               temp = Rot_KI * temp;

               vertice.x = temp(0);
               vertice.y = temp(1);
               vertice.z = temp(2);

               this->m_Vertices.push_back(temp.cast<MeshPREC>());
            }


            // Indices
            Eigen::Matrix<unsigned int,3,1> tempidx;
            for(unsigned int k=0;k<mesh->mNumFaces;k++){
               aiFace & face = mesh->mFaces[k];

               tempidx << face.mIndices[0],face.mIndices[1],face.mIndices[2];
               this->m_Faces.push_back(tempidx);

               // Calculate Normals again!
               Vector3  vertice1 = convertToVector3((mesh->mVertices[face.mIndices[0]]));
               Vector3  vertice2 = convertToVector3((mesh->mVertices[face.mIndices[1]]));
               Vector3  vertice3 = convertToVector3((mesh->mVertices[face.mIndices[2]]));

               Vector3 p1 = vertice2-vertice1;
               Vector3 p2 = vertice3-vertice1;

               Vector3 n= p1.cross(p2);
               n.normalize();
               if(n.norm()==0){
                  n(0) = 1; n(1)=0; n(2)=0;
               }
               this->m_Normals.push_back(n.cast<MeshPREC>());


            }
            /*
            if(mesh->HasNormals()){
            for(int k=0;k<mesh->mNumVertices;k++){
            aiVector3D & normal = mesh->mNormals[k];
            temp << normal.x,normal.y,normal.z;
            this->m_Normals.push_back(temp);
            }
            }*/
         }

      }else{
         return false;
      }

    return true;
   };

   //Helper Function
   Vector3 convertToVector3(const aiVector3D & a){
      Vector3 ret;
      ret << a.x, a.y, a.z;
      return ret;
   }


   void writeToLog(std::string name, Logging::Log * plog){
      std::stringstream logmessage;

      using std::endl;
      using std::cout;

      logmessage << "---> MeshData:: "<<name<<std::endl<<"\t Vertices : "<<std::endl;
      for (int i=0;i<m_Vertices.size();i++){
         logmessage <<"\t\t Vertex "<< i<<": "<<m_Vertices[i].transpose() <<endl;
      }
      logmessage << " \t Faces (Indices) & Normals : "<<endl;
      for (int i=0;i<m_Faces.size();i++){
         logmessage <<"\t\t Face "<<i<<": "<<m_Faces[i].transpose() <<  "\t n:"<<m_Normals[i].transpose() <<std::endl;
      }
      plog->logMessage(logmessage.str());
   };

   /* inline Vector3 getPointofTriangle(int IndicesIndex, int PointIndex){
   assert(IndicesIndex < m_Faces.size() && PointIndex <= 2 );
   return m_Vertices[m_Faces[IndicesIndex][PointIndex]];
   }
   */
};


#endif

