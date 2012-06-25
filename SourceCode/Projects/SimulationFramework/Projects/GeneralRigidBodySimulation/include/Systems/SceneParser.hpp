#ifndef SceneParser_hpp
#define SceneParser_hpp

#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "AssertionDebug.hpp"


#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/filesystem.hpp>
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include "Ogre.h"

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "DynamicsSystem.hpp"

#include "CommonFunctions.hpp"
#include "QuaternionHelpers.hpp"
#include "InertiaTensorCalculations.hpp"
#include "InitialConditionBodies.hpp"

#include <assimp.hpp>      // C++ importer interface
#include <aiScene.h>       // Output data structure
#include <aiPostProcess.h> // Post processing flags

//#include <Opcode.h>

//#include "OgreMeshExtraction.hpp"

#define TIXML_USE_TICPP
#include "ticpp.h"
//#include "tinyxml.h"

/*
* @Does not work yet, to implement a scene parser, implement everything starting from SimulationState, enter(), we exit the State, delete all Objects, and reinitialize with another system in XML format.
*
*/
template<typename TConfig>
class SceneParser{
public:

   DEFINE_CONFIG_TYPES_OF(TConfig)

      SceneParser(
      Ogre::SceneNode * baseFrame,
      boost::shared_ptr<Ogre::SceneManager> pSceneMgr,
      std::vector<Ogre::SceneNode*> &nodesSimBodies,
      std::vector<Ogre::SceneNode*> &nodesBodies,
      boost::shared_ptr<TSystem> pDynSys
      )
      : m_pSceneMgr(pSceneMgr),  m_rSceneNodeSimBodies(nodesSimBodies), m_rSceneNodeBodies(nodesBodies) , m_pDynSys(pDynSys)
   {
      ASSERTMSG(baseFrame != NULL, "Pointer is NULL");
      m_BaseFrame = baseFrame;
      m_pAppLog = RenderContext::getSingletonPtr()->m_pAppLog;
      m_bParseDynamics = true;
      m_SimBodies = 0;
   }

   SceneParser(
      Ogre::SceneNode * baseFrame,
      boost::shared_ptr<Ogre::SceneManager> pSceneMgr,
      std::vector<Ogre::SceneNode*> &nodesSimBodies,
      std::vector<Ogre::SceneNode*> &nodesBodies
      )
      : m_pSceneMgr(pSceneMgr),  m_rSceneNodeSimBodies(nodesSimBodies), m_rSceneNodeBodies(nodesBodies)
   {
      ASSERTMSG(baseFrame != NULL, "Pointer is NULL");
      m_BaseFrame = baseFrame;
      m_pAppLog = RenderContext::getSingletonPtr()->m_pAppLog;
      m_bParseDynamics = false;
      m_SimBodies = 0;
   }


   bool parseScene( boost::filesystem::path file )
   {

      m_currentParseFilePath = file;
      m_currentParseFileDir = m_currentParseFilePath.parent_path();

      m_pAppLog->logMessage("Parsing Scene...");

      CLEARLOG;
      logstream <<"Scene Input file: "  << file.string() <<endl;
      LOG(m_pAppLog);


      //Reset all variables
      m_SimBodies = 0;
      m_bodyList.clear();
      m_SimBodyInitStates.clear();
      m_SceneMeshs.clear();

      try{
         m_xmlDoc.LoadFile(m_currentParseFilePath.string());

         m_pAppLog->logMessage("File successfully loaded ...");

         m_pAppLog->logMessage("Try to parse the scene ...");

         // Start off with the gravity!
         m_xmlRootNode = m_xmlDoc.FirstChild("DynamicsSystem");
         if(m_xmlRootNode){
            ticpp::Node *node = NULL;

            node = m_xmlRootNode->FirstChild("SceneSettings");
            processSceneSettings(node);

            node = m_xmlRootNode->FirstChild("SceneObjects");
            processSceneObjects(node);

            /*ticpp::Node * initialConditionAll = m_xmlRootNode->FirstChild("InitialCondition");
            processinitialConditionAll(initialConditionAll);*/

         }else{
            m_pAppLog->logMessage("No DynamicsSystem Node found in XML ...");
            return false;
         }

      }
      catch(ticpp::Exception& ex){
         CLEARLOG;
         logstream <<"Scene XML error: "  << ex.what() <<endl;
         LOG(m_pAppLog);
         exit(-1);
      }


      //ASSERTMSG(false,"XML parsing...");

      return true;
   }

   boost::filesystem::path getCurrentSceneFilePath(){
      return m_currentParseFilePath;
   }

   const std::vector< DynamicsState<TLayoutConfig> > & getInitialConditionSimBodies(){
      return m_SimBodyInitStates;
   }

   unsigned int getNumberOfSimBodies(){ return m_SimBodies;}


private:

   void processSceneSettings( ticpp::Node *sceneSettings ){
      if(m_bParseDynamics){

         ticpp::Element *gravityElement = sceneSettings->FirstChild("Gravity",true)->ToElement();
         m_pDynSys->m_gravity = gravityElement->GetAttribute<double>("value");

         if(!stringToVector3<PREC>(m_pDynSys->m_gravityDir , gravityElement->GetAttribute("direction"))){
            throw ticpp::Exception("String conversion in SceneSettings: gravity failed");
         }

         ticpp::Element *timestepElement = sceneSettings->FirstChild("TimeStepperSettings",true)->ToElement();

            TimeStepperSettings<TLayoutConfig> timestepperSettings;
            InclusionSolverSettings<TLayoutConfig> inclusionSettings;

            // Get standart values!
            m_pDynSys->getSettings(timestepperSettings,inclusionSettings);

            if(!stringToType<PREC>(timestepperSettings.m_deltaT, timestepElement->GetAttribute("deltaT"))){
                  throw ticpp::Exception("String conversion in SceneSettings: deltaT failed");
            }
            inclusionSettings.m_deltaT = timestepperSettings.m_deltaT;

            if(!stringToType<PREC>(timestepperSettings.m_endTime, timestepElement->GetAttribute("endTime"))){
                  throw ticpp::Exception("String conversion in SceneSettings: endTime failed");
            }

           ticpp::Element *simFromRef = timestepElement->FirstChild("SimulateFromReference",false)->ToElement();
           if(simFromRef){

              bool enabled = false;
              if(!stringToType<bool>(enabled, simFromRef->GetAttribute("enabled"))){
                  throw ticpp::Exception("String conversion in SimulateFromReference: enable failed");
              }
              if(enabled){
                 std::string type = simFromRef->GetAttribute("type");
                 if(type == "useStates"){
                     timestepperSettings.m_eSimulateFromReference = TimeStepperSettings<TLayoutConfig>::USE_STATES;
                 }else if(type == "continue"){
                     timestepperSettings.m_eSimulateFromReference = TimeStepperSettings<TLayoutConfig>::CONTINUE;
                 }else{
                    throw ticpp::Exception("String conversion in SimulateFromReference: type. The type '" + type + std::string("' has no implementation in the parser"));
                 }
                 timestepperSettings.m_stateReferenceFile = simFromRef->GetAttribute("file");
                 checkFileExists(timestepperSettings.m_stateReferenceFile);
              }else{
                  timestepperSettings.m_eSimulateFromReference = TimeStepperSettings<TLayoutConfig>::NONE;
              }
           }


           ticpp::Element *inclusionElement = timestepElement->FirstChild("InclusionSolverSettings",false)->ToElement();
           if(inclusionElement){

               if(!stringToType<PREC>(inclusionSettings.m_alphaJORProx, inclusionElement->GetAttribute("alphaJORProx"))){
                     throw ticpp::Exception("String conversion in SceneSettings: alphaJORProx failed");
               }
               if(!stringToType<PREC>(inclusionSettings.m_alphaSORProx, inclusionElement->GetAttribute("alphaSORProx"))){
                     throw ticpp::Exception("String conversion in SceneSettings: alphaJORProx failed");
               }
               if(!stringToType<unsigned int>(inclusionSettings.m_MaxIter, inclusionElement->GetAttribute("maxIter"))){
                     throw ticpp::Exception("String conversion in SceneSettings: maxIter failed");
               }

               if(!stringToType<PREC>(inclusionSettings.m_AbsTol, inclusionElement->GetAttribute("absTol"))){
                     throw ticpp::Exception("String conversion in SceneSettings: absTol failed");
               }
               if(!stringToType<PREC>(inclusionSettings.m_RelTol, inclusionElement->GetAttribute("relTol"))){
                     throw ticpp::Exception("String conversion in SceneSettings: relTol failed");
               }

               if(inclusionElement->HasAttribute("isFiniteCheck")){
                  if(!stringToType<bool>(inclusionSettings.m_bIsFiniteCheck, inclusionElement->GetAttribute("isFiniteCheck"))){
                     throw ticpp::Exception("String conversion in SceneSettings: isFiniteCheck failed");
                  }
               }

               std::string method = inclusionElement->GetAttribute("method");
               if(method == "JOR"){
                  inclusionSettings.m_eMethod = InclusionSolverSettings<TLayoutConfig>::JOR;
               }
               else if (method == "SOR"){
                  inclusionSettings.m_eMethod = InclusionSolverSettings<TLayoutConfig>::SOR;
               }
               else{
                     throw ticpp::Exception("String conversion in SceneSettings: relTol failed");
               }


               if(!stringToType<bool>(inclusionSettings.m_bUseGPU, inclusionElement->GetAttribute("useGPU"))){
                     throw ticpp::Exception("String conversion in SceneSettings: useGPU failed");
               }

               if(inclusionElement->HasAttribute("useGPUID")){
                  if(!stringToType<int>(inclusionSettings.m_UseGPUDeviceId, inclusionElement->GetAttribute("useGPUID"))){
                        throw ticpp::Exception("String conversion in SceneSettings: useGPU failed");
                  }
                  if(inclusionSettings.m_UseGPUDeviceId <0){
                      inclusionSettings.m_UseGPUDeviceId = 0;
                  }
               }

           }
            //Write all values back
           m_pDynSys->setSettings(timestepperSettings,inclusionSettings);



      }

   }


   void processSceneObjects( ticpp::Node *sceneObjects ){

      ticpp::Iterator< ticpp::Node > child;

      for ( child = child.begin( sceneObjects ); child != child.end(); child++ ){

         if( child->Value() == "RigidBodies"){
            processRigidBodies( &(*child) );
         }

      }

      if( m_SimBodies == 0){
         throw ticpp::Exception("The scene in the XML contains no simulating bodies!");
      }

   }

   void processRigidBodies( ticpp::Node * rigidbodies ){

      //Clear current body list;
      m_bodyList.clear();
      m_bodyListScales.clear();

      int instances = rigidbodies->ToElement()->GetAttribute<int>("instances");


      for(int i=0; i<instances;i++){
         boost::shared_ptr< RigidBody<TLayoutConfig> > temp_ptr(new RigidBody<TLayoutConfig>());
         temp_ptr->m_id = i;
         m_bodyList.push_back(temp_ptr);

         Vector3 scale; scale.setOnes();
         m_bodyListScales.push_back(scale);
      }


      ticpp::Node * geometryNode = rigidbodies->FirstChild("Geometry");
      processGeometry(geometryNode);


      ticpp::Node * dynPropNode = rigidbodies->FirstChild("DynamicProperties");
      processDynamicProperties(dynPropNode);


         //Copy the pointers!
         if(m_eBodiesState == RigidBody<TLayoutConfig>::SIMULATED){
            if(m_bParseDynamics){
               for(int i=0; i < m_bodyList.size(); i++){
                  m_pDynSys->m_SimBodies.push_back(m_bodyList[i]);
               }
            }
            m_SimBodies += instances;
         }
         else if(m_eBodiesState == RigidBody<TLayoutConfig>::NOT_SIMULATED){
            if(m_bParseDynamics){
               for(int i=0; i < m_bodyList.size(); i++){
                  m_pDynSys->m_Bodies.push_back(m_bodyList[i]);
               }
            }
         }
         else{
            throw ticpp::Exception("Adding only simulated and not simulated objects supported!");
         }





      ticpp::Node * visualizationNode = rigidbodies->FirstChild("Visualization");
      processVisualization( visualizationNode);





   }

   void processGeometry( ticpp::Node * geometryNode){
      if(geometryNode->FirstChild()->Value() == "Sphere"){

         processSphereGeometry( geometryNode->FirstChild()->ToElement());
      }
      else if(geometryNode->FirstChild()->Value() == "Halfspace"){

         processHalfspaceGeometry( geometryNode->FirstChild()->ToElement());
      }else if(geometryNode->FirstChild()->Value() == "Mesh"){

         processMeshGeometry( geometryNode->FirstChild()->ToElement());

      }else{
         throw ticpp::Exception("The geometry '" + std::string(geometryNode->FirstChild()->Value()) + std::string("' has no implementation in the parser"));
      }
   }

   void processSphereGeometry( ticpp::Element * sphere){
      std::string type = sphere->GetAttribute("distribute");
      if(type == "uniform"){

         double radius = sphere->GetAttribute<double>("radius");
         Vector3 scale; scale(0)=radius; scale(1)=radius; scale(2)=radius;

         boost::shared_ptr<SphereGeometry<PREC> > pSphereGeom = boost::shared_ptr<SphereGeometry<PREC> >(new SphereGeometry<PREC>(radius));

         for(int i=0; i < m_bodyList.size(); i++){
            m_bodyListScales[i] = scale;
            m_bodyList[i]->m_geometry = pSphereGeom;
         }
      }
      else if(type == "random"){
         double minRadius;
         if(!stringToType<double>(minRadius,sphere->GetAttribute("minRadius"))){
            throw ticpp::Exception("String conversion in processSphereGeometry: minRadius failed");
         }
         if( minRadius <= 0){
            throw ticpp::Exception("In processSphereGeometry: minRadius to small!");
         }

         double maxRadius;
         if(!stringToType<double>(maxRadius,sphere->GetAttribute("maxRadius"))){
            throw ticpp::Exception("String conversion in processSphereGeometry: minRadius failed");
         }
         if( maxRadius <= minRadius){
            throw ticpp::Exception("In processSphereGeometry: maxRadius smaller or equal to minRadius!");
         }

         unsigned int seed;
         if(!stringToType<unsigned int>(seed,sphere->GetAttribute("seed"))){
            throw ticpp::Exception("String conversion in processSphereGeometry: seed failed");
         }

         typedef boost::mt19937  RNG;
         RNG generator(seed);
         boost::uniform_real<PREC> uniform(minRadius,maxRadius);
         boost::variate_generator< boost::mt19937 & , boost::uniform_real<PREC> > randomNumber(generator, uniform);

         for(int i=0; i < m_bodyList.size(); i++){
            double radius = randomNumber();
            Vector3 scale; scale(0)=radius; scale(1)=radius; scale(2)=radius;
            m_bodyListScales[i] = scale;
            boost::shared_ptr<SphereGeometry<PREC> > pSphereGeom = boost::shared_ptr<SphereGeometry<PREC> >(new SphereGeometry<PREC>(radius));
            m_bodyList[i]->m_geometry = pSphereGeom;
         }
      }
      else{
         throw ticpp::Exception("The attribute 'distribute' '" + type + std::string("' of 'Sphere' has no implementation in the parser"));
      }
   }

   void processHalfspaceGeometry( ticpp::Element * halfspace){
      std::string type = halfspace->GetAttribute("distribute");
      if(type == "uniform"){

         Vector3 n;
         if(!stringToVector3<PREC>(n, halfspace->GetAttribute("normal"))){
            throw ticpp::Exception("String conversion in HalfsphereGeometry: normal failed");
         }

         Vector3 p;
         if(!stringToVector3<PREC>(p, halfspace->GetAttribute("position"))){
            throw ticpp::Exception("String conversion in HalfsphereGeometry: position failed");
         }

         boost::shared_ptr<HalfspaceGeometry<PREC> > pHalfspaceGeom = boost::shared_ptr<HalfspaceGeometry<PREC> >(new HalfspaceGeometry<PREC>(n,p));

         for(int i=0; i < m_bodyList.size(); i++){
            m_bodyList[i]->m_geometry = pHalfspaceGeom;
         }

      }else{
         throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'Sphere' has no implementation in the parser"));
      }
   }


   void processMeshGeometry( ticpp::Element * mesh){

      boost::shared_ptr<MeshGeometry<PREC> > pMeshGeom;

      std::string meshName = mesh->GetAttribute<std::string>("name");

      bool bInstantiate;
      if(!stringToType<bool>(bInstantiate,mesh->GetAttribute("useInstance"))){
         throw ticpp::Exception("String conversion in processMeshGeometry: useInstance failed");
      }

      std::string type = mesh->GetAttribute("distribute");
      if(type == "uniform"){

         // import here an object model;
         // First make an meshinformatino structure

         boost::filesystem::path fileName =  mesh->GetAttribute<std::string>("file");
         checkFileExists(fileName);
         // if this file has already been added--> skip it and reference the
         typename ContainerSceneMeshs::iterator it = m_SceneMeshs.find(meshName);

         if( bInstantiate == false){

            if(it != m_SceneMeshs.end()){
               throw ticpp::Exception("Already defined mesh with name:" + meshName);
            }

            Vector3 scale_factor;
            if(!stringToVector3<PREC>(scale_factor, mesh->GetAttribute("scale"))){
               throw ticpp::Exception("String conversion in processMeshGeometry failed: scale");
            }
            if(scale_factor.norm()==0){
               throw ticpp::Exception("Wrong scale factor (=0) specified in processMeshGeometry!");
            }

            Vector3 trans;
            if(!stringToVector3<PREC>(trans, mesh->GetAttribute("translation"))){
               throw ticpp::Exception("String conversion in processMeshGeometry: translation failed: ");
            }

            Vector3 axis;
            if(!stringToVector3<PREC>(axis, mesh->GetAttribute("rotationAxis"))){
               throw ticpp::Exception("String conversion in processMeshGeometry: rotationAxis failed");
            }

            PREC angle;

            if(mesh->HasAttribute("angleDegree")){
               if(!stringToType<PREC>(angle, mesh->GetAttribute("angleDegree"))){
                  throw ticpp::Exception("String conversion in processMeshGeometry: angleDegree failed");
               }
               angle = angle / 180 * M_PI;
            }else if(mesh->HasAttribute("angleRadian")){
               if(!stringToType<PREC>(angle, mesh->GetAttribute("angleRadian"))){
                  throw ticpp::Exception("String conversion in processMeshGeometry: angleRadian  failed");
               }
            }else{
               throw ticpp::Exception("No angle found in processMeshGeometry");
            }

            Quaternion quat;
            setQuaternion(quat,axis,angle);


            Assimp::Importer importer;


            importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_LINE | aiPrimitiveType_POINT);
            importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_NORMALS | aiComponent_MESHES);
            // And have it read the given file with some example postprocessing
            // Usually - if speed is not the most important aspect for you - you'll
            // propably to request more postprocessing than we do in this example.
            const aiScene* scene = importer.ReadFile( fileName.string(),
               aiProcess_JoinIdenticalVertices  |
               aiProcess_SortByPType |
               aiProcess_Triangulate | aiProcess_GenNormals);

            // If the import failed, report it
            if(!scene)
            {
               throw ticpp::Exception("File import failed in processMeshGeometry: for file" + fileName.string() );
            }

            boost::shared_ptr<MeshData<MeshPREC> > meshData = boost::shared_ptr<MeshData<MeshPREC> >(new MeshData<MeshPREC>);

            fillMeshInfo(importer,scene, *meshData, scale_factor,quat,trans);

            // Build Geometry
            pMeshGeom = boost::shared_ptr<MeshGeometry<PREC> >(new MeshGeometry<PREC>(meshData));

            // Add geometry into the cache
            m_SceneMeshs.insert(typename ContainerSceneMeshs::value_type(meshName,pMeshGeom));

         }
         else{
            if(it == m_SceneMeshs.end()){
               throw ticpp::Exception("Mesh with name: " + meshName + " not found!");
            }
            // MeshData exists already for this file fileName
            pMeshGeom = it->second;
         }


         // Assign Geometry
         for(int i=0; i < m_bodyList.size(); i++){
            m_bodyList[i]->m_geometry = pMeshGeom;
         }

      }else{
         throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'Mesh' has no implementation in the parser"));
      }
   }

   void fillMeshInfo( Assimp::Importer & importer, const aiScene* scene, MeshData<MeshPREC> & meshInfo, Vector3 scale_factor, Quaternion quat, Vector3 trans){

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

               meshInfo.m_Vertices.push_back(temp.template cast<MeshPREC>());
            }


            // Indices
            Eigen::Matrix<unsigned int,3,1> tempidx;
            for(unsigned int k=0;k<mesh->mNumFaces;k++){
               aiFace & face = mesh->mFaces[k];

               tempidx << face.mIndices[0],face.mIndices[1],face.mIndices[2];
               meshInfo.m_Faces.push_back(tempidx);

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
               meshInfo.m_Normals.push_back(n.template cast<MeshPREC>());


            }
            /*
            if(mesh->HasNormals()){
            for(int k=0;k<mesh->mNumVertices;k++){
            aiVector3D & normal = mesh->mNormals[k];
            temp << normal.x,normal.y,normal.z;
            meshInfo.m_Normals.push_back(temp);
            }
            }*/
         }

      }else{
         throw ticpp::Exception("File import failed in fillMeshInfo");
      }

   }

   void checkFileExists(boost::filesystem::path file){
      if( !boost::filesystem::exists(file) ){
         throw ticpp::Exception("The file ' " + file.string() + "' does not exist!");
      }
   }

   void processDynamicProperties( ticpp::Node * dynProp){

      ticpp::Element * element = dynProp->FirstChild("DynamicState")->ToElement();

      std::string type = element->GetAttribute("type");
      if(type == "simulated"){
         m_eBodiesState =  RigidBody<TLayoutConfig>::SIMULATED;
         processDynamicPropertiesSimulated(dynProp);
      }
      else if(type == "not simulated"){
         m_eBodiesState =  RigidBody<TLayoutConfig>::NOT_SIMULATED;
         processDynamicPropertiesNotSimulated(dynProp);
      }
      else if(type == "animated"){
         m_eBodiesState =  RigidBody<TLayoutConfig>::ANIMATED;
         throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
      }
      else{
         throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
      }

      for(int i=0; i < m_bodyList.size(); i++){
         m_bodyList[i]->m_eState = m_eBodiesState;
      }

   }


   void processDynamicPropertiesSimulated( ticpp::Node * dynProp){


      // Mass ============================================================
      ticpp::Element *element = dynProp->FirstChild("Mass")->ToElement();
      std::string distribute = element->GetAttribute("distribute");
      if(distribute == "uniform"){

         double mass = element->GetAttribute<double>("value");

         for(int i=0; i < m_bodyList.size(); i++){
            m_bodyList[i]->m_mass = mass;
         }


      }else{
         throw ticpp::Exception("The attribute 'distribute' '" + distribute + std::string("' of 'Mass' has no implementation in the parser"));
      }

      // InertiaTensor ============================================================
      element = dynProp->FirstChild("InertiaTensor")->ToElement();
      std::string type = element->GetAttribute("type");
      if(type == "homogen"){
         for(int i=0; i < m_bodyList.size(); i++){
            InertiaTensor::calculateInertiaTensor(m_bodyList[i]);
         }
      }
      else{
         throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'InertiaTensor' has no implementation in the parser"));
      }

      element = dynProp->FirstChild("Material")->ToElement();
      distribute = element->GetAttribute("distribute");
      if(distribute == "uniform"){
         typename RigidBody<TLayoutConfig>::BodyMaterial eMaterial;
         type = element->GetAttribute("type");
         if(type == "standart"){
            eMaterial = RigidBody<TLayoutConfig>::STD_MATERIAL;
         }
         else if( type == "wood" ){
            eMaterial = RigidBody<TLayoutConfig>::WOOD;
         }
         else if( type == "metal"){
            eMaterial = RigidBody<TLayoutConfig>::METAL;
         }
         else if( type == "glas"){
            eMaterial = RigidBody<TLayoutConfig>::GLAS;
         }else{
            eMaterial = RigidBody<TLayoutConfig>::STD_MATERIAL;
         }
         for(int i=0; i < m_bodyList.size(); i++){
            m_bodyList[i]->m_eMaterial = eMaterial;
         }
      }
      else{
         throw ticpp::Exception("The attribute 'distribute' '" + distribute + std::string("' of 'Material' has no implementation in the parser"));
      }

      // InitialCondition ============================================================
      element = dynProp->FirstChild("InitialCondition")->ToElement();
      distribute = element->GetAttribute("distribute");
      m_SimBodyInitStates.push_back(DynamicsState<TLayoutConfig>((unsigned int)m_bodyList.size()));
      if(distribute == "linear"){
         processInitialConditionLinear(m_SimBodyInitStates.back(),element);
      }
      else if(distribute == "grid"){
         processInitialConditionGrid(m_SimBodyInitStates.back(),element);
      }
      else if(distribute == "posaxisangle"){
         processInitialConditionPositionAxisAngle(m_SimBodyInitStates.back(),element);
      }
      else if(distribute == "transform"){
         processInitialConditionTransforms(m_SimBodyInitStates.back(),element);
      }
      else if(distribute == "none"){
         // does nothing leaves the zero state pushed!
      }else{
         throw ticpp::Exception("The attribute 'distribute' '" + distribute + std::string("' of 'InitialCondition' has no implementation in the parser"));
      }

      InitialConditionBodies::applyDynamicsStateToBodies(m_SimBodyInitStates.back(), m_bodyList);

   }


   void processDynamicPropertiesNotSimulated( ticpp::Node * dynProp){

      // InitialCondition ============================================================
      ticpp::Element *element = dynProp->FirstChild("InitialCondition")->ToElement();
      std::string distribute = element->GetAttribute("distribute");

      DynamicsState<TLayoutConfig> state((unsigned int)m_bodyList.size());
      if(distribute == "linear"){
         processInitialConditionLinear(state,element);
      }
      else if(distribute == "grid"){
         processInitialConditionGrid(state,element);
      }
      else if(distribute == "file"){
         processInitialConditionFile(state,element);
      }
      else if(distribute == "posaxisangle"){
         processInitialConditionPositionAxisAngle(state,element);
      }
      else if(distribute == "transforms"){
         processInitialConditionTransforms(state,element);
      }
      else if(distribute == "none"){
         m_SimBodyInitStates.push_back(DynamicsState<TLayoutConfig>((unsigned int)m_bodyList.size())); // Adds a zero DynamicState
      }else{
         throw ticpp::Exception("The attribute 'distribute' '" + distribute + std::string("' of 'InitialCondition' has no implementation in the parser"));
      }

      InitialConditionBodies::applyDynamicsStateToBodies(state, m_bodyList);

   }

   template<typename TLayoutConfig>
   void processInitialConditionLinear(DynamicsState<TLayoutConfig> & state, ticpp::Element * initCond){

      Vector3 pos;
      if(!stringToVector3<PREC>(pos, initCond->GetAttribute("position"))){
         throw ticpp::Exception("String conversion in InitialCondition: position Linear failed");
      }
      Vector3 dir;
      if(!stringToVector3<PREC>(dir, initCond->GetAttribute("direction"))){
         throw ticpp::Exception("String conversion in InitialCondition: direction Linear failed");
      }
      PREC dist;
      if(!stringToType<PREC>(dist, initCond->GetAttribute("distance"))){
         throw ticpp::Exception("String conversion in InitialCondition: distance  Linear failed");
      }
      bool jitter;
      if(!stringToType(jitter, initCond->GetAttribute("jitter"))){
         throw ticpp::Exception("String conversion in InitialCondition: jitter Linear failed");
      }
      PREC delta;
      if(!stringToType<PREC>(delta, initCond->GetAttribute("delta"))){
         throw ticpp::Exception("String conversion in InitialCondition: delta Linear failed");
      }

      InitialConditionBodies::setupBodiesLinear<TLayoutConfig>(state,pos,dir,dist,jitter,delta);

   }
   template<typename TLayoutConfig>
   void processInitialConditionGrid(DynamicsState<TLayoutConfig> & state, ticpp::Element * initCond){

      Vector3 trans;
      if(!stringToVector3<PREC>(trans, initCond->GetAttribute("translation"))){
         throw ticpp::Exception("String conversion in InitialConditionGrid: translation failed");
      }
      int gridX;
      if(!stringToType<int>(gridX, initCond->GetAttribute("gridSizeX"))){
         throw ticpp::Exception("String conversion in InitialConditionGrid: gridSizeX failed");
      }
      int gridY;
      if(!stringToType<int>(gridY, initCond->GetAttribute("gridSizeY"))){
         throw ticpp::Exception("String conversion in InitialConditionGrid: gridSizeY failed");
      }
      PREC dist;
      if(!stringToType<PREC>(dist, initCond->GetAttribute("distance"))){
         throw ticpp::Exception("String conversion in InitialConditionGrid: distance failed");
      }
      bool jitter;
      if(!stringToType(jitter, initCond->GetAttribute("jitter"))){
         throw ticpp::Exception("String conversion in InitialConditionGrid: jitter failed");
      }
      double delta;
      if(!stringToType<double>(delta, initCond->GetAttribute("delta"))){
         throw ticpp::Exception("String conversion in InitialConditionGrid: delta failed");
      }

      InitialConditionBodies::setupBodiesGrid(state,gridX,gridY,dist,trans,jitter,delta);
   }
   template<typename  TLayoutConfig>
   void processInitialConditionFile(DynamicsState<TLayoutConfig> & state, ticpp::Element * initCond){
      m_SimBodyInitStates.push_back(DynamicsState<TLayoutConfig>((unsigned int)m_bodyList.size()));

      boost::filesystem::path name =  initCond->GetAttribute<std::string>("name");

      boost::filesystem::path filePath = m_currentParseFileDir / name;
      InitialConditionBodies::setupBodiesFromFile(state,filePath);
   }
   template<typename  TLayoutConfig>
   void processInitialConditionPositionAxisAngle(DynamicsState<TLayoutConfig> & state, ticpp::Element * initCond){

      int bodyCounter = 0;
      // Iterate over all values in the list
      ticpp::Iterator< ticpp::Element > valueElem("Value");
      for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++){

         Vector3 pos;
         if(!stringToVector3<PREC>(pos, valueElem->GetAttribute("position"))){
            throw ticpp::Exception("String conversion in InitialConditionPositionAxisAngle: position failed");
         }
         Vector3 axis;
         if(!stringToVector3<PREC>(axis, valueElem->GetAttribute("axis"))){
            throw ticpp::Exception("String conversion in InitialConditionPositionAxisAngle: axis failed");
         }

         if( axis.norm() == 0){
            throw ticpp::Exception("Specified wrong axis in InitialConditionPositionAxisAngle");
         }

         PREC angle;

         if(valueElem->HasAttribute("angleDegree")){
            if(!stringToType<PREC>(angle, valueElem->GetAttribute("angleDegree"))){
               throw ticpp::Exception("String conversion in InitialConditionPositionAxisAngle: angleDegree failed");
            }
            angle = angle / 180 * M_PI;
         }else if(valueElem->HasAttribute("angleRadian")){
            if(!stringToType<PREC>(angle, valueElem->GetAttribute("angleRadian"))){
               throw ticpp::Exception("String conversion in InitialConditionPositionAxisAngle: angleRadian failed");
            }
         }else{
            throw ticpp::Exception("No angle found in InitialConditionPositionAxisAngle");
         }

         if(bodyCounter >= state.m_SimBodyStates.size()){
            throw ticpp::Exception("To many intial condition specified!");
         }

         InitialConditionBodies::setupBodyPositionAxisAngle(state.m_SimBodyStates[bodyCounter], pos, axis, angle);

         bodyCounter++;
      }
   }

    template<typename  TLayoutConfig>
   void processInitialConditionTransforms(DynamicsState<TLayoutConfig> & state, ticpp::Element * initCond){



      int bodyCounter = 0;
      // Iterate over all values in the list
      ticpp::Iterator< ticpp::Element > valueElem("Value");
      for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++){


            Quaternion q_KI, q_BK;
            setQuaternionZero(q_KI);
            Vector3 I_r_IK, K_r_KB;
            I_r_IK.setZero();
            Matrix33 Rot_KI; // Temp

          // Iterate over all transforms an successfully applying the total trasnformation!
          ticpp::Iterator< ticpp::Element > transformElem("Transform");
          for ( transformElem = transformElem.begin( &(*valueElem) ); transformElem != transformElem.end(); transformElem++){

               Vector3 trans;
               if(!stringToVector3<PREC>(trans, transformElem->GetAttribute("translation"))){
                  throw ticpp::Exception("String conversion in InitialConditionTransforms: translation failed");
               }
               Vector3 axis;
               if(!stringToVector3<PREC>(axis, transformElem->GetAttribute("rotationAxis"))){
                  throw ticpp::Exception("String conversion in InitialConditionTransforms: rotationAxis failed");
               }

               if( axis.norm() == 0){
                  throw ticpp::Exception("Specified wrong axis in InitialConditionTransforms");
               }

               PREC angle;
               if(transformElem->HasAttribute("angleDegree")){
                  if(!stringToType<PREC>(angle, transformElem->GetAttribute("angleDegree"))){
                     throw ticpp::Exception("String conversion in InitialConditionTransforms: angleDegree failed");
                  }
                  angle = angle / 180 * M_PI;
               }else if(transformElem->HasAttribute("angleRadian")){
                  if(!stringToType<PREC>(angle, transformElem->GetAttribute("angleRadian"))){
                     throw ticpp::Exception("String conversion in InitialConditionTransforms: angleRadian failed");
                  }
               }else{
                  throw ticpp::Exception("No angle found in InitialConditionTransforms");
               }

               setQuaternion(q_BK,axis,angle);
               K_r_KB = trans;
               Rot_KI = getRotFromQuaternion(q_KI);
               I_r_IK += Rot_KI * K_r_KB; // Transforms like A_IK * A_r_AB;
               q_KI = quatMult(q_KI,q_BK); // Sequential (aktiv) rotation

          }

          if(bodyCounter >= state.m_SimBodyStates.size()){
              throw ticpp::Exception("To many intial condition specified!");
          }
          // Apply overall transformation!
          state.m_SimBodyStates[bodyCounter].m_q.template head<3>() = I_r_IK;
          state.m_SimBodyStates[bodyCounter].m_q.template tail<4>() = q_KI;

         bodyCounter++;
      }

      if(bodyCounter != state.m_SimBodyStates.size()){
          throw ticpp::Exception("To many little intial condition specified!");
      }

   }



   void processVisualization( ticpp::Node * visualizationNode){

      ticpp::Node * meshNode = visualizationNode->FirstChild("Mesh");

      processMesh(meshNode);

   }

   void processMesh( ticpp::Node * meshNode ){

      static int nodeCounter = 0;
      static int entityCounter = 0;

      boost::filesystem::path meshName = meshNode->ToElement()->GetAttribute<std::string>("file");

      bool scaleLikeGeometry = false;
      Vector3 scale;
      if(meshNode->ToElement()->HasAttribute("scaleLikeGeometry")){
         if(!stringToType<bool>(scaleLikeGeometry, meshNode->ToElement()->GetAttribute("scaleLikeGeometry"))){
            throw ticpp::Exception("String conversion of scale in processMesh: scaleWithGeometry failed");
         }
      }else{

         if(!stringToVector3<PREC>(scale, meshNode->ToElement()->GetAttribute("scale"))){
            throw ticpp::Exception("String conversion of scale in processMesh: scale failed");
         }
      }


      ticpp::Element * rendering =  meshNode->FirstChildElement("Rendering", false);
      bool attachAxis = false;
      double axesSize = 1;
      bool shadowsEnabled = true;
      if(rendering){
          bool attachAxis = false;
         if(rendering->HasAttribute("attachAxis")){
            if(!stringToType<bool>(attachAxis, rendering->GetAttribute("attachAxis"))){
               throw ticpp::Exception("String conversion of in processMesh: attachAxis failed");
            }
         }

         double axesSize = 1;
         if(rendering->HasAttribute("axesSize")){
            if(!stringToType<double>(axesSize, rendering->GetAttribute("axesSize"))){
               throw ticpp::Exception("String conversion of in processMesh: axesSize failed");
            }
         }


         if(rendering->HasAttribute("shadowsEnabled")){
              if(!stringToType<bool>(shadowsEnabled, rendering->GetAttribute("shadowsEnabled"))){
                  throw ticpp::Exception("String conversion of in processMesh: shadowsEnabled failed");
              }
         }
      };

      std::string type = meshNode->ToElement()->GetAttribute("type");

      if( type == "permutate" || type == "uniform")
      {
         std::vector<std::string> m_materialList;
         // Iterate over all material, save in list!
         ticpp::Iterator< ticpp::Element > material("Material");
         for ( material = material.begin( meshNode ); material != material.end(); material++ ){
            m_materialList.push_back(material->GetAttribute<std::string>("name"));
         }
         if(m_materialList.empty()){
            throw ticpp::Exception("No Material Node found in Mesh!");
         }



         std::stringstream entity_name,node_name;

         for(int i=0; i<m_bodyList.size();i++){
            entity_name.str(""); node_name.str("");
            entity_name << meshName.filename().string() << std::string("Entity");
            node_name << meshName.filename().string() << std::string("Node");
            entity_name << entityCounter;
            node_name<< nodeCounter;
            //cout << entity_name.str() <<endl;
            Ogre::Entity* ent = m_pSceneMgr->createEntity(entity_name.str(), meshName.string() );
            ent->setCastShadows(shadowsEnabled);
            Ogre::SceneNode* sceneNode = m_BaseFrame->createChildSceneNode(node_name.str());
            Ogre::SceneNode* sceneNodeScale = sceneNode->createChildSceneNode();
            if(scaleLikeGeometry){
               sceneNodeScale->setScale(m_bodyListScales[i](0),m_bodyListScales[i](1),m_bodyListScales[i](2));
            }
            else{
               sceneNodeScale->setScale(scale(0),scale(1),scale(2));
            }
            sceneNodeScale->attachObject(ent);

            if(attachAxis){
               Ogre::SceneNode* sceneNodeAxes = sceneNode->createChildSceneNode(entity_name.str() + "Axes");
               Ogre::Entity* axisEnt = m_pSceneMgr->createEntity(entity_name.str() + "AxesEnt","axes.mesh" );
               sceneNodeAxes->setScale(axesSize,axesSize,axesSize);
               sceneNodeAxes->attachObject(axisEnt);
            }

            int matIdx = i % m_materialList.size();

            ent->setMaterialName(m_materialList[matIdx]);

            //Set initial condition
            sceneNode->setPosition(m_bodyList[i]->m_r_S(0),m_bodyList[i]->m_r_S(1),m_bodyList[i]->m_r_S(2));
            sceneNode->setOrientation(m_bodyList[i]->m_q_KI(0),m_bodyList[i]->m_q_KI(1),m_bodyList[i]->m_q_KI(2),m_bodyList[i]->m_q_KI(3));


            if( m_eBodiesState == RigidBody<TLayoutConfig>::SIMULATED){
               m_rSceneNodeSimBodies.push_back(sceneNode);
            }
            else if( m_eBodiesState == RigidBody<TLayoutConfig>::NOT_SIMULATED){
               m_rSceneNodeBodies.push_back(sceneNode);
            }

            nodeCounter++;
            entityCounter++;
         }



      }
      else{
         throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'processMesh' has no implementation in the parser"));
      }
   }


   //Helper Function
   Vector3 convertToVector3(const aiVector3D & a){
      Vector3 ret;
      ret << a.x, a.y, a.z;
      return ret;
   }

   bool m_bParseDynamics; ///< Parse Dynamics stuff or do not. Playback Manager also has this SceneParser but does not need DynamicsStuff.

   boost::shared_ptr<TSystem> m_pDynSys;

   boost::shared_ptr<Ogre::SceneManager> m_pSceneMgr;
   Ogre::SceneNode * m_BaseFrame;
   std::vector<Ogre::SceneNode*>	&m_rSceneNodeSimBodies;
   std::vector<Ogre::SceneNode*>	&m_rSceneNodeBodies;



   boost::filesystem::path m_currentParseFilePath;
   boost::filesystem::path m_currentParseFileDir;
   ticpp::Document m_xmlDoc;
   const ticpp::Node * m_xmlRootNode;

   Ogre::Log *m_pAppLog;
   std::stringstream logstream;

   unsigned int m_SimBodies;

   // Temprary structures
   typename RigidBody<TLayoutConfig>::BodyState m_eBodiesState; ///< Used to process a RigidBody Node
   std::vector<boost::shared_ptr<RigidBody<TLayoutConfig> > > m_bodyList; ///< Used to process a RigidBody Node
   std::vector<Vector3> m_bodyListScales;
   std::vector< DynamicsState<TLayoutConfig> > m_SimBodyInitStates;

   typedef std::map<std::string, boost::shared_ptr<MeshGeometry<PREC> > > ContainerSceneMeshs;
   ContainerSceneMeshs m_SceneMeshs; // Cache all added meshs geometries, we do not want to add same meshs twice!

};

#endif
