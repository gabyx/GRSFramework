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

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "InclusionSolverSettings.hpp"
#include "TimeStepperSettings.hpp"

#include "MeshGeometry.hpp"

#include "CommonFunctions.hpp"
#include "QuaternionHelpers.hpp"
#include "InertiaTensorCalculations.hpp"
#include "InitialConditionBodies.hpp"

#include <assimp.hpp>      // C++ importer interface
#include <aiScene.h>       // Output data structure
#include <aiPostProcess.h> // Post processing flags

//#include "OgreMeshExtraction.hpp"

#define TIXML_USE_TICPP
#include "ticpp.h"
//#include "tinyxml.h"

/*
* @Does not work yet, to implement a scene parser, implement everything starting from SimulationState, enter(), we exit the State, delete all Objects, and reinitialize with another system in XML format.
*
*/
template<typename TConfig>
class SceneParser {
public:

    DEFINE_CONFIG_TYPES_OF(TConfig)

    SceneParser(boost::shared_ptr<DynamicsSystemType> pDynSys)
        : m_pDynSys(pDynSys) {

        m_pSimulationLog = NULL;
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
        ASSERTMSG(m_pSimulationLog, "There is no SimulationLog in the LogManager!");

        m_bParseDynamics = true;
        m_nSimBodies = 0;
    }

    virtual bool parseScene( boost::filesystem::path file ) {
        using namespace std;
        m_currentParseFilePath = file;
        m_currentParseFileDir = m_currentParseFilePath.parent_path();

        m_pSimulationLog->logMessage("Parsing Scene...");

        LOG( m_pSimulationLog, "Scene Input file: "  << file.string() <<std::endl; );


        //Reset all variables
        m_nSimBodies = 0;
        m_nBodies = 0;
        m_bodyList.clear();
        m_SimBodyInitStates.clear();
        m_SceneMeshs.clear();

        try {
            m_xmlDoc.LoadFile(m_currentParseFilePath.string());

            m_pSimulationLog->logMessage("File successfully loaded ...");

            m_pSimulationLog->logMessage("Try to parse the scene ...");

            // Start off with the gravity!
            m_xmlRootNode = m_xmlDoc.FirstChild("DynamicsSystem");
            if(m_xmlRootNode) {
                ticpp::Node *node = NULL;

                node = m_xmlRootNode->FirstChild("SceneSettings");
                processSceneSettings(node);

                node = m_xmlRootNode->FirstChild("SceneObjects");
                processSceneObjects(node);

                /*ticpp::Node * initialConditionAll = m_xmlRootNode->FirstChild("InitialCondition");
                processinitialConditionAll(initialConditionAll);*/

                processOtherOptions(m_xmlRootNode);

            } else {
                m_pSimulationLog->logMessage("No DynamicsSystem Node found in XML ...");
                return false;
            }

        } catch(ticpp::Exception& ex) {
            LOG(m_pSimulationLog,  "Scene XML error: "  << ex.what() <<std::endl;);
            exit(-1);
        }


        //ASSERTMSG(false,"XML parsing...");

        return true;
    }

    virtual void cleanUp(){

     m_bodyList.clear();
     m_bodyListScales.clear();
     m_SimBodyInitStates.clear();
     m_SceneMeshs.clear();

    }

    virtual boost::filesystem::path getParsedSceneFile() {
        return m_currentParseFilePath;
    }

    virtual const std::vector< DynamicsState<LayoutConfigType> > & getInitialConditionSimBodies() {
        ASSERTMSG(m_SimBodyInitStates.size(), "m_SimBodyInitStates.size() contains no initial states!")
        return m_SimBodyInitStates;
    }

    virtual unsigned int getNumberOfSimBodies() {
        return m_nSimBodies;
    }


protected:

    SceneParser() {
        m_pSimulationLog = NULL;
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
        ASSERTMSG(m_pSimulationLog, "There is no SimulationLog in the LogManager!");
        m_bParseDynamics = false;
        m_nSimBodies = 0;
        m_nBodies = 0;
    }

    virtual void processOtherOptions(const ticpp::Node *rootNode){
        /* Do nothing, here for derived classes! */
    }

    virtual void processSceneSettings( ticpp::Node *sceneSettings ) {

        LOG(m_pSimulationLog,"Process SceneSettings..."<<std::endl;);

        if(m_bParseDynamics) {

            ticpp::Element *gravityElement = sceneSettings->FirstChild("Gravity",true)->ToElement();
            m_pDynSys->m_gravity = gravityElement->GetAttribute<double>("value");

            if(!Utilities::stringToVector3<PREC>(m_pDynSys->m_gravityDir , gravityElement->GetAttribute("direction"))) {
                throw ticpp::Exception("String conversion in SceneSettings: gravity failed");
            }

            ticpp::Element *timestepElement = sceneSettings->FirstChild("TimeStepperSettings",true)->ToElement();

            TimeStepperSettings<LayoutConfigType> timestepperSettings;
            InclusionSolverSettings<LayoutConfigType> inclusionSettings;

            // Get standart values!
            m_pDynSys->getSettings(timestepperSettings,inclusionSettings);

            if(!Utilities::stringToType<PREC>(timestepperSettings.m_deltaT, timestepElement->GetAttribute("deltaT"))) {
                throw ticpp::Exception("String conversion in SceneSettings: deltaT failed");
            }
            inclusionSettings.m_deltaT = timestepperSettings.m_deltaT;

            if(!Utilities::stringToType<PREC>(timestepperSettings.m_endTime, timestepElement->GetAttribute("endTime"))) {
                throw ticpp::Exception("String conversion in SceneSettings: endTime failed");
            }

            ticpp::Element *simFromRef = timestepElement->FirstChild("SimulateFromReference",false)->ToElement();
            if(simFromRef) {

                bool enabled = false;
                if(!Utilities::stringToType<bool>(enabled, simFromRef->GetAttribute("enabled"))) {
                    throw ticpp::Exception("String conversion in SimulateFromReference: enable failed");
                }
                if(enabled) {
                    std::string type = simFromRef->GetAttribute("type");
                    if(type == "useStates") {
                        timestepperSettings.m_eSimulateFromReference = TimeStepperSettings<LayoutConfigType>::USE_STATES;
                    } else if(type == "continue") {
                        timestepperSettings.m_eSimulateFromReference = TimeStepperSettings<LayoutConfigType>::CONTINUE;
                    } else {
                        throw ticpp::Exception("String conversion in SimulateFromReference: type. The type '" + type + std::string("' has no implementation in the parser"));
                    }
                    timestepperSettings.m_simStateReferenceFile = simFromRef->GetAttribute("file");
                    checkFileExists(timestepperSettings.m_simStateReferenceFile);
                } else {
                    timestepperSettings.m_eSimulateFromReference = TimeStepperSettings<LayoutConfigType>::NONE;
                }
            }


            ticpp::Element *inclusionElement = timestepElement->FirstChild("InclusionSolverSettings",false)->ToElement();
            if(inclusionElement) {

                if(!Utilities::stringToType<PREC>(inclusionSettings.m_alphaJORProx, inclusionElement->GetAttribute("alphaJORProx"))) {
                    throw ticpp::Exception("String conversion in SceneSettings: alphaJORProx failed");
                }
                if(!Utilities::stringToType<PREC>(inclusionSettings.m_alphaSORProx, inclusionElement->GetAttribute("alphaSORProx"))) {
                    throw ticpp::Exception("String conversion in SceneSettings: alphaJORProx failed");
                }
                if(!Utilities::stringToType<unsigned int>(inclusionSettings.m_MaxIter, inclusionElement->GetAttribute("maxIter"))) {
                    throw ticpp::Exception("String conversion in SceneSettings: maxIter failed");
                }

                if(inclusionElement->HasAttribute("minIter")){
                    if(!Utilities::stringToType<unsigned int>(inclusionSettings.m_MinIter, inclusionElement->GetAttribute("minIter"))) {
                        throw ticpp::Exception("String conversion in SceneSettings: minIter failed");
                    }
                }else{
                    inclusionSettings.m_MinIter = 0;
                }

                if(inclusionElement->HasAttribute("convergenceMethod")){
                    std::string method = inclusionElement->GetAttribute("convergenceMethod");
                    if(method == "InLambda") {
                    inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings<LayoutConfigType>::InLambda;
                    } else if (method == "InVelocity") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings<LayoutConfigType>::InVelocity;
                    }
                    else if (method == "InVelocityLocal") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings<LayoutConfigType>::InVelocityLocal;
                    }
                    else if (method == "InEnergyVelocity") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings<LayoutConfigType>::InEnergyVelocity;
                    }
                    else if (method == "InEnergyLocalMix") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings<LayoutConfigType>::InEnergyLocalMix;
                    }
                    else {
                        throw ticpp::Exception("String conversion in SceneSettings: convergenceMethod failed: not a valid setting");
                    }
                }else{
                    inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings<LayoutConfigType>::InLambda;
                }

                if(!Utilities::stringToType<PREC>(inclusionSettings.m_AbsTol, inclusionElement->GetAttribute("absTol"))) {
                    throw ticpp::Exception("String conversion in SceneSettings: absTol failed");
                }
                if(!Utilities::stringToType<PREC>(inclusionSettings.m_RelTol, inclusionElement->GetAttribute("relTol"))) {
                    throw ticpp::Exception("String conversion in SceneSettings: relTol failed");
                }

                if(inclusionElement->HasAttribute("isFiniteCheck")) {
                    if(!Utilities::stringToType<bool>(inclusionSettings.m_bIsFiniteCheck, inclusionElement->GetAttribute("isFiniteCheck"))) {
                        throw ticpp::Exception("String conversion in SceneSettings: isFiniteCheck failed");
                    }
                }

                std::string method = inclusionElement->GetAttribute("method");
                if(method == "JOR") {
                    inclusionSettings.m_eMethod = InclusionSolverSettings<LayoutConfigType>::JOR;
                } else if (method == "SOR") {
                    inclusionSettings.m_eMethod = InclusionSolverSettings<LayoutConfigType>::SOR;
                } else {
                    throw ticpp::Exception("String conversion in SceneSettings: method failed: not a valid setting");
                }


                if(!Utilities::stringToType<bool>(inclusionSettings.m_bUseGPU, inclusionElement->GetAttribute("useGPU"))) {
                    throw ticpp::Exception("String conversion in SceneSettings: useGPU failed");
                }

                if(inclusionElement->HasAttribute("useGPUID")) {
                    if(!Utilities::stringToType<int>(inclusionSettings.m_UseGPUDeviceId, inclusionElement->GetAttribute("useGPUID"))) {
                        throw ticpp::Exception("String conversion in SceneSettings: useGPU failed");
                    }
                    if(inclusionSettings.m_UseGPUDeviceId <0) {
                        inclusionSettings.m_UseGPUDeviceId = 0;
                    }
                }

            }
            //Write all values back
            m_pDynSys->setSettings(timestepperSettings,inclusionSettings);



        }

    }


    virtual void processSceneObjects( ticpp::Node *sceneObjects) {

        LOG(m_pSimulationLog,"Process SceneObjects ..."<<std::endl;);

        ticpp::Iterator< ticpp::Node > child;

        for ( child = child.begin( sceneObjects ); child != child.end(); child++ ) {

            if( child->Value() == "RigidBodies") {
                processRigidBodies( &(*child) );
            }

        }

        if( m_nSimBodies == 0) {
            throw ticpp::Exception("The scene in the XML contains no simulating bodies!");
        }

    }

    virtual void processRigidBodies( ticpp::Node * rigidbodies ) {

        LOG(m_pSimulationLog,"Process RigidBodies ..."<<std::endl;);

        //Clear current body list;
        m_bodyList.clear();
        m_bodyListScales.clear();

        int instances = rigidbodies->ToElement()->GetAttribute<int>("instances");


        for(int i=0; i<instances; i++) {
            boost::shared_ptr< RigidBodyType > temp_ptr(new RigidBodyType());
            temp_ptr->m_id = i;
            m_bodyList.push_back(temp_ptr);

            Vector3 scale;
            scale.setOnes();
            m_bodyListScales.push_back(scale);
        }
        LOG(m_pSimulationLog,"Added "<<instances<<" RigidBody Instances..."<<std::endl;);


        ticpp::Node * geometryNode = rigidbodies->FirstChild("Geometry");
        processGeometry(geometryNode);


        ticpp::Node * dynPropNode = rigidbodies->FirstChild("DynamicProperties");
        processDynamicProperties(dynPropNode);






        //Copy the pointers!
        LOG(m_pSimulationLog,"Copy RigidBody References to DynamicSystem ..."<<std::endl;);
        if(m_eBodiesState == RigidBodyType::SIMULATED) {
            if(m_bParseDynamics) {
                for(int i=0; i < m_bodyList.size(); i++) {
                    m_pDynSys->m_SimBodies.push_back(m_bodyList[i]);
                }
            }
            m_nSimBodies += instances;
            m_nBodies += instances;
        } else if(m_eBodiesState == RigidBodyType::NOT_SIMULATED) {
            if(m_bParseDynamics) {
                for(int i=0; i < m_bodyList.size(); i++) {
                    m_pDynSys->m_Bodies.push_back(m_bodyList[i]);
                }
            }
            m_nBodies += instances;
        } else {
            throw ticpp::Exception("Adding only simulated and not simulated objects supported!");
        }





        ticpp::Node * visualizationNode = rigidbodies->FirstChild("Visualization");
        processVisualization( visualizationNode);





    }

    virtual void processGeometry( ticpp::Node * geometryNode) {
        LOG(m_pSimulationLog,"Process Geometry ..."<<std::endl;);
        if(geometryNode->FirstChild()->Value() == "Sphere") {

            processSphereGeometry( geometryNode->FirstChild()->ToElement());
        } else if(geometryNode->FirstChild()->Value() == "Halfspace") {

            processHalfspaceGeometry( geometryNode->FirstChild()->ToElement());
        } else if(geometryNode->FirstChild()->Value() == "Mesh") {

            processMeshGeometry( geometryNode->FirstChild()->ToElement());

        } else {
            throw ticpp::Exception("The geometry '" + std::string(geometryNode->FirstChild()->Value()) + std::string("' has no implementation in the parser"));
        }
    }

    virtual void processSphereGeometry( ticpp::Element * sphere) {
        std::string type = sphere->GetAttribute("distribute");
        if(type == "uniform") {

            double radius = sphere->GetAttribute<double>("radius");
            Vector3 scale;
            scale(0)=radius;
            scale(1)=radius;
            scale(2)=radius;

            boost::shared_ptr<SphereGeometry<PREC> > pSphereGeom = boost::shared_ptr<SphereGeometry<PREC> >(new SphereGeometry<PREC>(radius));

            for(int i=0; i < m_bodyList.size(); i++) {
                m_bodyListScales[i] = scale;
                m_bodyList[i]->m_geometry = pSphereGeom;
            }
        } else if(type == "random") {
            double minRadius;
            if(!Utilities::stringToType<double>(minRadius,sphere->GetAttribute("minRadius"))) {
                throw ticpp::Exception("String conversion in processSphereGeometry: minRadius failed");
            }
            if( minRadius <= 0) {
                throw ticpp::Exception("In processSphereGeometry: minRadius to small!");
            }

            double maxRadius;
            if(!Utilities::stringToType<double>(maxRadius,sphere->GetAttribute("maxRadius"))) {
                throw ticpp::Exception("String conversion in processSphereGeometry: minRadius failed");
            }
            if( maxRadius <= minRadius) {
                throw ticpp::Exception("In processSphereGeometry: maxRadius smaller or equal to minRadius!");
            }

            unsigned int seed;
            if(!Utilities::stringToType<unsigned int>(seed,sphere->GetAttribute("seed"))) {
                throw ticpp::Exception("String conversion in processSphereGeometry: seed failed");
            }

            typedef boost::mt19937  RNG;
            RNG generator(seed);
            boost::uniform_real<PREC> uniform(minRadius,maxRadius);
            boost::variate_generator< boost::mt19937 & , boost::uniform_real<PREC> > randomNumber(generator, uniform);

            for(int i=0; i < m_bodyList.size(); i++) {
                double radius = randomNumber();
                Vector3 scale;
                scale(0)=radius;
                scale(1)=radius;
                scale(2)=radius;
                m_bodyListScales[i] = scale;
                boost::shared_ptr<SphereGeometry<PREC> > pSphereGeom = boost::shared_ptr<SphereGeometry<PREC> >(new SphereGeometry<PREC>(radius));
                m_bodyList[i]->m_geometry = pSphereGeom;
            }
        } else {
            throw ticpp::Exception("The attribute 'distribute' '" + type + std::string("' of 'Sphere' has no implementation in the parser"));
        }
    }

    virtual void processHalfspaceGeometry( ticpp::Element * halfspace) {
        std::string type = halfspace->GetAttribute("distribute");
        if(type == "uniform") {

            Vector3 n;
            if(!Utilities::stringToVector3<PREC>(n, halfspace->GetAttribute("normal"))) {
                throw ticpp::Exception("String conversion in HalfsphereGeometry: normal failed");
            }

            Vector3 p;
            if(!Utilities::stringToVector3<PREC>(p, halfspace->GetAttribute("position"))) {
                throw ticpp::Exception("String conversion in HalfsphereGeometry: position failed");
            }

            boost::shared_ptr<HalfspaceGeometry<PREC> > pHalfspaceGeom = boost::shared_ptr<HalfspaceGeometry<PREC> >(new HalfspaceGeometry<PREC>(n,p));

            for(int i=0; i < m_bodyList.size(); i++) {
                m_bodyList[i]->m_geometry = pHalfspaceGeom;
            }

        } else {
            throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'Sphere' has no implementation in the parser"));
        }
    }


    virtual void processMeshGeometry( ticpp::Element * mesh) {

        boost::shared_ptr<MeshGeometry<PREC> > pMeshGeom;

        std::string meshName = mesh->GetAttribute<std::string>("name");

        bool bInstantiate;
        if(!Utilities::stringToType<bool>(bInstantiate,mesh->GetAttribute("useInstance"))) {
            throw ticpp::Exception("String conversion in processMeshGeometry: useInstance failed");
        }

        std::string type = mesh->GetAttribute("distribute");
        if(type == "uniform") {

            // import here an object model;
            // First make an meshinformatino structure

            boost::filesystem::path fileName =  mesh->GetAttribute<std::string>("file");
            checkFileExists(fileName);
            // if this file has already been added--> skip it and reference the
            typename ContainerSceneMeshs::iterator it = m_SceneMeshs.find(meshName);

            if( bInstantiate == false) {

                if(it != m_SceneMeshs.end()) {
                    throw ticpp::Exception("Already defined mesh with name:" + meshName);
                }

                Vector3 scale_factor;
                if(!Utilities::stringToVector3<PREC>(scale_factor, mesh->GetAttribute("scale"))) {
                    throw ticpp::Exception("String conversion in processMeshGeometry failed: scale");
                }
                if(scale_factor.norm()==0) {
                    throw ticpp::Exception("Wrong scale factor (=0) specified in processMeshGeometry!");
                }

                Vector3 trans;
                if(!Utilities::stringToVector3<PREC>(trans, mesh->GetAttribute("translation"))) {
                    throw ticpp::Exception("String conversion in processMeshGeometry: translation failed: ");
                }

                Vector3 axis;
                if(!Utilities::stringToVector3<PREC>(axis, mesh->GetAttribute("rotationAxis"))) {
                    throw ticpp::Exception("String conversion in processMeshGeometry: rotationAxis failed");
                }

                PREC angle;

                if(mesh->HasAttribute("angleDegree")) {
                    if(!Utilities::stringToType<PREC>(angle, mesh->GetAttribute("angleDegree"))) {
                        throw ticpp::Exception("String conversion in processMeshGeometry: angleDegree failed");
                    }
                    angle = angle / 180 * M_PI;
                } else if(mesh->HasAttribute("angleRadian")) {
                    if(!Utilities::stringToType<PREC>(angle, mesh->GetAttribute("angleRadian"))) {
                        throw ticpp::Exception("String conversion in processMeshGeometry: angleRadian  failed");
                    }
                } else {
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
                if(!scene) {
                    throw ticpp::Exception("File import failed in processMeshGeometry: for file" + fileName.string() );
                }

                boost::shared_ptr<MeshData<MeshPREC> > meshData = boost::shared_ptr<MeshData<MeshPREC> >(new MeshData<MeshPREC>);

                if(!meshData->setup(importer,scene, scale_factor,quat,trans)) {
                    throw ticpp::Exception("Imported Mesh (with Assimp) could not be setup internally");
                }

                // Build Geometry
                pMeshGeom = boost::shared_ptr<MeshGeometry<PREC> >(new MeshGeometry<PREC>(meshData));

                // Add geometry into the cache
                m_SceneMeshs.insert(typename ContainerSceneMeshs::value_type(meshName,pMeshGeom));

            } else {
                if(it == m_SceneMeshs.end()) {
                    throw ticpp::Exception("Mesh with name: " + meshName + " not found!");
                }
                // MeshData exists already for this file fileName
                pMeshGeom = it->second;
            }


            // Assign Geometry
            for(int i=0; i < m_bodyList.size(); i++) {
                m_bodyList[i]->m_geometry = pMeshGeom;
            }

        } else {
            throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'Mesh' has no implementation in the parser"));
        }
    }

    virtual void fillMeshInfo( Assimp::Importer & importer, const aiScene* scene, MeshData<MeshPREC> & meshInfo, Vector3 scale_factor, Quaternion quat, Vector3 trans) {



    }

    virtual void checkFileExists(boost::filesystem::path file) {
        if( !boost::filesystem::exists(file) ) {
            throw ticpp::Exception("The file ' " + file.string() + "' does not exist!");
        }
    }

    virtual void processDynamicProperties( ticpp::Node * dynProp) {
        LOG(m_pSimulationLog,"Process DynamicProperties ..."<<std::endl;);
        ticpp::Element * element = dynProp->FirstChild("DynamicState")->ToElement();

        std::string type = element->GetAttribute("type");
        if(type == "simulated") {
            m_eBodiesState =  RigidBodyType::SIMULATED;

            processDynamicPropertiesSimulated(dynProp);
        } else if(type == "not simulated") {
            m_eBodiesState =  RigidBodyType::NOT_SIMULATED;

            processDynamicPropertiesNotSimulated(dynProp);
        } else if(type == "animated") {
            m_eBodiesState =  RigidBodyType::ANIMATED;
            throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
        } else {
            throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
        }

        for(int i=0; i < m_bodyList.size(); i++) {
            m_bodyList[i]->m_eState = m_eBodiesState;
        }

    }


    virtual void processDynamicPropertiesSimulated( ticpp::Node * dynProp) {

        // First allocate a new SolverDate structure
        for(int i=0; i < m_bodyList.size(); i++) {
            m_bodyList[i]->m_pSolverData = new RigidBodySolverDataType();
        }


        // Mass ============================================================
        ticpp::Element *element = dynProp->FirstChild("Mass")->ToElement();
        std::string distribute = element->GetAttribute("distribute");
        if(distribute == "uniform") {

            double mass = element->GetAttribute<double>("value");

            for(int i=0; i < m_bodyList.size(); i++) {
                m_bodyList[i]->m_mass = mass;
            }


        } else {
            throw ticpp::Exception("The attribute 'distribute' '" + distribute + std::string("' of 'Mass' has no implementation in the parser"));
        }

        // InertiaTensor ============================================================
        element = dynProp->FirstChild("InertiaTensor")->ToElement();
        std::string type = element->GetAttribute("type");
        if(type == "homogen") {
            for(int i=0; i < m_bodyList.size(); i++) {
                InertiaTensor::calculateInertiaTensor(m_bodyList[i]);
            }
        } else {
            throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'InertiaTensor' has no implementation in the parser"));
        }

        element = dynProp->FirstChild("Material")->ToElement();
        distribute = element->GetAttribute("distribute");
        if(distribute == "uniform") {
            typename RigidBodyType::BodyMaterial eMaterial;
            type = element->GetAttribute("type");
            if(type == "standart") {
                eMaterial = RigidBodyType::STD_MATERIAL;
            } else if( type == "wood" ) {
                eMaterial = RigidBodyType::WOOD;
            } else if( type == "metal") {
                eMaterial = RigidBodyType::METAL;
            } else if( type == "glas") {
                eMaterial = RigidBodyType::GLAS;
            } else {
                eMaterial = RigidBodyType::STD_MATERIAL;
            }
            for(int i=0; i < m_bodyList.size(); i++) {
                m_bodyList[i]->m_eMaterial = eMaterial;
            }
        } else {
            throw ticpp::Exception("The attribute 'distribute' '" + distribute + std::string("' of 'Material' has no implementation in the parser"));
        }

         // InitialCondition ============================================================
        element = dynProp->FirstChild("InitialPosition")->ToElement();
        distribute = element->GetAttribute("distribute");

        m_SimBodyInitStates.push_back( DynamicsState<LayoutConfigType>((unsigned int)m_bodyList.size()));

        if(distribute == "linear") {
            processInitialConditionLinear(m_SimBodyInitStates.back(),element);
        } else if(distribute == "grid") {
            processInitialConditionGrid(m_SimBodyInitStates.back(),element);
        } else if(distribute == "file") {
            processInitialConditionFile(m_SimBodyInitStates.back(),element);
        } else if(distribute == "posaxisangle") {
            processInitialConditionPositionAxisAngle(m_SimBodyInitStates.back(),element);
        } else if(distribute == "transforms") {
            processInitialConditionTransforms(m_SimBodyInitStates.back(),element);
        } else if(distribute == "none") {
            // does nothing leaves the zero state pushed!
        } else {
            throw ticpp::Exception("The attribute 'distribute' '" + distribute + std::string("' of 'InitialCondition' has no implementation in the parser"));
        }

        InitialConditionBodies::applyDynamicsStateToBodies(m_SimBodyInitStates.back(), m_bodyList);
    }


    virtual void processDynamicPropertiesNotSimulated( ticpp::Node * dynProp) {

        // InitialCondition ============================================================
        ticpp::Element *element = dynProp->FirstChild("InitialPosition")->ToElement();
        std::string distribute = element->GetAttribute("distribute");

        DynamicsState<LayoutConfigType> state((unsigned int)m_bodyList.size());
        if(distribute == "linear") {
            processInitialConditionLinear(state,element);
        } else if(distribute == "grid") {
            processInitialConditionGrid(state,element);
        } else if(distribute == "file") {
            processInitialConditionFile(state,element);
        } else if(distribute == "posaxisangle") {
            processInitialConditionPositionAxisAngle(state,element);
        } else if(distribute == "transforms") {
            processInitialConditionTransforms(state,element);
        } else if(distribute == "none") {
            // does nothing leaves the zero state pushed!
        } else {
            throw ticpp::Exception("The attribute 'distribute' '" + distribute + std::string("' of 'InitialCondition' has no implementation in the parser"));
        }

        InitialConditionBodies::applyDynamicsStateToBodies(state, m_bodyList);

    }


    virtual void processInitialConditionLinear(DynamicsState<LayoutConfigType> & state, ticpp::Element * initCond) {

        Vector3 pos;
        if(!Utilities::stringToVector3<PREC>(pos, initCond->GetAttribute("position"))) {
            throw ticpp::Exception("String conversion in InitialCondition: position Linear failed");
        }
        Vector3 dir;
        if(!Utilities::stringToVector3<PREC>(dir, initCond->GetAttribute("direction"))) {
            throw ticpp::Exception("String conversion in InitialCondition: direction Linear failed");
        }
        PREC dist;
        if(!Utilities::stringToType<PREC>(dist, initCond->GetAttribute("distance"))) {
            throw ticpp::Exception("String conversion in InitialCondition: distance  Linear failed");
        }
        bool jitter;
        if(!Utilities::stringToType(jitter, initCond->GetAttribute("jitter"))) {
            throw ticpp::Exception("String conversion in InitialCondition: jitter Linear failed");
        }

        PREC delta;
        if(!Utilities::stringToType<PREC>(delta, initCond->GetAttribute("delta"))) {
            throw ticpp::Exception("String conversion in InitialCondition: delta Linear failed");
        }

        unsigned int seed = 5;
        if(initCond->HasAttribute("seed")){
            if(!Utilities::stringToType(seed, initCond->GetAttribute("seed"))) {
                throw ticpp::Exception("String conversion in InitialConditionGrid: jitter seed failed");
            }
        }

        InitialConditionBodies::setupBodiesLinear(state,pos,dir,dist,jitter,delta,seed);

    }

    virtual void processInitialConditionGrid(DynamicsState<LayoutConfigType> & state, ticpp::Element * initCond) {

        Vector3 trans;
        if(!Utilities::stringToVector3<PREC>(trans, initCond->GetAttribute("translation"))) {
            throw ticpp::Exception("String conversion in InitialConditionGrid: translation failed");
        }
        int gridX;
        if(!Utilities::stringToType<int>(gridX, initCond->GetAttribute("gridSizeX"))) {
            throw ticpp::Exception("String conversion in InitialConditionGrid: gridSizeX failed");
        }
        int gridY;
        if(!Utilities::stringToType<int>(gridY, initCond->GetAttribute("gridSizeY"))) {
            throw ticpp::Exception("String conversion in InitialConditionGrid: gridSizeY failed");
        }
        PREC dist;
        if(!Utilities::stringToType<PREC>(dist, initCond->GetAttribute("distance"))) {
            throw ticpp::Exception("String conversion in InitialConditionGrid: distance failed");
        }
        bool jitter;
        if(!Utilities::stringToType(jitter, initCond->GetAttribute("jitter"))) {
            throw ticpp::Exception("String conversion in InitialConditionGrid: jitter failed");
        }
        int seed = 5;
        if(initCond->HasAttribute("seed")){
            if(!Utilities::stringToType(seed, initCond->GetAttribute("seed"))) {
                throw ticpp::Exception("String conversion in InitialConditionGrid: jitter seed failed");
            }
        }
        double delta;
        if(!Utilities::stringToType<double>(delta, initCond->GetAttribute("delta"))) {
            throw ticpp::Exception("String conversion in InitialConditionGrid: delta failed");
        }

        InitialConditionBodies::setupBodiesGrid(state,gridX,gridY,dist,trans,jitter,delta, seed);
    }

    virtual void processInitialConditionFile(DynamicsState<LayoutConfigType> & state, ticpp::Element * initCond) {
        m_SimBodyInitStates.push_back(DynamicsState<LayoutConfigType>((unsigned int)m_bodyList.size()));

        boost::filesystem::path name =  initCond->GetAttribute<std::string>("name");

        boost::filesystem::path filePath = m_currentParseFileDir / name;
        InitialConditionBodies::setupBodiesFromFile(state,filePath);
    }

    virtual void processInitialConditionPositionAxisAngle(DynamicsState<LayoutConfigType> & state, ticpp::Element * initCond) {

        int bodyCounter = 0;
        // Iterate over all values in the list
        ticpp::Iterator< ticpp::Element > valueElem("Value");
        for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++) {

            Vector3 pos;
            if(!Utilities::stringToVector3<PREC>(pos, valueElem->GetAttribute("position"))) {
                throw ticpp::Exception("String conversion in InitialConditionPositionAxisAngle: position failed");
            }
            Vector3 axis;
            if(!Utilities::stringToVector3<PREC>(axis, valueElem->GetAttribute("axis"))) {
                throw ticpp::Exception("String conversion in InitialConditionPositionAxisAngle: axis failed");
            }

            if( axis.norm() == 0) {
                throw ticpp::Exception("Specified wrong axis in InitialConditionPositionAxisAngle");
            }

            PREC angle;

            if(valueElem->HasAttribute("angleDegree")) {
                if(!Utilities::stringToType<PREC>(angle, valueElem->GetAttribute("angleDegree"))) {
                    throw ticpp::Exception("String conversion in InitialConditionPositionAxisAngle: angleDegree failed");
                }
                angle = angle / 180 * M_PI;
            } else if(valueElem->HasAttribute("angleRadian")) {
                if(!Utilities::stringToType<PREC>(angle, valueElem->GetAttribute("angleRadian"))) {
                    throw ticpp::Exception("String conversion in InitialConditionPositionAxisAngle: angleRadian failed");
                }
            } else {
                throw ticpp::Exception("No angle found in InitialConditionPositionAxisAngle");
            }

            if(bodyCounter >= state.m_SimBodyStates.size()) {
                throw ticpp::Exception("To many intial condition specified!");
            }


            InitialConditionBodies::setupBodyPositionAxisAngle(state.m_SimBodyStates[bodyCounter], pos, axis, angle);

            bodyCounter++;
        }

        if(state.m_SimBodyStates.size() != bodyCounter){
            throw ticpp::Exception("To little intial condition specified!");
        }
    }

    virtual void processInitialConditionTransforms(DynamicsState<LayoutConfigType> & state, ticpp::Element * initCond) {



        int bodyCounter = 0;
        // Iterate over all values in the list
        ticpp::Iterator< ticpp::Element > valueElem("Value");
        for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++) {


            Quaternion q_KI, q_BK;
            setQuaternionZero(q_KI);
            Vector3 I_r_IK, K_r_KB;
            I_r_IK.setZero();
            Matrix33 Rot_KI; // Temp

            // Iterate over all transforms an successfully applying the total trasnformation!
            ticpp::Iterator< ticpp::Element > transformElem("Transform");
            for ( transformElem = transformElem.begin( &(*valueElem) ); transformElem != transformElem.end(); transformElem++) {

                Vector3 trans;
                if(!Utilities::stringToVector3<PREC>(trans, transformElem->GetAttribute("translation"))) {
                    throw ticpp::Exception("String conversion in InitialConditionTransforms: translation failed");
                }
                Vector3 axis;
                if(!Utilities::stringToVector3<PREC>(axis, transformElem->GetAttribute("rotationAxis"))) {
                    throw ticpp::Exception("String conversion in InitialConditionTransforms: rotationAxis failed");
                }

                if( axis.norm() == 0) {
                    throw ticpp::Exception("Specified wrong axis in InitialConditionTransforms");
                }

                PREC angle;
                if(transformElem->HasAttribute("angleDegree")) {
                    if(!Utilities::stringToType<PREC>(angle, transformElem->GetAttribute("angleDegree"))) {
                        throw ticpp::Exception("String conversion in InitialConditionTransforms: angleDegree failed");
                    }
                    angle = angle / 180 * M_PI;
                } else if(transformElem->HasAttribute("angleRadian")) {
                    if(!Utilities::stringToType<PREC>(angle, transformElem->GetAttribute("angleRadian"))) {
                        throw ticpp::Exception("String conversion in InitialConditionTransforms: angleRadian failed");
                    }
                } else {
                    throw ticpp::Exception("No angle found in InitialConditionTransforms");
                }

                setQuaternion(q_BK,axis,angle);
                K_r_KB = trans;
                Rot_KI = getRotFromQuaternion(q_KI);
                I_r_IK += Rot_KI * K_r_KB; // Transforms like A_IK * A_r_AB;
                q_KI = quatMult(q_KI,q_BK); // Sequential (aktiv) rotation

            }

            if(bodyCounter >= state.m_SimBodyStates.size()) {
                throw ticpp::Exception("To many intial condition specified!");
            }
            // Apply overall transformation!
            state.m_SimBodyStates[bodyCounter].m_q.template head<3>() = I_r_IK;
            state.m_SimBodyStates[bodyCounter].m_q.template tail<4>() = q_KI;

            bodyCounter++;
        }

        if(bodyCounter != state.m_SimBodyStates.size()) {
            throw ticpp::Exception("To many little intial condition specified!");
        }

    }



    virtual void processVisualization( ticpp::Node * visualizationNode) {

        ticpp::Node * meshNode = visualizationNode->FirstChild("Mesh");

        processMesh(meshNode);

    }

    virtual void processMesh( ticpp::Node * meshNode ) {
        /* Do nothing here: A derived version of this parser may process the visualization if needed*/
    }



    bool m_bParseDynamics; ///< Parse Dynamics stuff or do not. Playback Manager also has this SceneParser but does not need DynamicsStuff.

    boost::shared_ptr<DynamicsSystemType> m_pDynSys;

    boost::filesystem::path m_currentParseFilePath;
    boost::filesystem::path m_currentParseFileDir;
    ticpp::Document m_xmlDoc;
    const ticpp::Node * m_xmlRootNode;

    Logging::Log * m_pSimulationLog;
    std::stringstream logstream;

    unsigned int m_nSimBodies, m_nBodies;
    // Temprary structures
    typename RigidBodyType::BodyState m_eBodiesState; ///< Used to process a RigidBody Node
    std::vector<boost::shared_ptr< RigidBodyType > > m_bodyList; ///< Used to process a RigidBody Node
    std::vector<Vector3> m_bodyListScales;
    std::vector< DynamicsState<LayoutConfigType> > m_SimBodyInitStates;

    typedef std::map<std::string, boost::shared_ptr<MeshGeometry<PREC> > > ContainerSceneMeshs;
    ContainerSceneMeshs m_SceneMeshs; // Cache all added meshs geometries, we do not want to add same meshs twice!

};


#endif
