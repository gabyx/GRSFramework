#ifndef SceneParser_hpp
#define SceneParser_hpp

#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/filesystem.hpp>
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>       // Output data structure
#include <assimp/postprocess.h> // Post processing flags

#define TIXML_USE_TICPP
#include "ticpp/ticpp.h"
//#include "tinyxml.h"

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"
#include "LogDefines.hpp"

#include DynamicsSystem_INCLUDE_FILE

#include "RigidBodyId.hpp"
#include "ContactParams.hpp"
#include "MeshGeometry.hpp"
#include "ExternalForces.hpp"

#include "CommonFunctions.hpp"
#include "QuaternionHelpers.hpp"
#include "InertiaTensorCalculations.hpp"
#include "InitialConditionBodies.hpp"


#include InclusionSolverSettings_INCLUDE_FILE
#include "TimeStepperSettings.hpp"
#include "RecorderSettings.hpp"

#include "PrintGeometryDetails.hpp"


class GetScaleOfGeomVisitor : public boost::static_visitor<> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    GetScaleOfGeomVisitor(Vector3 & scale): m_scale(scale) {};

    void operator()(  boost::shared_ptr<const SphereGeometry >  & sphereGeom ) {
        m_scale.setConstant(sphereGeom->m_radius);
    }
    void operator()(  boost::shared_ptr<const BoxGeometry >  & boxGeom) {
        m_scale = boxGeom->m_extent;
    }

    template<typename T>
    void operator()(  boost::shared_ptr<T>  & ptr) {
        ERRORMSG("This GetScaleOfGeomVisitor visitor operator() has not been implemented!");
    }

    Vector3 & m_scale;
};

class SceneParser {
public:

    DEFINE_CONFIG_TYPES
    typedef DynamicsSystemType::RigidBodyStatesContainerType RigidBodyStatesContainerType;


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

        m_pSimulationLog->logMessage("---> Parsing Scene...");
        LOG( m_pSimulationLog, "---> Scene Input file: "  << file.string() <<std::endl; );

        if(!boost::filesystem::exists(m_currentParseFilePath)) {
            ERRORMSG("Scene Input file does not exist!");
        }


        //Reset all variables
        m_globalMaxGroupId = 0;
        m_nSimBodies = 0;
        m_nBodies = 0;
        m_bodyListGroup.clear();
        m_bodyScalesGroup.clear();
        m_initStatesGroup.clear();


        try {
            m_xmlDoc.LoadFile(m_currentParseFilePath.string());

            m_pSimulationLog->logMessage("---> File successfully loaded ...");

            m_pSimulationLog->logMessage("---> Try to parse the scene ...");

            // Start off with the gravity!
            m_xmlRootNode = m_xmlDoc.FirstChild("DynamicsSystem");
            if(m_xmlRootNode) {
                ticpp::Node *node = NULL;

                node = m_xmlRootNode->FirstChild("SceneSettings");
                processSceneSettings(node);

                node = m_xmlRootNode->FirstChild("SceneObjects");
                processSceneObjects(node);

                node = m_xmlRootNode->FirstChild("SceneSettings");
                processSceneSettings2(node);

                processOtherOptions(m_xmlRootNode);

            } else {
                m_pSimulationLog->logMessage("---> No DynamicsSystem Node found in XML ...");
                return false;
            }

        } catch(ticpp::Exception& ex) {
            LOG(m_pSimulationLog,  "Scene XML error: "  << ex.what() <<std::endl;);
            exit(-1);
        }


        //ASSERTMSG(false,"XML parsing...");

        return true;
    }

    virtual void cleanUp() {

        m_bodyListGroup.clear();
        m_bodyScalesGroup.clear();
        m_initStatesGroup.clear();

    }

    virtual boost::filesystem::path getParsedSceneFile() {
        return m_currentParseFilePath;
    }

    virtual const RigidBodyStatesContainerType & getInitialConditionSimBodies() {
        ASSERTMSG(m_initStates.size(), "m_initStates.size() contains no initial states!")
        return m_initStates;
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

    virtual void processOtherOptions(const ticpp::Node *rootNode) {
        /* Do nothing, here for derived classes! */
    }

    virtual void processSceneSettings( ticpp::Node *sceneSettings ) {

        LOG(m_pSimulationLog,"---> Process SceneSettings..."<<std::endl;);

        if(m_bParseDynamics) {

            ticpp::Element *gravityElement = sceneSettings->FirstChild("Gravity",true)->ToElement();
            m_pDynSys->m_gravity = gravityElement->GetAttribute<double>("value");

            if(!Utilities::stringToVector3<PREC>(m_pDynSys->m_gravityDir , gravityElement->GetAttribute("direction"))) {
                throw ticpp::Exception("---> String conversion in SceneSettings: gravity failed");
            }

            m_pDynSys->m_gravityDir.normalize();

            ticpp::Element *timestepElement = sceneSettings->FirstChild("TimeStepperSettings",true)->ToElement();

            TimeStepperSettings timestepperSettings;
            InclusionSolverSettings inclusionSettings;

            // Get standart values!
            m_pDynSys->getSettings(timestepperSettings,inclusionSettings);

            if(!Utilities::stringToType<PREC>(timestepperSettings.m_deltaT, timestepElement->GetAttribute("deltaT"))) {
                throw ticpp::Exception("---> String conversion in SceneSettings: deltaT failed");
            }
            inclusionSettings.m_deltaT = timestepperSettings.m_deltaT;

            if(!Utilities::stringToType<PREC>(timestepperSettings.m_endTime, timestepElement->GetAttribute("endTime"))) {
                throw ticpp::Exception("---> String conversion in SceneSettings: endTime failed");
            }

            ticpp::Element *simFromRef = timestepElement->FirstChild("SimulateFromReference",false)->ToElement();
            if(simFromRef) {

                bool enabled = false;
                if(!Utilities::stringToType<bool>(enabled, simFromRef->GetAttribute("enabled"))) {
                    throw ticpp::Exception("---> String conversion in SimulateFromReference: enable failed");
                }
                if(enabled) {
                    std::string type = simFromRef->GetAttribute("type");
                    if(type == "useStates") {
                        timestepperSettings.m_eSimulateFromReference = TimeStepperSettings::USE_STATES;
                    } else if(type == "continue") {
                        timestepperSettings.m_eSimulateFromReference = TimeStepperSettings::CONTINUE;
                    } else {
                        throw ticpp::Exception("---> String conversion in SimulateFromReference: type. The type '" + type + std::string("' has no implementation in the parser"));
                    }
                    timestepperSettings.m_simStateReferenceFile = simFromRef->GetAttribute("file");
                    checkFileExists(timestepperSettings.m_simStateReferenceFile);
                } else {
                    timestepperSettings.m_eSimulateFromReference = TimeStepperSettings::NONE;
                }
            }


            ticpp::Element *inclusionElement = timestepElement->FirstChild("InclusionSolverSettings",false)->ToElement();
            if(inclusionElement) {

                if(!Utilities::stringToType<PREC>(inclusionSettings.m_alphaJORProx, inclusionElement->GetAttribute("alphaJORProx"))) {
                    throw ticpp::Exception("---> String conversion in SceneSettings: alphaJORProx failed");
                }
                if(!Utilities::stringToType<PREC>(inclusionSettings.m_alphaSORProx, inclusionElement->GetAttribute("alphaSORProx"))) {
                    throw ticpp::Exception("---> String conversion in SceneSettings: alphaJORProx failed");
                }
                if(!Utilities::stringToType<unsigned int>(inclusionSettings.m_MaxIter, inclusionElement->GetAttribute("maxIter"))) {
                    throw ticpp::Exception("---> String conversion in SceneSettings: maxIter failed");
                }

                if(inclusionElement->HasAttribute("minIter")) {
                    if(!Utilities::stringToType<unsigned int>(inclusionSettings.m_MinIter, inclusionElement->GetAttribute("minIter"))) {
                        throw ticpp::Exception("---> String conversion in SceneSettings: minIter failed");
                    }
                } else {
                    inclusionSettings.m_MinIter = 0;
                }

                if(inclusionElement->HasAttribute("convergenceMethod")) {
                    std::string method = inclusionElement->GetAttribute("convergenceMethod");
                    if(method == "InLambda") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings::InLambda;
                    } else if (method == "InVelocity") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings::InVelocity;
                    } else if (method == "InVelocityLocal") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings::InVelocityLocal;
                    } else if (method == "InEnergyVelocity") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings::InEnergyVelocity;
                    } else if (method == "InEnergyLocalMix") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings::InEnergyLocalMix;
                    } else {
                        throw ticpp::Exception("---> String conversion in SceneSettings: convergenceMethod failed: not a valid setting");
                    }
                } else {
                    inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings::InLambda;
                }

                if(!Utilities::stringToType<PREC>(inclusionSettings.m_AbsTol, inclusionElement->GetAttribute("absTol"))) {
                    throw ticpp::Exception("---> String conversion in SceneSettings: absTol failed");
                }
                if(!Utilities::stringToType<PREC>(inclusionSettings.m_RelTol, inclusionElement->GetAttribute("relTol"))) {
                    throw ticpp::Exception("---> String conversion in SceneSettings: relTol failed");
                }

                if(inclusionElement->HasAttribute("isFiniteCheck")) {
                    if(!Utilities::stringToType<bool>(inclusionSettings.m_bIsFiniteCheck, inclusionElement->GetAttribute("isFiniteCheck"))) {
                        throw ticpp::Exception("---> String conversion in SceneSettings: isFiniteCheck failed");
                    }
                }

                std::string method = inclusionElement->GetAttribute("method");
                if(method == "JOR") {
                    inclusionSettings.m_eMethod = InclusionSolverSettings::JOR;
                } else if (method == "SOR") {
                    inclusionSettings.m_eMethod = InclusionSolverSettings::SOR;
                } else {
                    throw ticpp::Exception("---> String conversion in SceneSettings: method failed: not a valid setting");
                }


                if(!Utilities::stringToType<bool>(inclusionSettings.m_bUseGPU, inclusionElement->GetAttribute("useGPU"))) {
                    throw ticpp::Exception("---> String conversion in SceneSettings: useGPU failed");
                }

                if(inclusionElement->HasAttribute("useGPUID")) {
                    if(!Utilities::stringToType<int>(inclusionSettings.m_UseGPUDeviceId, inclusionElement->GetAttribute("useGPUID"))) {
                        throw ticpp::Exception("---> String conversion in SceneSettings: useGPU failed");
                    }
                    if(inclusionSettings.m_UseGPUDeviceId <0) {
                        inclusionSettings.m_UseGPUDeviceId = 0;
                    }
                }

            }
            //Write all values back
            m_pDynSys->setSettings(timestepperSettings,inclusionSettings);

            // OutputParameters
            RecorderSettings recorderSettings; // Fills in standart values
            ticpp::Node *node = sceneSettings->FirstChild("RecorderSettings",false);
            if(node) {
                ticpp::Element *elem = node->ToElement();
                std::string method = elem->GetAttribute("recorderMode");
                if(method == "everyTimeStep") {
                    recorderSettings.setMode(RecorderSettings::RECORD_EVERY_STEP);
                } else if (method == "everyXTimeStep") {
                    recorderSettings.setMode(RecorderSettings::RECORD_EVERY_X_STEP);
                    PREC fps;
                    if(!Utilities::stringToType<double>(fps, elem->GetAttribute("statesPerSecond"))) {
                        throw ticpp::Exception("---> String conversion in RecorderSettings: statesPerSecond failed");
                    }
                    recorderSettings.setEveryXTimestep(fps,timestepperSettings.m_deltaT);
                } else if (method == "noOutput" || method=="none" || method=="nothing") {
                    recorderSettings.setMode(RecorderSettings::RECORD_NOTHING);
                } else {
                    throw ticpp::Exception("---> String conversion in RecorderSettings: recorderMode failed: not a valid setting");
                }
            }

            m_pDynSys->setSettings(recorderSettings);

            // Parse ContactParameter Map
            processContactParameterMap(sceneSettings);


        } // m_bparseDynamics

        // Parse external forces
        if(m_bParseDynamics) {
            processExternalForces(sceneSettings);
        }

        // Parse Global Geometries (also in playback manager)!
        processGlobalGeometries(sceneSettings);

    }

    virtual void processSceneSettings2( ticpp::Node *sceneSettings ) {

        LOG(m_pSimulationLog,"---> Process SceneSettings2..."<<std::endl;);

        if(m_bParseDynamics) {
            LOG(m_pSimulationLog,"---> Process GlobalInitialCondition..."<<std::endl;);
            processGlobalInitialCondition(sceneSettings);
        }
    }

    virtual void processContactParameterMap(ticpp::Node *sceneSettings) {


        ticpp::Node *paramMap = sceneSettings->FirstChild("ContactParameterMap",false);


        if(paramMap) {
            LOG(m_pSimulationLog,"---> Process ContactParameterMap..."<<std::endl;);
            typename RigidBodyType::BodyMaterialType material1,material2;
            PREC mu,epsilonN,epsilonT;

            ticpp::Element * element = paramMap->FirstChild("ContactParameterStandard",false)->ToElement();
            if(element) {

                LOG(m_pSimulationLog,"---> Add ContactParameterStandard..."<<std::endl;);
                if(!Utilities::stringToType<PREC>(mu, element->GetAttribute("mu"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameterStandard: mu failed");
                }
                if(!Utilities::stringToType<PREC>(epsilonN, element->GetAttribute("epsilonN"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameterStandard: epsilonN failed");
                }
                if(!Utilities::stringToType<PREC>(epsilonT, element->GetAttribute("epsilonT"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameterStandard: epsilonT failed");
                }
                ContactParams params(epsilonN,epsilonT,mu);

                m_pDynSys->m_ContactParameterMap.setStandardValues(params);
            }


            ticpp::Iterator< ticpp::Element > valueElem("ContactParameter");
            for ( valueElem = valueElem.begin( paramMap->ToElement() ); valueElem != valueElem.end(); valueElem++) {

                if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(material1, valueElem->GetAttribute("materialId1"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: materialId1 failed");
                }
                if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(material2, valueElem->GetAttribute("materialId2"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: materialId2 failed");
                }
                if(!Utilities::stringToType<PREC>(mu, valueElem->GetAttribute("mu"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: mu failed");
                }
                if(!Utilities::stringToType<PREC>(epsilonN, valueElem->GetAttribute("epsilonN"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: epsilonN failed");
                }
                if(!Utilities::stringToType<PREC>(epsilonT, valueElem->GetAttribute("epsilonT"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: epsilonT failed");
                }
                ContactParams params(epsilonN,epsilonT,mu);

                LOG(m_pSimulationLog,"---> Add ContactParameter of id="<<material1<<" to id="<<material2<<std::endl;);
                if(!m_pDynSys->m_ContactParameterMap.addContactParameter(material1,material2,params)) {
                    throw ticpp::Exception("---> Add ContactParameter failed");
                }

            }

        }


    }

    virtual void processGlobalGeometries(ticpp::Node *sceneSettings) {

        ticpp::Node *globalGeom = sceneSettings->FirstChild("GlobalGeometries",false);
        if(globalGeom) {
            ticpp::Iterator< ticpp::Node > child;
            for ( child = child.begin( globalGeom ); child != child.end(); child++ ) {

                if( child->Value() == "Geometry") {
                    processGeometry( &(*child) , true);
                }
            }
        }
    }

    virtual void processExternalForces( ticpp::Node *sceneSettings ) {
        ticpp::Node *externalForces = sceneSettings->FirstChild("ExternalForces",false);
        if(externalForces ) {
            LOG(m_pSimulationLog,"---> Process ExternalForces ..."<<std::endl;);
            ticpp::Iterator< ticpp::Node > child;
            for ( child = child.begin( externalForces ); child != child.end(); child++ ) {
                if( child->Value() == "ForceField") {
                    ticpp::Element* elem = (&(*child))->ToElement();
                    processForceField( elem );
                }
            }
        }
    }

    void processForceField( ticpp::Element * forceField) {

        bool enabled = false;
        if(!Utilities::stringToType<bool>(enabled, forceField->GetAttribute("enabled"))) {
            throw ticpp::Exception("---> String conversion in processForceField: enable failed");
        }
        if(enabled) {

            std::string apply  = forceField->GetAttribute("applyTo");

            std::vector<RigidBodyIdType > applyList;
            if( !(apply=="all" || apply=="All" || apply=="ALL" )) {
                //process all applyTo bodies
            } else if (apply=="all" || apply=="All" || apply=="ALL" ) {
                // do nothing
            } else {
                throw ticpp::Exception("---> String conversion in processForceField: applyTo failed");
            }

            std::string type = forceField->GetAttribute("type");
            if(type == "spatialspherical-timerandom") {

                unsigned int seed;
                if(!Utilities::stringToType<unsigned int>(seed, forceField->GetAttribute("seed"))) {
                    throw ticpp::Exception("---> String conversion in processForceField: seed failed");
                }
                PREC boostTime;
                if(!Utilities::stringToType<PREC>(boostTime, forceField->GetAttribute("boostTime"))) {
                    throw ticpp::Exception("---> String conversion in processForceField: boostTime failed");
                }
                PREC pauseTime;
                if(!Utilities::stringToType<PREC>(pauseTime, forceField->GetAttribute("pauseTime"))) {
                    throw ticpp::Exception("---> String conversion in processForceField: pauseTime failed");
                }

                PREC startTime;
                if(!Utilities::stringToType<PREC>(startTime, forceField->GetAttribute("startTime"))) {
                    throw ticpp::Exception("---> String conversion in processForceField: startTime failed");
                }

                PREC endTime;
                if(!Utilities::stringToType<PREC>(endTime, forceField->GetAttribute("endTime"))) {
                    throw ticpp::Exception("---> String conversion in processForceField: endTime failed");
                }
                PREC amplitude;
                if(!Utilities::stringToType<PREC>(amplitude, forceField->GetAttribute("amplitude"))) {
                    throw ticpp::Exception("---> String conversion in processForceField: amplitude failed");
                }

                Vector3 boxMin;
                if(!Utilities::stringToVector3<PREC>(boxMin, forceField->GetAttribute("minPoint"))) {
                    throw ticpp::Exception("---> String conversion in processForceField: boxMin failed");
                }
                Vector3 boxMax;
                if(!Utilities::stringToVector3<PREC>(boxMax, forceField->GetAttribute("maxPoint"))) {
                    throw ticpp::Exception("---> String conversion in processForceField: boxMax failed");
                }

                bool randomOn;
                if(!Utilities::stringToType<bool>(randomOn, forceField->GetAttribute("randomOn"))) {
                    throw ticpp::Exception("---> String conversion in processForceField: randomOn failed");
                }


                m_pDynSys->m_externalForces.addExternalForceCalculation(
                    new SpatialSphericalTimeRandomForceField(
                        seed,
                        boostTime,
                        pauseTime,
                        startTime,
                        endTime,
                        amplitude,
                        AABB(boxMin,boxMax),
                        randomOn
                    )
                );
            } else {
                throw ticpp::Exception("---> String conversion in processForceField: applyTo failed");
            }
        }
    }

    virtual void processGlobalInitialCondition( ticpp::Node *sceneSettings) {
        ticpp::Node *initCond = sceneSettings->FirstChild("GlobalInitialCondition",false);
        if(initCond) {
            ticpp::Element *elem = initCond->ToElement();

            bool enabled = false;
            if(!Utilities::stringToType<bool>(enabled, elem->GetAttribute("enabled"))) {
                throw ticpp::Exception("---> String conversion in GlobalInitialCondition: enable failed");
            }
            if(enabled) {
                double time = -1;
                short which = 2;
                std::string str = elem->GetAttribute("whichState");
                if( str == "end" || str == "END" || str== "End") {
                    which = 2;
                } else if(str == "beg" || str == "begin" || str== "BEG" || str == "Beg") {
                    which = 0;
                } else if(str == "time" || str =="TIME") {
                    which = 1;
                    if(!Utilities::stringToType<double>(time, elem->GetAttribute("time"))) {
                        throw ticpp::Exception("---> String conversion in GlobalInitialCondition: time failed");
                    }
                } else {
                    throw ticpp::Exception("---> String conversion in GlobalInitialCondition: whichState failed");
                }

                boost::filesystem::path relpath = elem->GetAttribute("relpath");


                setupInitialConditionBodiesFromFile_imp(relpath, time, which);

                bool useTime = false;
                if(!Utilities::stringToType<bool>(useTime, elem->GetAttribute("useTimeToContinue"))) {
                    throw ticpp::Exception("---> String conversion in GlobalInitialCondition: useTimeToContinue failed");
                }

                // Set the time in the dynamics system timestepper settings
                if(useTime){
                    TimeStepperSettings setTime;
                    m_pDynSys->getSettings(setTime);
                    setTime.m_startTime = time;
                    m_pDynSys->setSettings(setTime);
                }


            }

        }
    }

    virtual void setupInitialConditionBodiesFromFile_imp(boost::filesystem::path relpath, double &time , short which ){

        InitialConditionBodies::setupInitialConditionBodiesFromFile(relpath,m_pDynSys->m_simBodiesInitStates,time,true,true,which);
        LOG(m_pSimulationLog,"---> Found time: "<< time << " in " << relpath << std::endl;);

        m_pDynSys->applyInitStatesToBodies();

    }


    virtual void processSceneObjects( ticpp::Node *sceneObjects) {

        LOG(m_pSimulationLog,"---> Process SceneObjects ..."<<std::endl;);

        ticpp::Iterator< ticpp::Node > child;

        for ( child = child.begin( sceneObjects ); child != child.end(); child++ ) {

            if( child->Value() == "RigidBodies") {
                processRigidBodies( &(*child) );
            }

        }

        if( m_nSimBodies == 0) {
            throw ticpp::Exception("---> The scene in the XML contains no simulating bodies!");
        }

    }

    virtual void processRigidBodies( ticpp::Node * rigidbodies ) {

        LOG(m_pSimulationLog,"---> Process RigidBodies ..."<<std::endl;);
        ticpp::Element* rigidBodiesEl = rigidbodies->ToElement();

        //Clear current body list;
        m_initStatesGroup.clear();
        m_bodyListGroup.clear();
        m_bodyScalesGroup.clear();

        unsigned int instances = rigidbodies->ToElement()->GetAttribute<unsigned int>("instances");

        unsigned int groupId, startIdx;
        if(rigidBodiesEl->HasAttribute("groupId")) {
            m_globalMaxGroupId++; // Goes one up!
            groupId = rigidBodiesEl->GetAttribute<unsigned int>("groupId");
            m_globalMaxGroupId = groupId = std::max(m_globalMaxGroupId,groupId);

        } else {
            m_globalMaxGroupId++;
            groupId = m_globalMaxGroupId;
        }

        // Get the startidx for this group
        auto it = groupIdToNBodies.find(groupId);
        if( it == groupIdToNBodies.end()) {
            groupIdToNBodies[groupId] = 0;
        }
        startIdx = groupIdToNBodies[groupId];
        // update the number of bodies
        groupIdToNBodies[groupId] += instances;


        for(int i=0; i<instances; i++) {

            typename RigidBodyId::Type id = RigidBodyId::makeId(startIdx+i, groupId);

            RigidBodyType * temp_ptr = new RigidBodyType(id);

            //Assign a unique id

            LOG(m_pSimulationLog,"---> Added RigidBody Instance: "<<RigidBodyId::getBodyIdString(temp_ptr)<<std::endl);
            m_bodyListGroup.push_back(temp_ptr);

            Vector3 scale;
            scale.setOnes();
            m_bodyScalesGroup.push_back(scale);
        }
        LOG(m_pSimulationLog,"---> Added "<<instances<<" RigidBody Instances..."<<std::endl;);



        ticpp::Node * geometryNode = rigidbodies->FirstChild("Geometry");
        processGeometry(geometryNode);



        ticpp::Node * dynPropNode = rigidbodies->FirstChild("DynamicProperties");
        processDynamicProperties(dynPropNode);



        //Copy the pointers!

        if(m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {
            if(m_bParseDynamics) {
                LOG(m_pSimulationLog,"---> Copy Simulated RigidBody References to DynamicSystem ..."<<std::endl;);

                if(! m_pDynSys->m_SimBodies.addBodies(m_bodyListGroup.begin(),m_bodyListGroup.end())) {
                    ERRORMSG("Could not add body to m_SimBodies!, some bodies exist already in map!");
                };

                m_pDynSys->m_simBodiesInitStates.insert( m_initStatesGroup.begin(), m_initStatesGroup.end() );

            } else {
                m_initStates.insert( m_initStatesGroup.begin(), m_initStatesGroup.end() );
            }

            m_nSimBodies += instances;
            m_nBodies += instances;
        } else if(m_eBodiesState == RigidBodyType::BodyState::STATIC) {
            if(m_bParseDynamics) {
                LOG(m_pSimulationLog,"---> Copy Static RigidBody References to DynamicSystem ..."<<std::endl;);

                if(! m_pDynSys->m_Bodies.addBodies(m_bodyListGroup.begin(),m_bodyListGroup.end())) {
                    ERRORMSG("Could not add body to m_SimBodies!, some bodies exist already in map!");
                };

                //m_pDynSys->m_simBodiesInitStates.insert( m_initStatesGroup.begin(), m_initStatesGroup.end() );
            }
            m_nBodies += instances;
        } else {
            throw ticpp::Exception("---> Adding only simulated and not simulated objects supported!");
        }





        ticpp::Node * visualizationNode = rigidbodies->FirstChild("Visualization");
        processVisualization( visualizationNode);





    }

    virtual void processGeometry( ticpp::Node * geometryNode, bool addToGlobalGeoms = false) {
        LOG(m_pSimulationLog,"---> Process Geometry ..."<<std::endl;);
        if(geometryNode->FirstChild()->Value() == "Sphere") {
            processSphereGeometry( geometryNode->FirstChild()->ToElement(),  addToGlobalGeoms);
        } else if(geometryNode->FirstChild()->Value() == "Halfspace") {
            processHalfspaceGeometry( geometryNode->FirstChild()->ToElement(),  addToGlobalGeoms);
        } else if(geometryNode->FirstChild()->Value() == "Mesh") {
            processMeshGeometry( geometryNode->FirstChild()->ToElement(),  addToGlobalGeoms);
        } else if(geometryNode->FirstChild()->Value() == "Box") {
            processBoxGeometry( geometryNode->FirstChild()->ToElement(),  addToGlobalGeoms);
        } else if(geometryNode->FirstChild()->Value() == "GlobalGeomId") {
            processGlobalGeomId(geometryNode->FirstChild()->ToElement());
        } else {
            throw ticpp::Exception("---> The geometry '" + std::string(geometryNode->FirstChild()->Value()) + std::string("' has no implementation in the parser"));
        }
    }

    virtual void processSphereGeometry( ticpp::Element * sphere, bool addToGlobalGeoms = false) {
        std::string type = sphere->GetAttribute("distribute");
        if(type == "uniform") {

            double radius = sphere->GetAttribute<double>("radius");
            Vector3 scale;
            scale(0)=radius;
            scale(1)=radius;
            scale(2)=radius;

            boost::shared_ptr<SphereGeometry > pSphereGeom = boost::shared_ptr<SphereGeometry >(new SphereGeometry(radius));

            if(addToGlobalGeoms) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,sphere->GetAttribute("id"))) {
                    throw ticpp::Exception("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    throw ticpp::Exception("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                addToGlobalGeomList(id,pSphereGeom);
            } else {
                for(int i=0; i < m_bodyListGroup.size(); i++) {
                    m_bodyScalesGroup[i] = scale;
                    LOG(m_pSimulationLog, "\t---> Body id:" << m_bodyListGroup[i]->m_id << ", GeometryType: Sphere" << std::endl);
                    LOG(m_pSimulationLog, "\t\t---> radius: " << radius << std::endl; );
                    m_bodyListGroup[i]->m_geometry = pSphereGeom;
                }
            }
        } else if(type == "random") {
            double minRadius;
            if(!Utilities::stringToType<double>(minRadius,sphere->GetAttribute("minRadius"))) {
                throw ticpp::Exception("---> String conversion in processSphereGeometry: minRadius failed");
            }
            if( minRadius <= 0) {
                throw ticpp::Exception("---> In processSphereGeometry: minRadius to small!");
            }

            double maxRadius;
            if(!Utilities::stringToType<double>(maxRadius,sphere->GetAttribute("maxRadius"))) {
                throw ticpp::Exception("---> String conversion in processSphereGeometry: minRadius failed");
            }
            if( maxRadius <= minRadius) {
                throw ticpp::Exception("---> In processSphereGeometry: maxRadius smaller or equal to minRadius!");
            }

            unsigned int seed;
            if(!Utilities::stringToType<unsigned int>(seed,sphere->GetAttribute("seed"))) {
                throw ticpp::Exception("---> String conversion in processSphereGeometry: seed failed");
            }

            typedef boost::mt19937  RNG;
            RNG generator(seed);
            boost::uniform_real<PREC> uniform(minRadius,maxRadius);
            boost::variate_generator< boost::mt19937 & , boost::uniform_real<PREC> > randomNumber(generator, uniform);


            if(addToGlobalGeoms) {

                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,sphere->GetAttribute("id"))) {
                    throw ticpp::Exception("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    throw ticpp::Exception("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }

                unsigned int instances = 1;
                if(sphere->HasAttribute("instances")) {

                    if(!Utilities::stringToType<unsigned int>(instances,sphere->GetAttribute("instances"))) {
                        throw ticpp::Exception("---> String conversion in addToGlobalGeomList: instances failed");
                    }
                }

                for(int i = id; i < id + instances; i++) {
                    double radius = randomNumber();
                    boost::shared_ptr<SphereGeometry > pSphereGeom = boost::shared_ptr<SphereGeometry >(new SphereGeometry(radius));
                    addToGlobalGeomList(i, pSphereGeom);
                }
            } else {


                for(int i=0; i < m_bodyListGroup.size(); i++) {
                    double radius = randomNumber();
                    Vector3 scale;
                    scale(0)=radius;
                    scale(1)=radius;
                    scale(2)=radius;
                    m_bodyScalesGroup[i] = scale;
                    boost::shared_ptr<SphereGeometry > pSphereGeom = boost::shared_ptr<SphereGeometry >(new SphereGeometry(radius));
                    LOG(m_pSimulationLog, "\t---> Body id:" << m_bodyListGroup[i]->m_id << ", GeometryType: Sphere" << std::endl);
                    LOG(m_pSimulationLog, "\t\t---> radius: " << radius << std::endl; );
                    m_bodyListGroup[i]->m_geometry = pSphereGeom;

                }
            }



        } else {
            throw ticpp::Exception("---> The attribute 'distribute' '" + type + std::string("' of 'Sphere' has no implementation in the parser"));
        }


    }

    virtual void processHalfspaceGeometry( ticpp::Element * halfspace, bool addToGlobalGeoms = false) {
        std::string type = halfspace->GetAttribute("distribute");
        if(type == "uniform") {

            Vector3 n;
            if(!Utilities::stringToVector3<PREC>(n, halfspace->GetAttribute("normal"))) {
                throw ticpp::Exception("---> String conversion in HalfsphereGeometry: normal failed");
            }

            Vector3 p;
            if(!Utilities::stringToVector3<PREC>(p, halfspace->GetAttribute("position"))) {
                throw ticpp::Exception("---> String conversion in HalfsphereGeometry: position failed");
            }

            boost::shared_ptr<HalfspaceGeometry > pHalfspaceGeom = boost::shared_ptr<HalfspaceGeometry >(new HalfspaceGeometry(n,p));

            if(addToGlobalGeoms) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,halfspace->GetAttribute("id"))) {
                    throw ticpp::Exception("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    throw ticpp::Exception("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                addToGlobalGeomList(id, pHalfspaceGeom);
            } else {
                for(int i=0; i < m_bodyListGroup.size(); i++) {
                    m_bodyListGroup[i]->m_geometry = pHalfspaceGeom;
                }
            }

        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'Sphere' has no implementation in the parser"));
        }
    }

    virtual void processBoxGeometry( ticpp::Element * box, bool addToGlobalGeoms = false) {
        std::string type = box->GetAttribute("distribute");
        if(type == "uniform") {

            Vector3 extent;
            if(!Utilities::stringToVector3<PREC>(extent, box->GetAttribute("extent"))) {
                throw ticpp::Exception("---> String conversion in BoxGeometry: extent failed");
            }

            Vector3 center;
            if(!Utilities::stringToVector3<PREC>(center, box->GetAttribute("center"))) {
                throw ticpp::Exception("---> String conversion in BoxGeometry: position failed");
            }

            boost::shared_ptr<BoxGeometry > pBoxGeom = boost::shared_ptr<BoxGeometry >(new BoxGeometry(center,extent));

            Vector3 scale;
            scale(0)=extent(0);
            scale(1)=extent(1);
            scale(2)=extent(2);

            if(addToGlobalGeoms) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,box->GetAttribute("id"))) {
                    throw ticpp::Exception("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    throw ticpp::Exception("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                addToGlobalGeomList(id, pBoxGeom);
            } else {
                for(int i=0; i < m_bodyListGroup.size(); i++) {
                    m_bodyScalesGroup[i] = scale;
                    m_bodyListGroup[i]->m_geometry = pBoxGeom;
                }
            }

        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'Box' has no implementation in the parser"));
        }
    }


    virtual void processMeshGeometry( ticpp::Element * mesh, bool addToGlobalGeoms = false) {

        boost::shared_ptr<MeshGeometry > pMeshGeom;

        std::string meshName = mesh->GetAttribute<std::string>("name");

        bool bInstantiate;
        if(!Utilities::stringToType<bool>(bInstantiate,mesh->GetAttribute("useInstance"))) {
            throw ticpp::Exception("---> String conversion in processMeshGeometry: useInstance failed");
        }

        std::string type = mesh->GetAttribute("distribute");
        if(type == "uniform") {

            // import here an object model;
            // First make an meshinformatino structure

            boost::filesystem::path fileName =  mesh->GetAttribute<std::string>("file");
            checkFileExists(fileName);

            Vector3 scale_factor;
            if(!Utilities::stringToVector3<PREC>(scale_factor, mesh->GetAttribute("scale"))) {
                throw ticpp::Exception("---> String conversion in processMeshGeometry failed: scale");
            }
            if(scale_factor.norm()==0) {
                throw ticpp::Exception("---> Wrong scale factor (=0) specified in processMeshGeometry!");
            }

            Vector3 trans;
            if(!Utilities::stringToVector3<PREC>(trans, mesh->GetAttribute("translation"))) {
                throw ticpp::Exception("---> String conversion in processMeshGeometry: translation failed: ");
            }

            Vector3 axis;
            if(!Utilities::stringToVector3<PREC>(axis, mesh->GetAttribute("rotationAxis"))) {
                throw ticpp::Exception("---> String conversion in processMeshGeometry: rotationAxis failed");
            }

            PREC angle;

            if(mesh->HasAttribute("angleDegree")) {
                if(!Utilities::stringToType<PREC>(angle, mesh->GetAttribute("angleDegree"))) {
                    throw ticpp::Exception("---> String conversion in processMeshGeometry: angleDegree failed");
                }
                angle = angle / 180 * M_PI;
            } else if(mesh->HasAttribute("angleRadian")) {
                if(!Utilities::stringToType<PREC>(angle, mesh->GetAttribute("angleRadian"))) {
                    throw ticpp::Exception("---> String conversion in processMeshGeometry: angleRadian  failed");
                }
            } else {
                throw ticpp::Exception("---> No angle found in processMeshGeometry");
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
                                   aiProcess_Triangulate /*| aiProcess_GenNormals*/);

            // If the import failed, report it
            if(!scene) {
                throw ticpp::Exception("---> File import failed in processMeshGeometry: for file" + fileName.string() );
            }

            MeshData * meshData = new MeshData();

            if(!meshData->setup(importer,scene, scale_factor,quat,trans)) {
                throw ticpp::Exception("---> Imported Mesh (with Assimp) could not be setup internally");
            }

            // Build Geometry
            pMeshGeom = boost::shared_ptr<MeshGeometry >(new MeshGeometry(meshData));

            if(mesh->HasAttribute("writeToLog")) {
                bool writeToLog;
                if(!Utilities::stringToType<bool>(writeToLog, mesh->GetAttribute("writeToLog"))) {
                    throw ticpp::Exception("---> String conversion in processMeshGeometry: angleDegree failed");
                }
                if(writeToLog) {
                    meshData->writeToLog(fileName.string(), m_pSimulationLog);
                }
            }



            // Assign Geometry
            if(addToGlobalGeoms) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,mesh->GetAttribute("id"))) {
                    throw ticpp::Exception("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    throw ticpp::Exception("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                addToGlobalGeomList(id, pMeshGeom);
            } else {
                for(int i=0; i < m_bodyListGroup.size(); i++) {
                    m_bodyScalesGroup[i] = scale_factor;
                    m_bodyListGroup[i]->m_geometry = pMeshGeom;
                }
            }

        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'Mesh' has no implementation in the parser"));
        }
    }

    virtual void processGlobalGeomId( ticpp::Element * globalGeomId ) {

        std::string distribute = globalGeomId->GetAttribute("distribute");
        if(distribute == "uniform") {

            unsigned int id;
            if(!Utilities::stringToType<unsigned int>(id,globalGeomId->GetAttribute("id"))) {
                throw ticpp::Exception("---> String conversion in processGlobalGeomId: id failed");
            }

            typename DynamicsSystemType::GlobalGeometryMapType::iterator it = findGlobalGeomId(id);
            // it->second is the GeometryType in RigidBody
            if(it == this->getGlobalGeometryListRef().end()) {
                LOG(m_pSimulationLog,"---> Geometry with id: " << id << " not found in global geometry list!" <<std::endl;);
                throw ticpp::Exception("---> Geometry search in processGlobalGeomId: failed!");
            }

            for(int i=0; i < m_bodyListGroup.size(); i++) {
                GetScaleOfGeomVisitor vis(m_bodyScalesGroup[i]);
                boost::apply_visitor(vis, it->second);
                m_bodyListGroup[i]->m_geometry = it->second;
                m_bodyListGroup[i]->m_globalGeomId = id;
                LOG(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(m_bodyListGroup[i]) << ", GlobalGeomId: " << id <<  std::endl);
            }

        } else if(distribute == "linear") {

            unsigned int startId;
            if(!Utilities::stringToType<unsigned int>(startId,globalGeomId->GetAttribute("startId"))) {
                throw ticpp::Exception("---> String conversion in processGlobalGeomId: startId failed");
            }

            if(startId == 0) {
                throw ticpp::Exception("---> processGlobalGeomId: a global geometry startId: 0 is not allowed!");
                // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
            }

            for(int i=0; i < m_bodyListGroup.size(); i++) {
                int id = startId+i;
                typename DynamicsSystemType::GlobalGeometryMapType::iterator it = findGlobalGeomId(startId+i);
                // it->second is the GeometryType in RigidBody
                if(it == this->getGlobalGeometryListRef().end()) {
                    LOG(m_pSimulationLog,"---> processGlobalGeomId: Geometry with id: " << startId+i << " not found in global geometry list!" <<std::endl;);
                    throw ticpp::Exception("---> processGlobalGeomId: Geometry search failed!");
                }

                GetScaleOfGeomVisitor vis(m_bodyScalesGroup[i]);
                boost::apply_visitor(vis, it->second);
                m_bodyListGroup[i]->m_geometry = it->second;
                m_bodyListGroup[i]->m_globalGeomId = id;
                LOG(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(m_bodyListGroup[i]) << ", GlobalGeomId: " << id <<  std::endl);

            }


        } else if(distribute == "random") {

            unsigned int startId;
            if(!Utilities::stringToType<unsigned int>(startId,globalGeomId->GetAttribute("startId"))) {
                throw ticpp::Exception("---> String conversion in processGlobalGeomId: startId failed");
            }

            if(startId == 0) {
                throw ticpp::Exception("---> processGlobalGeomId: a global geometry startId: 0 is not allowed!");
                // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
            }

            unsigned int endId;
            if(!Utilities::stringToType<unsigned int>(endId,globalGeomId->GetAttribute("endId"))) {
                throw ticpp::Exception("---> String conversion in processGlobalGeomId: endId failed");
            }
            if(startId > endId) {
                throw ticpp::Exception("---> addToGlobalGeomList:  startId > endId  is not allowed!");
                // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
            }
            unsigned int seed;
            if(!Utilities::stringToType<unsigned int>(seed,globalGeomId->GetAttribute("seed"))) {
                throw ticpp::Exception("---> String conversion in processGlobalGeomId: seed failed");
            }

            typedef boost::mt19937  RNG;
            RNG generator(seed);
            boost::uniform_int<unsigned int> uniform(startId,endId);
            boost::variate_generator< boost::mt19937 & , boost::uniform_int<unsigned int> > randomNumber(generator, uniform);

            for(int i=0; i < m_bodyListGroup.size(); i++) {

                unsigned int id = randomNumber();
                typename DynamicsSystemType::GlobalGeometryMapType::iterator it = findGlobalGeomId(id);
                // it->second is the GeometryType in RigidBody
                if(it == this->getGlobalGeometryListRef().end()) {
                    LOG(m_pSimulationLog,"---> Geometry with id: " << id << " not found in global geometry list!" <<std::endl;);
                    throw ticpp::Exception("---> Geometry search in processGlobalGeomId: failed!");
                }

                GetScaleOfGeomVisitor vis(m_bodyScalesGroup[i]);
                boost::apply_visitor(vis, it->second);
                m_bodyListGroup[i]->m_geometry = it->second;
                m_bodyListGroup[i]->m_globalGeomId = id;
                LOG(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(m_bodyListGroup[i]) << ", GlobalGeomId: " << id <<  std::endl);
            }


        } else {
            throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'GlobalGeomId' has no implementation in the parser"));
        }



    }


    virtual typename DynamicsSystemType::GlobalGeometryMapType::iterator findGlobalGeomId(unsigned int id) {
        return m_pDynSys->m_globalGeometries.find(id);
    }


    template<typename T>
    void addToGlobalGeomList(unsigned int id,  boost::shared_ptr<T> ptr) {



        std::pair<typename DynamicsSystemType::GlobalGeometryMapType::iterator, bool> ret =
            this->getGlobalGeometryListRef().insert(typename DynamicsSystemType::GlobalGeometryMapType::value_type( id, ptr) );
        if(ret.second == false) {
            std::stringstream ss;
            ss << "---> addToGlobalGeomList: geometry with id: " <<  id<< " exists already!";
            throw ticpp::Exception(ss.str());
        }
        LOG(m_pSimulationLog,"---> Added geometry with id: " <<  id << " to global geometry list" <<std::endl;);

        // Print some details:
        LOG(m_pSimulationLog,"\t---> GlobalGeomId: " << id <<std::endl);
        PrintGeometryDetailsVisitor(m_pSimulationLog, ret.first->second, "\t\t--->");


    }

    virtual typename DynamicsSystemType::GlobalGeometryMapType & getGlobalGeometryListRef() {
        return m_pDynSys->m_globalGeometries;
    }

    virtual void fillMeshInfo( Assimp::Importer & importer, const aiScene* scene, MeshData & meshInfo, Vector3 scale_factor, Quaternion quat, Vector3 trans) {



    }

    virtual void checkFileExists(boost::filesystem::path file) {
        if( !boost::filesystem::exists(file) ) {
            throw ticpp::Exception("---> The file ' " + file.string() + "' does not exist!");
        }
    }

    virtual void processDynamicProperties( ticpp::Node * dynProp) {
        LOG(m_pSimulationLog,"---> Process DynamicProperties ..."<<std::endl;);
        ticpp::Element * element = dynProp->FirstChild("DynamicState")->ToElement();



        std::string type = element->GetAttribute("type");
        if(type == "simulated") {
            m_eBodiesState =  RigidBodyType::BodyState::SIMULATED;
        } else if(type == "not simulated") {
            m_eBodiesState =  RigidBodyType::BodyState::STATIC;
        } else if(type == "animated") {
            m_eBodiesState =  RigidBodyType::BodyState::ANIMATED;
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
        }

        // apply first to all bodies :-)
        for(int i=0; i < m_bodyListGroup.size(); i++) {
            m_bodyListGroup[i]->m_eState = m_eBodiesState;
        }

        if(m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {
            processDynamicPropertiesSimulated(dynProp);
        } else if(m_eBodiesState == RigidBodyType::BodyState::STATIC) {
            processDynamicPropertiesNotSimulated(dynProp);
        }
    }


    virtual void processDynamicPropertiesSimulated( ticpp::Node * dynProp) {
        ticpp::Element *element = nullptr;
        std::string distribute;

        if(m_bParseDynamics) {
            // First allocate a new SolverDate structure
            for(int i=0; i < m_bodyListGroup.size(); i++) {
                m_bodyListGroup[i]->m_pSolverData = new RigidBodySolverDataType();
            }


            // Mass ============================================================
            element = dynProp->FirstChild("Mass")->ToElement();
            distribute = element->GetAttribute("distribute");
            if(distribute == "uniform") {

                double mass = element->GetAttribute<double>("value");

                for(int i=0; i < m_bodyListGroup.size(); i++) {
                    m_bodyListGroup[i]->m_mass = mass;
                }


            } else {
                throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'Mass' has no implementation in the parser"));
            }

            // InertiaTensor ============================================================
            element = dynProp->FirstChild("InertiaTensor")->ToElement();
            std::string type = element->GetAttribute("type");
            if(type == "homogen") {
                for(int i=0; i < m_bodyListGroup.size(); i++) {
                    InertiaTensor::calculateInertiaTensor(m_bodyListGroup[i]);
                }
            } else {
                throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'InertiaTensor' has no implementation in the parser"));
            }

            element = dynProp->FirstChild("Material")->ToElement();
            distribute = element->GetAttribute("distribute");
            if(distribute == "uniform") {
                typename RigidBodyType::BodyMaterialType eMaterial = 0;

                if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(eMaterial, element->GetAttribute("id"))) {
                    throw ticpp::Exception("---> String conversion in Material: id failed");
                }

                for(int i=0; i < m_bodyListGroup.size(); i++) {
                    m_bodyListGroup[i]->m_eMaterial = eMaterial;
                }
            } else {
                throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'Material' has no implementation in the parser"));
            }
        }

        // InitialPosition ============================================================
        ticpp::Node * node = dynProp->FirstChild("InitialCondition",true);
        processInitialCondition(node,true);
    }

    virtual void processInitialCondition(ticpp::Node * initCondNode, bool simBodies) {
        ticpp::Element *element = nullptr;
        std::string distribute;

        // Fill as many initial states as needed!
        m_initStatesGroup.clear();
        for(int i=0; i < m_bodyListGroup.size(); i++) {
            auto res = m_initStatesGroup.insert( std::make_pair(m_bodyListGroup[i]->m_id, RigidBodyState(m_bodyListGroup[i]->m_id) ) );
            if(!res.second) {
                std::stringstream s;
                s << "--->Initial State of body id: " << RigidBodyId::getBodyIdString(m_bodyListGroup[i]) << " has already been added!";
                throw ticpp::Exception(s.str());

            }
        }

        element = initCondNode->ToElement();
        std::string type  = element->GetAttribute("type");
        if( type == "file") {
            throw ticpp::Exception(" Not yet implemented");
        } else if (type == "posvel") {

            element = initCondNode->FirstChild("InitialPosition")->ToElement();
            distribute = element->GetAttribute("distribute");

            if(distribute == "linear") {
                processInitialPositionLinear(element);
            } else if(distribute == "grid") {
                processInitialPositionGrid(element);
            } else if(distribute == "posaxisangle") {
                processInitialPositionPosAxisAngle(element);
            } else if(distribute == "transforms") {
                processInitialPositionTransforms(element);
            } else if(distribute == "generalized") {
                //processInitialPositionGeneralized(element);
            } else if(distribute == "none") {
                // does nothing leaves the zero state pushed!
            } else {
                throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'InitialPosition' has no implementation in the parser"));
            }

            //Initial Velocity
            if(m_bParseDynamics || simBodies == false) {
                ticpp::Node * initVel = initCondNode->FirstChild("InitialVelocity",false);
                if(initVel) {
                    element = initVel->ToElement();
                    distribute = element->GetAttribute("distribute");

                    if(distribute == "transrot") {
                        processInitialVelocityTransRot(element);
                    } else if(distribute == "generalized") {
                        //processInitialVelocityGeneralized(element);
                    } else if(distribute == "none") {
                        // does nothing leaves the zero state pushed!
                    } else {
                        throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'InitialVelocity' has no implementation in the parser"));
                    }
                }
            }

        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'InitialCondition' has no implementation in the parser"));
        }

        InitialConditionBodies::applyRigidBodyStatesToBodies(m_bodyListGroup, m_initStatesGroup);
    }


    virtual void processDynamicPropertiesNotSimulated( ticpp::Node * dynProp) {

        // InitialPosition ============================================================
        ticpp::Node * node = dynProp->FirstChild("InitialCondition",true);
        processInitialCondition(node,false);


        ticpp::Element *element = nullptr;
        std::string distribute;

        if(m_bParseDynamics) {
            element = dynProp->FirstChild("Material")->ToElement();
            distribute = element->GetAttribute("distribute");
            if(distribute == "uniform") {
                typename RigidBodyType::BodyMaterialType eMaterial = 0;

                if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(eMaterial, element->GetAttribute("id"))) {
                    throw ticpp::Exception("---> String conversion in Material: id failed");
                }

                for(int i=0; i < m_bodyListGroup.size(); i++) {
                    m_bodyListGroup[i]->m_eMaterial = eMaterial;
                }
            } else {
                throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'Material' has no implementation in the parser"));
            }
        }

    }


    virtual void processInitialPositionLinear(ticpp::Element * initCond) {

        Vector3 pos;
        if(!Utilities::stringToVector3<PREC>(pos, initCond->GetAttribute("position"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionLinear: position Linear failed");
        }
        Vector3 dir;
        if(!Utilities::stringToVector3<PREC>(dir, initCond->GetAttribute("direction"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionLinear: direction Linear failed");
        }
        PREC dist;
        if(!Utilities::stringToType<PREC>(dist, initCond->GetAttribute("distance"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionLinear: distance  Linear failed");
        }
        bool jitter;
        if(!Utilities::stringToType(jitter, initCond->GetAttribute("jitter"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionLinear: jitter Linear failed");
        }

        PREC delta;
        if(!Utilities::stringToType<PREC>(delta, initCond->GetAttribute("delta"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionLinear: delta Linear failed");
        }

        unsigned int seed = 5;
        if(initCond->HasAttribute("seed")) {
            if(!Utilities::stringToType(seed, initCond->GetAttribute("seed"))) {
                throw ticpp::Exception("---> String conversion in InitialPositionGrid: jitter seed failed");
            }
        }

        InitialConditionBodies::setupPositionBodiesLinear(m_initStatesGroup,pos,dir,dist,jitter,delta,seed);

    }

    virtual void processInitialPositionGrid(ticpp::Element * initCond) {

        Vector3 trans;
        if(!Utilities::stringToVector3<PREC>(trans, initCond->GetAttribute("translation"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionGrid: translation failed");
        }
        int gridX;
        if(!Utilities::stringToType<int>(gridX, initCond->GetAttribute("gridSizeX"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionGrid: gridSizeX failed");
        }
        int gridY;
        if(!Utilities::stringToType<int>(gridY, initCond->GetAttribute("gridSizeY"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionGrid: gridSizeY failed");
        }
        PREC dist;
        if(!Utilities::stringToType<PREC>(dist, initCond->GetAttribute("distance"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionGrid: distance failed");
        }
        bool jitter;
        if(!Utilities::stringToType(jitter, initCond->GetAttribute("jitter"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionGrid: jitter failed");
        }
        int seed = 5;
        if(initCond->HasAttribute("seed")) {
            if(!Utilities::stringToType(seed, initCond->GetAttribute("seed"))) {
                throw ticpp::Exception("---> String conversion in InitialPositionGrid: jitter seed failed");
            }
        }
        double delta;
        if(!Utilities::stringToType<double>(delta, initCond->GetAttribute("delta"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionGrid: delta failed");
        }

        InitialConditionBodies::setupPositionBodiesGrid(m_initStatesGroup,gridX,gridY,dist,trans,jitter,delta, seed);
    }

//    virtual void processInitialPositionFile(DynamicsState & state, ticpp::Element * initCond) {
//        m_SimBodyInitStates.push_back(DynamicsState((unsigned int)m_bodyListGroup.size()));
//
//        boost::filesystem::path name =  initCond->GetAttribute<std::string>("relpath");
//
//        boost::filesystem::path filePath = m_currentParseFileDir / name;
//        InitialConditionBodies::setupPositionBodiesFromFile(state,filePath);
//    }

    virtual void processInitialPositionPosAxisAngle(ticpp::Element * initCond) {

        int bodyCounter = 0;

        Vector3 pos;
        Vector3 axis;
        PREC angle;

        // Iterate over all values in the list
        ticpp::Iterator< ticpp::Element > valueElem("Value");
        for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++) {

            if(bodyCounter >= m_bodyListGroup.size()) {
                LOG(m_pSimulationLog,"---> InitialPositionPosAxisAngle: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            }

            if(!Utilities::stringToVector3<PREC>(pos, valueElem->GetAttribute("position"))) {
                throw ticpp::Exception("---> String conversion in InitialPositionPosAxisAngle: position failed");
            }

            if(!Utilities::stringToVector3<PREC>(axis, valueElem->GetAttribute("axis"))) {
                throw ticpp::Exception("---> String conversion in InitialPositionPosAxisAngle: axis failed");
            }

            if( axis.norm() == 0) {
                throw ticpp::Exception("---> Specified wrong axis in InitialPositionPosAxisAngle");
            }


            if(valueElem->HasAttribute("angleDegree")) {
                if(!Utilities::stringToType<PREC>(angle, valueElem->GetAttribute("angleDegree"))) {
                    throw ticpp::Exception("---> String conversion in InitialPositionPosAxisAngle: angleDegree failed");
                }
                angle = angle / 180 * M_PI;
            } else if(valueElem->HasAttribute("angleRadian")) {
                if(!Utilities::stringToType<PREC>(angle, valueElem->GetAttribute("angleRadian"))) {
                    throw ticpp::Exception("---> String conversion in InitialPositionPosAxisAngle: angleRadian failed");
                }
            } else {
                throw ticpp::Exception("---> No angle found in InitialPositionPosAxisAngle");
            }


            InitialConditionBodies::setupPositionBodyPosAxisAngle( m_initStatesGroup[m_bodyListGroup[bodyCounter]->m_id],
                    pos, axis, angle);


            bodyCounter++;
        }

        if(bodyCounter < m_bodyListGroup.size()) {
            LOG(m_pSimulationLog,"---> InitialPositionPosAxisAngle: You specified to little values, -> applying last to all remainig bodies ..."<<std::endl;);
            for(int i=bodyCounter; i<m_bodyListGroup.size(); i++) {
                InitialConditionBodies::setupPositionBodyPosAxisAngle(m_initStatesGroup[m_bodyListGroup[i]->m_id], pos, axis, angle);
            }
        }
    }

    virtual void processInitialPositionTransforms(ticpp::Element * initCond) {



        int bodyCounter = 0;

        Quaternion q_KI, q_BK;
        Vector3 I_r_IK, K_r_KB;
        Matrix33 Rot_KI; // Temp

        // Iterate over all values in the list
        ticpp::Iterator< ticpp::Element > valueElem("Value");
        for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++) {

            if(bodyCounter >= m_bodyListGroup.size()) {
                LOG(m_pSimulationLog,"---> InitialPositionTransforms: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            }


            setQuaternionZero(q_KI);
            I_r_IK.setZero();


            // Iterate over all transforms an successfully applying the total trasnformation!
            ticpp::Iterator< ticpp::Element > transformElem("Transform");
            for ( transformElem = transformElem.begin( &(*valueElem) ); transformElem != transformElem.end(); transformElem++) {

                Vector3 trans;
                if(!Utilities::stringToVector3<PREC>(trans, transformElem->GetAttribute("translation"))) {
                    throw ticpp::Exception("---> String conversion in InitialPositionTransforms: translation failed");
                }
                Vector3 axis;
                if(!Utilities::stringToVector3<PREC>(axis, transformElem->GetAttribute("rotationAxis"))) {
                    throw ticpp::Exception("---> String conversion in InitialPositionTransforms: rotationAxis failed");
                }

                if( axis.norm() == 0) {
                    throw ticpp::Exception("---> Specified wrong axis in InitialPositionTransforms");
                }

                PREC angle;
                if(transformElem->HasAttribute("angleDegree")) {
                    if(!Utilities::stringToType<PREC>(angle, transformElem->GetAttribute("angleDegree"))) {
                        throw ticpp::Exception("---> String conversion in InitialPositionTransforms: angleDegree failed");
                    }
                    angle = angle / 180 * M_PI;
                } else if(transformElem->HasAttribute("angleRadian")) {
                    if(!Utilities::stringToType<PREC>(angle, transformElem->GetAttribute("angleRadian"))) {
                        throw ticpp::Exception("---> String conversion in InitialPositionTransforms: angleRadian failed");
                    }
                } else {
                    throw ticpp::Exception("---> No angle found in InitialPositionTransforms");
                }

                setQuaternion(q_BK,axis,angle);
                K_r_KB = trans;
                Rot_KI = getRotFromQuaternion<PREC>(q_KI);
                I_r_IK += Rot_KI * K_r_KB; // Transforms like A_IK * A_r_AB;
                q_KI = quatMult(q_KI,q_BK); // Sequential (aktiv) rotation

            }

            // Apply overall transformation!
            auto & state = m_initStatesGroup[m_bodyListGroup[bodyCounter]->m_id];
            state.m_q.head<3>() = I_r_IK;
            state.m_q.tail<4>() = q_KI;

            bodyCounter++;
        }

        if(bodyCounter < m_bodyListGroup.size()) {
            LOG(m_pSimulationLog,"---> InitialPositionTransforms: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
            for(int i=bodyCounter; i<m_bodyListGroup.size(); i++) {
                auto & state = m_initStatesGroup[m_bodyListGroup[i]->m_id];
                state.m_q.head<3>() = I_r_IK;
                state.m_q.tail<4>() = q_KI;
            }
        }

    }

    virtual void processInitialVelocityTransRot(ticpp::Element * initCond) {


        int bodyCounter = 0;
        Vector3 transDir,rotDir;
        PREC rot,vel;

        // Iterate over all values in the list
        ticpp::Iterator< ticpp::Element > valueElem("Value");
        for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++) {

            if(bodyCounter >= m_bodyListGroup.size()) {
                LOG(m_pSimulationLog,"---> InitialVelocityTransRot: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            }


            if(!Utilities::stringToVector3<PREC>(transDir, valueElem->GetAttribute("transDir"))) {
                throw ticpp::Exception("---> String conversion in InitialVelocityTransRot: transDir failed");
            }
            transDir.normalize();

            if(!Utilities::stringToType<PREC>(vel, valueElem->GetAttribute("absTransVel"))) {
                throw ticpp::Exception("---> String conversion in InitialVelocityTransRot: absTransVel failed");
            }

            if(!Utilities::stringToVector3<PREC>(rotDir, valueElem->GetAttribute("rotDir"))) {
                throw ticpp::Exception("---> String conversion in InitialVelocityTransRot: transDir failed");
            }
            rotDir.normalize();


            if(!Utilities::stringToType<PREC>(rot, valueElem->GetAttribute("absRotVel"))) {
                throw ticpp::Exception("---> String conversion in InitialVelocityTransRot: absTransVel failed");
            }
            auto & state = m_initStatesGroup[m_bodyListGroup[bodyCounter]->m_id];
            state.m_u.head<3>() = transDir*vel;
            state.m_u.tail<3>() = rotDir*rot;

            bodyCounter++;
        }

        if(bodyCounter < m_bodyListGroup.size()) {
            LOG(m_pSimulationLog,"---> InitialVelocityTransRot: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
            for(int i=bodyCounter; i<m_bodyListGroup.size(); i++) {
                auto & state = m_initStatesGroup[m_bodyListGroup[i]->m_id];
                state.m_u.head<3>() = transDir*vel;
                state.m_u.tail<3>() = rotDir*rot;
            }
        }

    }



    virtual void processVisualization( ticpp::Node * visualizationNode) {

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
    std::unordered_map<unsigned int,unsigned int> groupIdToNBodies;
    unsigned int m_globalMaxGroupId; // Group Id used to build a unique id!

    // Temprary structures for each sub list of rigid bodies
    typename RigidBodyType::BodyState m_eBodiesState; ///< Used to process a RigidBody Node
    typename std::vector<RigidBodyType*> m_bodyListGroup; ///< Used to process a RigidBody Node
    std::vector<Vector3> m_bodyScalesGroup;
    RigidBodyStatesContainerType m_initStatesGroup;


    RigidBodyStatesContainerType m_initStates; ///< Init states of sim bodies, if  m_bParseDynamics = false

    typedef std::unordered_map<std::string, boost::shared_ptr<MeshGeometry > > ContainerSceneMeshs;

};


#endif
