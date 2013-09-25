#ifndef SceneParser_hpp
#define SceneParser_hpp

#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "AssertionDebug.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/filesystem.hpp>
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "RigidBody.hpp"

#include "ContactParams.hpp"

#include "InclusionSolverSettings.hpp"
#include "TimeStepperSettings.hpp"
#include "RecorderSettings.hpp"

#include "MeshGeometry.hpp"

#include "CommonFunctions.hpp"
#include "QuaternionHelpers.hpp"
#include "InertiaTensorCalculations.hpp"
#include "InitialConditionBodies.hpp"

#include "ExternalForces.hpp"


#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>       // Output data structure
#include <assimp/postprocess.h> // Post processing flags

//#include "OgreMeshExtraction.hpp"

#define TIXML_USE_TICPP
#include "ticpp/ticpp.h"
//#include "tinyxml.h"


template<typename TDynamicsSystemConfig>
class GetScaleOfGeomVisitor : public boost::static_visitor<>{
    public:
    typedef TDynamicsSystemConfig DynamicsSystemConfig;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(TDynamicsSystemConfig)

    GetScaleOfGeomVisitor(Vector3 & scale): m_scale(scale){};

    void operator()(  boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom ){
        m_scale.setConstant(sphereGeom->m_radius);
    }
    void operator()(  boost::shared_ptr<const BoxGeometry<PREC> >  & boxGeom){
        m_scale = boxGeom->m_extent;
    }

    template<typename T>
    void operator()(  boost::shared_ptr<T>  & ptr){
        ERRORMSG("This GetScaleOfGeomVisitor visitor operator() has not been implemented!");
    }

    Vector3 & m_scale;
};

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

        m_pSimulationLog->logMessage("---> Parsing Scene...");
        LOG( m_pSimulationLog, "---> Scene Input file: "  << file.string() <<std::endl; );

        if(!boost::filesystem::exists(m_currentParseFilePath)){
            ERRORMSG("Scene Input file does not exist!");
        }


        //Reset all variables
        m_globalMaxGroupId = 0;
        m_nSimBodies = 0;
        m_nBodies = 0;
        m_bodyList.clear();
        m_SimBodyInitStates.clear();


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

                /*ticpp::Node * initialConditionAll = m_xmlRootNode->FirstChild("InitialPositionLinear");
                processinitialConditionAll(initialConditionAll);*/

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

    virtual void cleanUp(){

     m_bodyList.clear();
     m_bodyListScales.clear();
     m_SimBodyInitStates.clear();

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

        LOG(m_pSimulationLog,"---> Process SceneSettings..."<<std::endl;);

        if(m_bParseDynamics) {

            ticpp::Element *gravityElement = sceneSettings->FirstChild("Gravity",true)->ToElement();
            m_pDynSys->m_gravity = gravityElement->GetAttribute<double>("value");

            if(!Utilities::stringToVector3<PREC>(m_pDynSys->m_gravityDir , gravityElement->GetAttribute("direction"))) {
                throw ticpp::Exception("---> String conversion in SceneSettings: gravity failed");
            }

            ticpp::Element *timestepElement = sceneSettings->FirstChild("TimeStepperSettings",true)->ToElement();

            TimeStepperSettings<LayoutConfigType> timestepperSettings;
            InclusionSolverSettings<LayoutConfigType> inclusionSettings;

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
                        timestepperSettings.m_eSimulateFromReference = TimeStepperSettings<LayoutConfigType>::USE_STATES;
                    } else if(type == "continue") {
                        timestepperSettings.m_eSimulateFromReference = TimeStepperSettings<LayoutConfigType>::CONTINUE;
                    } else {
                        throw ticpp::Exception("---> String conversion in SimulateFromReference: type. The type '" + type + std::string("' has no implementation in the parser"));
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
                    throw ticpp::Exception("---> String conversion in SceneSettings: alphaJORProx failed");
                }
                if(!Utilities::stringToType<PREC>(inclusionSettings.m_alphaSORProx, inclusionElement->GetAttribute("alphaSORProx"))) {
                    throw ticpp::Exception("---> String conversion in SceneSettings: alphaJORProx failed");
                }
                if(!Utilities::stringToType<unsigned int>(inclusionSettings.m_MaxIter, inclusionElement->GetAttribute("maxIter"))) {
                    throw ticpp::Exception("---> String conversion in SceneSettings: maxIter failed");
                }

                if(inclusionElement->HasAttribute("minIter")){
                    if(!Utilities::stringToType<unsigned int>(inclusionSettings.m_MinIter, inclusionElement->GetAttribute("minIter"))) {
                        throw ticpp::Exception("---> String conversion in SceneSettings: minIter failed");
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
                        throw ticpp::Exception("---> String conversion in SceneSettings: convergenceMethod failed: not a valid setting");
                    }
                }else{
                    inclusionSettings.m_eConvergenceMethod = InclusionSolverSettings<LayoutConfigType>::InLambda;
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
                    inclusionSettings.m_eMethod = InclusionSolverSettings<LayoutConfigType>::JOR;
                } else if (method == "SOR") {
                    inclusionSettings.m_eMethod = InclusionSolverSettings<LayoutConfigType>::SOR;
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
            RecorderSettings<LayoutConfigType> recorderSettings; // Fills in standart values
            ticpp::Node *node = sceneSettings->FirstChild("RecorderSettings",false);
            if(node){
                ticpp::Element *elem = node->ToElement();
                std::string method = elem->GetAttribute("recorderMode");
                if(method == "everyTimeStep") {
                    recorderSettings.setMode(RecorderSettings<LayoutConfigType>::RECORD_EVERY_STEP);
                } else if (method == "everyXTimeStep") {
                    recorderSettings.setMode(RecorderSettings<LayoutConfigType>::RECORD_EVERY_X_STEP);
                    PREC fps;
                    if(!Utilities::stringToType<double>(fps, elem->GetAttribute("statesPerSecond"))) {
                        throw ticpp::Exception("---> String conversion in RecorderSettings: statesPerSecond failed");
                    }
                    recorderSettings.setEveryXTimestep(fps,timestepperSettings.m_deltaT);
                } else if (method == "noOutput" || method=="none" || method=="nothing") {
                    recorderSettings.setMode(RecorderSettings<LayoutConfigType>::RECORD_NOTHING);
                }
                else {
                    throw ticpp::Exception("---> String conversion in RecorderSettings: recorderMode failed: not a valid setting");
                }
            }

            m_pDynSys->setSettings(recorderSettings);

            // Parse ContactParameter Map
            processContactParameterMap(sceneSettings);


        } // m_bparseDynamics

        // Parse external forces
        if(m_bParseDynamics){
             processExternalForces(sceneSettings);
        }

        // Parse Global Geometries (also in playback manager)!
        processGlobalGeometries(sceneSettings);

    }

    virtual void processContactParameterMap(ticpp::Node *sceneSettings){


        ticpp::Node *paramMap = sceneSettings->FirstChild("ContactParameterMap",false);


        if(paramMap){
            LOG(m_pSimulationLog,"---> Process ContactParameterMap..."<<std::endl;);
            typename RigidBodyType::BodyMaterial material1,material2;
            PREC mu,epsilonN,epsilonT;

            ticpp::Element * element = paramMap->FirstChild("ContactParameterStandard",false)->ToElement();
            if(element){

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
                ContactParams<LayoutConfigType> params(epsilonN,epsilonT,mu);

                m_pDynSys->m_ContactParameterMap.setStandardValues(params);
            }


            ticpp::Iterator< ticpp::Element > valueElem("ContactParameter");
            for ( valueElem = valueElem.begin( paramMap->ToElement() ); valueElem != valueElem.end(); valueElem++) {

                if(!Utilities::stringToType<typename RigidBodyType::BodyMaterial>(material1, valueElem->GetAttribute("materialId1"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: materialId1 failed");
                }
                if(!Utilities::stringToType<typename RigidBodyType::BodyMaterial>(material2, valueElem->GetAttribute("materialId2"))) {
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
                ContactParams<LayoutConfigType> params(epsilonN,epsilonT,mu);

                LOG(m_pSimulationLog,"---> Add ContactParameter of id="<<material1<<" to id="<<material2<<std::endl;);
                if(!m_pDynSys->m_ContactParameterMap.addContactParameter(material1,material2,params)){
                   throw ticpp::Exception("---> Add ContactParameter failed");
                }

            }

        }


    }

    virtual void processGlobalGeometries(ticpp::Node *sceneSettings){

        ticpp::Node *globalGeom = sceneSettings->FirstChild("GlobalGeometries",false);
        if(globalGeom){
            ticpp::Iterator< ticpp::Node > child;
            for ( child = child.begin( globalGeom ); child != child.end(); child++ ) {

                if( child->Value() == "Geometry") {
                    processGeometry( &(*child) , true);
                }
            }
        }
    }

    virtual void processExternalForces( ticpp::Node *sceneSettings ){
        ticpp::Node *externalForces = sceneSettings->FirstChild("ExternalForces",false);
        if(externalForces ){
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

    void processForceField( ticpp::Element * forceField){

        bool enabled = false;
        if(!Utilities::stringToType<bool>(enabled, forceField->GetAttribute("enabled"))) {
            throw ticpp::Exception("---> String conversion in processForceField: enable failed");
        }
        if(enabled){

            std::string apply  = forceField->GetAttribute("applyTo");

            std::vector<typename RigidBodyType::RigidBodyIdType > applyList;
            if( !(apply=="all" || apply=="All" || apply=="ALL" )) {
                //process all applyTo bodies
            }else if (apply=="all" || apply=="All" || apply=="ALL" ) {
                // do nothing
            }
            else{
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
                                                    new SpatialSphericalTimeRandomForceField<DynamicsSystemType>(
                                                                seed,
                                                                boostTime,
                                                                pauseTime,
                                                                startTime,
                                                                endTime,
                                                                amplitude,
                                                                AABB<LayoutConfigType>(boxMin,boxMax),
                                                                randomOn
                                                                )
                                                    );
            }else{
                throw ticpp::Exception("---> String conversion in processForceField: applyTo failed");
            }
        }
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
        m_bodyList.clear();
        m_bodyListScales.clear();

        unsigned int instances = rigidbodies->ToElement()->GetAttribute<unsigned int>("instances");

        unsigned int groupId;
        if(rigidBodiesEl->HasAttribute("groupId")){
            m_globalMaxGroupId++; // Goes one up!
            groupId = rigidBodiesEl->GetAttribute<unsigned int>("groupId");
            m_globalMaxGroupId = groupId = std::max(m_globalMaxGroupId,groupId);
        }else{
            m_globalMaxGroupId++;
            groupId = m_globalMaxGroupId;
        }


        for(int i=0; i<instances; i++) {

            typename RigidBodyId::Type id = RigidBodyId::makeId(i, groupId);

            RigidBodyType * temp_ptr = new RigidBodyType(id);

            //Assign a unique id

            LOG(m_pSimulationLog,"---> Added RigidBody Instance: "<<RigidBodyId::getBodyIdString(temp_ptr)<<std::endl);
            m_bodyList.push_back(temp_ptr);

            Vector3 scale;
            scale.setOnes();
            m_bodyListScales.push_back(scale);
        }
        LOG(m_pSimulationLog,"---> Added "<<instances<<" RigidBody Instances..."<<std::endl;);



        ticpp::Node * geometryNode = rigidbodies->FirstChild("Geometry");
        processGeometry(geometryNode);



        ticpp::Node * dynPropNode = rigidbodies->FirstChild("DynamicProperties");
        processDynamicProperties(dynPropNode);







        //Copy the pointers!

        if(m_eBodiesState == RigidBodyType::SIMULATED) {
            if(m_bParseDynamics) {
                LOG(m_pSimulationLog,"---> Copy RigidBody References to DynamicSystem ..."<<std::endl;);
                for(int i=0; i < m_bodyList.size(); i++) {
                    m_pDynSys->m_SimBodies.addBody(m_bodyList[i]);
                }
            }
            m_nSimBodies += instances;
            m_nBodies += instances;
        } else if(m_eBodiesState == RigidBodyType::NOT_SIMULATED) {
            if(m_bParseDynamics) {
                LOG(m_pSimulationLog,"---> Copy RigidBody References to DynamicSystem ..."<<std::endl;);
                for(int i=0; i < m_bodyList.size(); i++) {
                    m_pDynSys->m_Bodies.addBody(m_bodyList[i]);
                }
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
        }else if(geometryNode->FirstChild()->Value() == "GlobalGeomId"){
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

            boost::shared_ptr<SphereGeometry<PREC> > pSphereGeom = boost::shared_ptr<SphereGeometry<PREC> >(new SphereGeometry<PREC>(radius));

            if(addToGlobalGeoms){
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,sphere->GetAttribute("id"))) {
                    throw ticpp::Exception("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0){
                    throw ticpp::Exception("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                addToGlobalGeomList(id,pSphereGeom);
            }else{
                for(int i=0; i < m_bodyList.size(); i++) {
                    m_bodyListScales[i] = scale;
                    m_bodyList[i]->m_geometry = pSphereGeom;
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


            if(addToGlobalGeoms){

                    unsigned int id;
                    if(!Utilities::stringToType<unsigned int>(id,sphere->GetAttribute("id"))) {
                        throw ticpp::Exception("---> String conversion in addToGlobalGeomList: id failed");
                    }
                    if(id == 0){
                        throw ticpp::Exception("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                        // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                    }

                    unsigned int instances = 1;
                    if(sphere->HasAttribute("instances")){

                        if(!Utilities::stringToType<unsigned int>(instances,sphere->GetAttribute("instances"))) {
                            throw ticpp::Exception("---> String conversion in addToGlobalGeomList: instances failed");
                        }
                    }

                    for(int i = id; i < id + instances;i++){
                        double radius = randomNumber();
                        boost::shared_ptr<SphereGeometry<PREC> > pSphereGeom = boost::shared_ptr<SphereGeometry<PREC> >(new SphereGeometry<PREC>(radius));
                        addToGlobalGeomList(i, pSphereGeom);
                    }
            }else{


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

            boost::shared_ptr<HalfspaceGeometry<PREC> > pHalfspaceGeom = boost::shared_ptr<HalfspaceGeometry<PREC> >(new HalfspaceGeometry<PREC>(n,p));

            if(addToGlobalGeoms){
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,halfspace->GetAttribute("id"))) {
                    throw ticpp::Exception("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0){
                    throw ticpp::Exception("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                addToGlobalGeomList(id, pHalfspaceGeom);
            }else{
                for(int i=0; i < m_bodyList.size(); i++) {
                    m_bodyList[i]->m_geometry = pHalfspaceGeom;
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

            boost::shared_ptr<BoxGeometry<PREC> > pBoxGeom = boost::shared_ptr<BoxGeometry<PREC> >(new BoxGeometry<PREC>(center,extent));

            Vector3 scale;
            scale(0)=extent(0);
            scale(1)=extent(1);
            scale(2)=extent(2);

            if(addToGlobalGeoms){
                    unsigned int id;
                    if(!Utilities::stringToType<unsigned int>(id,box->GetAttribute("id"))) {
                        throw ticpp::Exception("---> String conversion in addToGlobalGeomList: id failed");
                    }
                    if(id == 0){
                        throw ticpp::Exception("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                        // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                    }
                    addToGlobalGeomList(id, pBoxGeom);
            }else{
                for(int i=0; i < m_bodyList.size(); i++) {
                    m_bodyListScales[i] = scale;
                    m_bodyList[i]->m_geometry = pBoxGeom;
                }
            }

        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'Box' has no implementation in the parser"));
        }
    }


    virtual void processMeshGeometry( ticpp::Element * mesh, bool addToGlobalGeoms = false) {

        boost::shared_ptr<MeshGeometry<PREC> > pMeshGeom;

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
                                   aiProcess_Triangulate | aiProcess_GenNormals);

            // If the import failed, report it
            if(!scene) {
                throw ticpp::Exception("---> File import failed in processMeshGeometry: for file" + fileName.string() );
            }

            MeshData<MeshPREC> * meshData = new MeshData<MeshPREC>();

            if(!meshData->setup(importer,scene, scale_factor,quat,trans)) {
                throw ticpp::Exception("---> Imported Mesh (with Assimp) could not be setup internally");
            }

            // Build Geometry
            pMeshGeom = boost::shared_ptr<MeshGeometry<PREC> >(new MeshGeometry<PREC>(meshData));



            // Assign Geometry
            if(addToGlobalGeoms){
                    unsigned int id;
                    if(!Utilities::stringToType<unsigned int>(id,mesh->GetAttribute("id"))) {
                        throw ticpp::Exception("---> String conversion in addToGlobalGeomList: id failed");
                    }
                    if(id == 0){
                        throw ticpp::Exception("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                        // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                    }
                    addToGlobalGeomList(id, pMeshGeom);
            }else{
                for(int i=0; i < m_bodyList.size(); i++) {
                    m_bodyList[i]->m_geometry = pMeshGeom;
                }
            }

        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'Mesh' has no implementation in the parser"));
        }
    }

    virtual void processGlobalGeomId( ticpp::Element * globalGeomId ){

        std::string distribute = globalGeomId->GetAttribute("distribute");
        if(distribute == "uniform") {

            unsigned int id;
            if(!Utilities::stringToType<unsigned int>(id,globalGeomId->GetAttribute("id"))) {
                throw ticpp::Exception("---> String conversion in processGlobalGeomId: id failed");
            }

            typename DynamicsSystemType::GlobalGeometryMapType::iterator it = findGlobalGeomId(id);
            // it->second is the GeometryType in RigidBody
            if(it == this->getGlobalGeometryListRef().end()){
               LOG(m_pSimulationLog,"---> Geometry with id: " << id << " not found in global geometry list!" <<std::endl;);
               throw ticpp::Exception("---> Geometry search in processGlobalGeomId: failed!");
            }

            for(int i=0; i < m_bodyList.size(); i++){
                GetScaleOfGeomVisitor<DynamicsSystemType> vis(m_bodyListScales[i]);
                boost::apply_visitor(vis, it->second);
                m_bodyList[i]->m_geometry = it->second;
                m_bodyList[i]->m_globalGeomId = id;
            }

        }
        else if(distribute == "linear"){

            unsigned int startId;
            if(!Utilities::stringToType<unsigned int>(startId,globalGeomId->GetAttribute("startId"))) {
                throw ticpp::Exception("---> String conversion in processGlobalGeomId: startId failed");
            }

            if(startId == 0){
                throw ticpp::Exception("---> processGlobalGeomId: a global geometry startId: 0 is not allowed!");
                // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
            }

            for(int i=0; i < m_bodyList.size(); i++){
                int id = startId+i;
                typename DynamicsSystemType::GlobalGeometryMapType::iterator it = findGlobalGeomId(startId+i);
                // it->second is the GeometryType in RigidBody
                if(it == this->getGlobalGeometryListRef().end()){
                   LOG(m_pSimulationLog,"---> processGlobalGeomId: Geometry with id: " << startId+i << " not found in global geometry list!" <<std::endl;);
                   throw ticpp::Exception("---> processGlobalGeomId: Geometry search failed!");
                }

                GetScaleOfGeomVisitor<DynamicsSystemType> vis(m_bodyListScales[i]);
                boost::apply_visitor(vis, it->second);
                m_bodyList[i]->m_geometry = it->second;
                m_bodyList[i]->m_globalGeomId = id;
            }


        }
        else if(distribute == "random"){

            unsigned int startId;
            if(!Utilities::stringToType<unsigned int>(startId,globalGeomId->GetAttribute("startId"))) {
                throw ticpp::Exception("---> String conversion in processGlobalGeomId: startId failed");
            }

            if(startId == 0){
                throw ticpp::Exception("---> processGlobalGeomId: a global geometry startId: 0 is not allowed!");
                // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
            }

            unsigned int endId;
            if(!Utilities::stringToType<unsigned int>(endId,globalGeomId->GetAttribute("endId"))) {
                throw ticpp::Exception("---> String conversion in processGlobalGeomId: endId failed");
            }
            if(startId > endId){
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

            for(int i=0; i < m_bodyList.size(); i++){

                unsigned int id = randomNumber();
                typename DynamicsSystemType::GlobalGeometryMapType::iterator it = findGlobalGeomId(id);
                // it->second is the GeometryType in RigidBody
                if(it == this->getGlobalGeometryListRef().end()){
                   LOG(m_pSimulationLog,"---> Geometry with id: " << id << " not found in global geometry list!" <<std::endl;);
                   throw ticpp::Exception("---> Geometry search in processGlobalGeomId: failed!");
                }

                GetScaleOfGeomVisitor<DynamicsSystemType> vis(m_bodyListScales[i]);
                boost::apply_visitor(vis, it->second);
                m_bodyList[i]->m_geometry = it->second;
                m_bodyList[i]->m_globalGeomId = id;
            }


        }
        else {
            throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'GlobalGeomId' has no implementation in the parser"));
        }



    }


    virtual typename DynamicsSystemType::GlobalGeometryMapType::iterator findGlobalGeomId(unsigned int id){
        return m_pDynSys->m_globalGeometries.find(id);
    }


    template<typename T>
    void addToGlobalGeomList(unsigned int id,  boost::shared_ptr<T> ptr){

            std::pair<typename DynamicsSystemType::GlobalGeometryMapType::iterator, bool> ret =
            this->getGlobalGeometryListRef().insert(typename DynamicsSystemType::GlobalGeometryMapType::value_type( id, ptr) );
            if(ret.second == false){
                std::stringstream ss;
                ss << "---> addToGlobalGeomList: geometry with id: " <<  id<< " exists already!";
                throw ticpp::Exception(ss.str());
            }
            LOG(m_pSimulationLog,"---> Added geometry with id: " <<  id << " to global geometry list" <<std::endl;);
    }

    virtual typename DynamicsSystemType::GlobalGeometryMapType & getGlobalGeometryListRef(){
        return m_pDynSys->m_globalGeometries;
    }

    virtual void fillMeshInfo( Assimp::Importer & importer, const aiScene* scene, MeshData<MeshPREC> & meshInfo, Vector3 scale_factor, Quaternion quat, Vector3 trans) {



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
            m_eBodiesState =  RigidBodyType::SIMULATED;
        } else if(type == "not simulated") {
            m_eBodiesState =  RigidBodyType::NOT_SIMULATED;
        } else if(type == "animated") {
            m_eBodiesState =  RigidBodyType::ANIMATED;
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
        }

        // apply first to all bodies :-)
        for(int i=0; i < m_bodyList.size(); i++) {
            m_bodyList[i]->m_eState = m_eBodiesState;
        }

        if(m_eBodiesState == RigidBodyType::SIMULATED) {
            processDynamicPropertiesSimulated(dynProp);
        } else if(m_eBodiesState == RigidBodyType::NOT_SIMULATED) {
            processDynamicPropertiesNotSimulated(dynProp);
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
            throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'Mass' has no implementation in the parser"));
        }

        // InertiaTensor ============================================================
        element = dynProp->FirstChild("InertiaTensor")->ToElement();
        std::string type = element->GetAttribute("type");
        if(type == "homogen") {
            for(int i=0; i < m_bodyList.size(); i++) {
                InertiaTensor::calculateInertiaTensor(m_bodyList[i]);
            }
        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'InertiaTensor' has no implementation in the parser"));
        }

        element = dynProp->FirstChild("Material")->ToElement();
        distribute = element->GetAttribute("distribute");
        if(distribute == "uniform") {
            typename RigidBodyType::BodyMaterial eMaterial = 0;

            if(!Utilities::stringToType<typename RigidBodyType::BodyMaterial>(eMaterial, element->GetAttribute("id"))){
              throw ticpp::Exception("---> String conversion in Material: id failed");
            }

            for(int i=0; i < m_bodyList.size(); i++) {
                m_bodyList[i]->m_eMaterial = eMaterial;
            }
        } else {
            throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'Material' has no implementation in the parser"));
        }

         // InitialPosition ============================================================

        m_SimBodyInitStates.push_back( DynamicsState<LayoutConfigType>((unsigned int)m_bodyList.size()));

        element = dynProp->FirstChild("InitialPosition")->ToElement();
        distribute = element->GetAttribute("distribute");

        if(distribute == "linear") {
            processInitialPositionLinear(m_SimBodyInitStates.back(),element);
        } else if(distribute == "grid") {
            processInitialPositionGrid(m_SimBodyInitStates.back(),element);
        } else if(distribute == "file") {
            processInitialPositionFile(m_SimBodyInitStates.back(),element);
        } else if(distribute == "posaxisangle") {
            processInitialPositionPosAxisAngle(m_SimBodyInitStates.back(),element);
        } else if(distribute == "transforms") {
            processInitialPositionTransforms(m_SimBodyInitStates.back(),element);
        } else if(distribute == "none") {
            // does nothing leaves the zero state pushed!
        } else {
            throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'InitialPositionLinear' has no implementation in the parser"));
        }

        //Initial Velocity
        ticpp::Node * initVel = dynProp->FirstChild("InitialVelocity",false);
        if(initVel){
            element = initVel->ToElement();
            distribute = element->GetAttribute("distribute");

            if(distribute == "uniform") {
                processInitialVelocityTransRot(m_SimBodyInitStates.back(),element);
            } else if(distribute == "none") {
                // does nothing leaves the zero state pushed!
            } else {
                throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'InitialPositionLinear' has no implementation in the parser"));
            }
        }

        InitialConditionBodies::applyDynamicsStateToBodies<typename DynamicsSystemType::RigidBodyType,
            std::vector<RigidBodyType*> >(m_SimBodyInitStates.back(), m_bodyList);

    }


    virtual void processDynamicPropertiesNotSimulated( ticpp::Node * dynProp) {

        // InitialPositionLinear ============================================================
        ticpp::Element *element = dynProp->FirstChild("InitialPosition")->ToElement();
        std::string distribute = element->GetAttribute("distribute");

        DynamicsState<LayoutConfigType> state((unsigned int)m_bodyList.size());
        if(distribute == "linear") {
            processInitialPositionLinear(state,element);
        } else if(distribute == "grid") {
            processInitialPositionGrid(state,element);
        } else if(distribute == "file") {
            processInitialPositionFile(state,element);
        } else if(distribute == "posaxisangle") {
            processInitialPositionPosAxisAngle(state,element);
        } else if(distribute == "transforms") {
            processInitialPositionTransforms(state,element);
        } else if(distribute == "none") {
            // does nothing leaves the zero state pushed!
        } else {
            throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'InitialPositionLinear' has no implementation in the parser"));
        }

        InitialConditionBodies::applyDynamicsStateToBodies<
            typename DynamicsSystemType::RigidBodyType,
            std::vector<RigidBodyType*> >(state, m_bodyList);

        element = dynProp->FirstChild("Material")->ToElement();
        distribute = element->GetAttribute("distribute");
        if(distribute == "uniform") {
            typename RigidBodyType::BodyMaterial eMaterial = 0;

            if(!Utilities::stringToType<typename RigidBodyType::BodyMaterial>(eMaterial, element->GetAttribute("id"))){
              throw ticpp::Exception("---> String conversion in Material: id failed");
            }

            for(int i=0; i < m_bodyList.size(); i++) {
                m_bodyList[i]->m_eMaterial = eMaterial;
            }
        } else {
            throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'Material' has no implementation in the parser"));
        }

    }


    virtual void processInitialPositionLinear(DynamicsState<LayoutConfigType> & state, ticpp::Element * initCond) {

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
        if(initCond->HasAttribute("seed")){
            if(!Utilities::stringToType(seed, initCond->GetAttribute("seed"))) {
                throw ticpp::Exception("---> String conversion in InitialPositionGrid: jitter seed failed");
            }
        }

        InitialConditionBodies::setupPositionBodiesLinear(state,pos,dir,dist,jitter,delta,seed);

    }

    virtual void processInitialPositionGrid(DynamicsState<LayoutConfigType> & state, ticpp::Element * initCond) {

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
        if(initCond->HasAttribute("seed")){
            if(!Utilities::stringToType(seed, initCond->GetAttribute("seed"))) {
                throw ticpp::Exception("---> String conversion in InitialPositionGrid: jitter seed failed");
            }
        }
        double delta;
        if(!Utilities::stringToType<double>(delta, initCond->GetAttribute("delta"))) {
            throw ticpp::Exception("---> String conversion in InitialPositionGrid: delta failed");
        }

        InitialConditionBodies::setupPositionBodiesGrid(state,gridX,gridY,dist,trans,jitter,delta, seed);
    }

    virtual void processInitialPositionFile(DynamicsState<LayoutConfigType> & state, ticpp::Element * initCond) {
        m_SimBodyInitStates.push_back(DynamicsState<LayoutConfigType>((unsigned int)m_bodyList.size()));

        boost::filesystem::path name =  initCond->GetAttribute<std::string>("name");

        boost::filesystem::path filePath = m_currentParseFileDir / name;
        InitialConditionBodies::setupPositionBodiesFromFile(state,filePath);
    }

    virtual void processInitialPositionPosAxisAngle(DynamicsState<LayoutConfigType> & state, ticpp::Element * initCond) {

        int bodyCounter = 0;

        Vector3 pos;
        Vector3 axis;
        PREC angle;

        // Iterate over all values in the list
        ticpp::Iterator< ticpp::Element > valueElem("Value");
        for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++) {

            if(bodyCounter >= state.m_SimBodyStates.size()){
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


            InitialConditionBodies::setupPositionBodyPosAxisAngle(state.m_SimBodyStates[bodyCounter], pos, axis, angle);

            bodyCounter++;
        }

        if(bodyCounter < state.m_SimBodyStates.size()) {
            LOG(m_pSimulationLog,"---> InitialPositionPosAxisAngle: You specified to little values, -> applying last to all remainig bodies ..."<<std::endl;);
            for(int i=bodyCounter;i<state.m_SimBodyStates.size();i++){
                InitialConditionBodies::setupPositionBodyPosAxisAngle(state.m_SimBodyStates[i], pos, axis, angle);
            }
        }
    }

    virtual void processInitialPositionTransforms(DynamicsState<LayoutConfigType> & state, ticpp::Element * initCond) {



        int bodyCounter = 0;

        Quaternion q_KI, q_BK;
        Vector3 I_r_IK, K_r_KB;
        Matrix33 Rot_KI; // Temp

        // Iterate over all values in the list
        ticpp::Iterator< ticpp::Element > valueElem("Value");
        for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++) {

            if(bodyCounter >= state.m_SimBodyStates.size()){
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
                Rot_KI = getRotFromQuaternion(q_KI);
                I_r_IK += Rot_KI * K_r_KB; // Transforms like A_IK * A_r_AB;
                q_KI = quatMult(q_KI,q_BK); // Sequential (aktiv) rotation

            }

            // Apply overall transformation!
            state.m_SimBodyStates[bodyCounter].m_q.template head<3>() = I_r_IK;
            state.m_SimBodyStates[bodyCounter].m_q.template tail<4>() = q_KI;

            bodyCounter++;
        }

        if(bodyCounter < state.m_SimBodyStates.size()) {
            LOG(m_pSimulationLog,"---> InitialPositionTransforms: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
            for(int i=bodyCounter;i<state.m_SimBodyStates.size();i++){
                state.m_SimBodyStates[i].m_q.template head<3>() = I_r_IK;
                state.m_SimBodyStates[i].m_q.template tail<4>() = q_KI;
            }
        }

    }

    virtual void processInitialVelocityTransRot(DynamicsState<LayoutConfigType> & state, ticpp::Element * initCond) {


        int bodyCounter = 0;
        Vector3 transDir,rotDir;
        PREC rot,vel;

        // Iterate over all values in the list
        ticpp::Iterator< ticpp::Element > valueElem("Value");
        for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++) {

            if(bodyCounter >= state.m_SimBodyStates.size()){
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

            state.m_SimBodyStates[bodyCounter].m_u.template head<3>() = transDir*vel;
            state.m_SimBodyStates[bodyCounter].m_u.template tail<3>() = rotDir*rot;

            bodyCounter++;
        }

        if(bodyCounter < state.m_SimBodyStates.size()) {
            LOG(m_pSimulationLog,"---> InitialVelocityTransRot: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
            for(int i=bodyCounter;i<state.m_SimBodyStates.size();i++){
                state.m_SimBodyStates[i].m_u.template head<3>() = transDir*vel;
                state.m_SimBodyStates[i].m_u.template tail<3>() = rotDir*rot;
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
    unsigned int m_globalMaxGroupId; // Group Id used to build a unique id!
    // Temprary structures
    typename RigidBodyType::BodyState m_eBodiesState; ///< Used to process a RigidBody Node
    typename std::vector<RigidBodyType*> m_bodyList; ///< Used to process a RigidBody Node
    std::vector<Vector3> m_bodyListScales;
    std::vector< DynamicsState<LayoutConfigType> > m_SimBodyInitStates;



    typedef std::map<std::string, boost::shared_ptr<MeshGeometry<PREC> > > ContainerSceneMeshs;

};


#endif
