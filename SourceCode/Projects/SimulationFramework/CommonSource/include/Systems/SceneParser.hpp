#ifndef SceneParser_hpp
#define SceneParser_hpp

#include <vector>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include <memory>
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
#include "ContactParameter.hpp"
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

#include "Range.hpp"

class GetScaleOfGeomVisitor : public boost::static_visitor<> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    GetScaleOfGeomVisitor(Vector3 & scale): m_scale(scale) {};

    void operator()(  std::shared_ptr<const SphereGeometry >  & sphereGeom ) {
        m_scale.setConstant(sphereGeom->m_radius);
    }
    void operator()(  std::shared_ptr<const BoxGeometry >  & boxGeom) {
        m_scale = boxGeom->m_extent;
    }

    template<typename T>
    void operator()(  std::shared_ptr<T>  & ptr) {
        ERRORMSG("This GetScaleOfGeomVisitor visitor operator() has not been implemented!");
    }

    Vector3 & m_scale;
};

class SceneParser {
public:

    DEFINE_CONFIG_TYPES

    typedef typename DynamicsSystemType::GlobalGeometryMapType        GlobalGeometryMapType;
    typedef typename DynamicsSystemType::RigidBodyStatesContainerType RigidBodyStatesContainerType;
    typedef typename DynamicsSystemType::RigidBodySimContainerType    RigidBodySimContainerType ;
    typedef typename DynamicsSystemType::RigidBodyStaticContainerType RigidBodyStaticContainerType;

    typedef Range<RigidBodyIdType> BodyRange;

    SceneParser(std::shared_ptr<DynamicsSystemType> pDynSys)
        : m_pDynSys(pDynSys)
    {

        ASSERTMSG( m_pDynSys , "pDynSys is nullptr")

        m_pGlobalGeometries = &pDynSys->m_globalGeometries;
        m_pInitStates = &pDynSys->m_simBodiesInitStates;
        m_pSimBodies = &pDynSys->m_SimBodies;
        m_pBodies = &pDynSys->m_Bodies;


        m_pSimulationLog = nullptr;
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
        ASSERTMSG(m_pSimulationLog, "There is no SimulationLog in the LogManager!");

        setStandartValues();
    }

    /**
    * range is only applied to the groups with the attribute enableSelectiveIds="true"
    */
    template<typename BodyRangeType>
    bool parseScene( const boost::filesystem::path & file,
                     BodyRangeType&& range,
                     bool parseBodiesNonSelGroup = true,
                     bool parseOnlySimBodies = false,
                     bool parseSceneSettings = true
                    )
    {
        m_parseSceneObjects      = true;
        m_parseSelectiveBodyIds  = true;
        m_bodyIdRange            = std::forward<BodyRangeType>(range);
        m_parseAllBodiesNonSelGroup = parseBodiesNonSelGroup;
        m_parseOnlySimBodies     = parseOnlySimBodies;

        m_parseOnlyVisualizationProperties = m_pDynSys ?  false : true;
        m_allocateBodies = !m_parseOnlyVisualizationProperties;

        m_parseSceneSettings     = parseSceneSettings;

        // If the range of bodies is empty, dont use it;
        if(m_bodyIdRange.empty() ){
            m_parseSelectiveBodyIds = false;
        }
        m_startRangeIdIt = m_bodyIdRange.begin();

        parseSceneIntern(file);
    }

    // Parses all bodies
    void parseScene( const boost::filesystem::path & file,
                     bool parseSceneSettings = true,
                     bool parseSceneObjects = true,
                     bool parseOnlySimBodies = false
                     )
    {
        m_parseSceneObjects      = parseSceneObjects;
        m_parseSelectiveBodyIds  = false;
        m_parseAllBodiesNonSelGroup = true;
        m_parseOnlySimBodies     = parseOnlySimBodies;

        m_parseOnlyVisualizationProperties = m_pDynSys ? false : true;
        m_allocateBodies = !m_parseOnlyVisualizationProperties;

        m_parseSceneSettings     = parseSceneSettings;
        parseSceneIntern(file);
    }

    virtual void cleanUp() {
        m_bodyListGroup.clear();
        m_groupIdToNBodies.clear();
        m_bodyIdRangeTmp.clear();
    }

    virtual boost::filesystem::path getParsedSceneFile() {
        return m_currentParseFilePath;
    }

    virtual unsigned int getNumberOfSimBodies() {
        return m_nSimBodies;
    }


protected:

    SceneParser() {
        m_pSimulationLog = nullptr;
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
        ASSERTMSG(m_pSimulationLog, "There is no SimulationLog in the LogManager!");
        setStandartValues();
    }

    void setStandartValues(){
        m_nSimBodies = 0;
        m_nBodies = 0;
        m_globalMaxGroupId = 0;

        m_parseOnlyVisualizationProperties = false;
        m_parseSceneSettings = true;
        m_parseSceneObjects = true;

        m_allocateBodies = true;
    }

    virtual void parseSceneIntern(const boost::filesystem::path & file){


        if(!m_parseOnlyVisualizationProperties && !m_pDynSys){
            ERRORMSG("You try to parse the dynamics part of the bodies, but m_pDynSys is nullptr!" <<
                     "Please set it in the appropriate constructor of the SceneParser")
        }

        using namespace std;

        bool reparse =false;
        if(file != m_currentParseFilePath){
            reparse = true;
        }
        m_currentParseFilePath = file;
        m_currentParseFileDir = m_currentParseFilePath.parent_path();

        LOG( m_pSimulationLog, "---> Parsing Scene..." << std::endl;);
        LOG( m_pSimulationLog, "     m_parseSceneSettings: " << m_parseSceneSettings << std::endl <<
                               "     m_parseSelectiveBodyIds: "  << m_parseSelectiveBodyIds << std::endl <<
                               "     m_parseOnlySimBodies: "  << m_parseOnlySimBodies << std::endl <<
                               "     m_parseAllBodiesNonSelGroup: "  << m_parseAllBodiesNonSelGroup << std::endl <<
                               "     m_parseOnlyVisualizationProperties: "  << m_parseOnlyVisualizationProperties << std::endl;);

        LOG( m_pSimulationLog, "---> Scene Input file: "  << file.string() <<std::endl; );

        if(!boost::filesystem::exists(m_currentParseFilePath)) {
            ERRORMSG("Scene Input file does not exist!");
        }


        //Reset all variables
        m_globalMaxGroupId = 0;
        m_nSimBodies = 0;
        m_nBodies = 0;
        m_bodyListGroup.clear();
        m_groupIdToNBodies.clear();
        m_bodyIdRangeTmp.clear();

        try {

            if(reparse){
                m_xmlDoc.LoadFile(m_currentParseFilePath.string());
            }
            LOG(m_pSimulationLog, "---> File successfully loaded ..."<<std::endl;);

            LOG(m_pSimulationLog, "---> Try to parse the scene ..."<<std::endl;);

            // Start off with the gravity!
            m_xmlRootNode = m_xmlDoc.FirstChild("DynamicsSystem");
            if(m_xmlRootNode) {
                ticpp::Node *node = nullptr;

                node = m_xmlRootNode->FirstChild("SceneSettings");
                parseSceneSettings(node);

                node = m_xmlRootNode->FirstChild("SceneObjects");
                parseSceneObjects(node);

                node = m_xmlRootNode->FirstChild("SceneSettings");
                parseSceneSettings2(node);

                parseOtherOptions(m_xmlRootNode);

            } else {
                LOG(m_pSimulationLog, "---> No DynamicsSystem Node found in XML ..." <<std::endl;);
                ERRORMSG("---> No DynamicsSystem Node found in XML ..."<<std::endl);
            }

        } catch(ticpp::Exception& ex) {
            LOG(m_pSimulationLog,  "Scene XML error: "  << ex.what() <<std::endl);
            ERRORMSG( "Scene XML error: "  << ex.what() <<std::endl );
        }

    }

    /// SceneSettings ==============================================================================
    // virtual
    virtual void parseOtherOptions(const ticpp::Node *rootNode) {
        /* Do nothing, only for derived classes! */
    }

    virtual void parseSceneSettings( ticpp::Node *sceneSettings ) {
        if(!m_parseSceneSettings){
            LOG(m_pSimulationLog,"---> Skip SceneSettings"<<std::endl;);
            return;
        }
        LOG(m_pSimulationLog,"---> Parse SceneSettings..."<<std::endl;);

        if(!m_parseOnlyVisualizationProperties) {

            ticpp::Element *gravityElement = sceneSettings->FirstChild("Gravity",true)->ToElement();
            m_pDynSys->m_gravity = gravityElement->GetAttribute<double>("value");

            if(!Utilities::stringToVector3<PREC>(m_pDynSys->m_gravityDir , gravityElement->GetAttribute("direction"))) {
                throw ticpp::Exception("---> String conversion in SceneSettings: gravity failed");
            }

            m_pDynSys->m_gravityDir.normalize();

            ticpp::Element *timestepElement = sceneSettings->FirstChild("TimeStepperSettings",true)->ToElement();

            TimeStepperSettings timestepperSettings;
            InclusionSolverSettingsType inclusionSettings;

            // Get standart values!
            m_pDynSys->getSettings(timestepperSettings,inclusionSettings);

            if(!Utilities::stringToType<PREC>(timestepperSettings.m_deltaT, timestepElement->GetAttribute("deltaT"))) {
                throw ticpp::Exception("---> String conversion in SceneSettings: deltaT failed");
            }
            inclusionSettings.m_deltaT = timestepperSettings.m_deltaT;

            if(!Utilities::stringToType<PREC>(timestepperSettings.m_endTime, timestepElement->GetAttribute("endTime"))) {
                throw ticpp::Exception("---> String conversion in SceneSettings: endTime failed");
            }

            ticpp::Node * ptr = timestepElement->FirstChild("SimulateFromReference",false);
            if(ptr) {
                ticpp::Element *simFromRef = ptr->ToElement();
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
                    throw ticpp::Exception("---> String conversion in InclusionSolverSettings: alphaJORProx failed");
                }
                if(!Utilities::stringToType<PREC>(inclusionSettings.m_alphaSORProx, inclusionElement->GetAttribute("alphaSORProx"))) {
                    throw ticpp::Exception("---> String conversion in InclusionSolverSettings: alphaJORProx failed");
                }
                if(!Utilities::stringToType<unsigned int>(inclusionSettings.m_MaxIter, inclusionElement->GetAttribute("maxIter"))) {
                    throw ticpp::Exception("---> String conversion in InclusionSolverSettings: maxIter failed");
                }

                if(inclusionElement->HasAttribute("minIter")) {
                    if(!Utilities::stringToType<unsigned int>(inclusionSettings.m_MinIter, inclusionElement->GetAttribute("minIter"))) {
                        throw ticpp::Exception("---> String conversion in InclusionSolverSettings: minIter failed");
                    }
                } else {
                    inclusionSettings.m_MinIter = 0;
                }

                if(inclusionElement->HasAttribute("convergenceMethod")) {
                    std::string method = inclusionElement->GetAttribute("convergenceMethod");
                    if(method == "InLambda") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettingsType::InLambda;
                    } else if (method == "InVelocity") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettingsType::InVelocity;
                    } else if (method == "InVelocityLocal") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettingsType::InVelocityLocal;
                    } else if (method == "InEnergyVelocity") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettingsType::InEnergyVelocity;
                    } else if (method == "InEnergyLocalMix") {
                        inclusionSettings.m_eConvergenceMethod = InclusionSolverSettingsType::InEnergyLocalMix;
                    } else {
                        throw ticpp::Exception("---> String conversion in InclusionSolverSettings: convergenceMethod failed: not a valid setting");
                    }
                } else {
                    inclusionSettings.m_eConvergenceMethod = InclusionSolverSettingsType::InVelocity;
                }

                if(!Utilities::stringToType<PREC>(inclusionSettings.m_AbsTol, inclusionElement->GetAttribute("absTol"))) {
                    throw ticpp::Exception("---> String conversion in InclusionSolverSettings: absTol failed");
                }
                if(!Utilities::stringToType<PREC>(inclusionSettings.m_RelTol, inclusionElement->GetAttribute("relTol"))) {
                    throw ticpp::Exception("---> String conversion in InclusionSolverSettings: relTol failed");
                }

                if(inclusionElement->HasAttribute("computeResidual")) {
                    if(!Utilities::stringToType<bool>(inclusionSettings.m_bComputeResidual, inclusionElement->GetAttribute("computeResidual"))) {
                    throw ticpp::Exception("---> String conversion in InclusionSolverSettings: computeResidual failed");
                }
                }


                if(inclusionElement->HasAttribute("isFiniteCheck")) {
                    if(!Utilities::stringToType<bool>(inclusionSettings.m_bIsFiniteCheck, inclusionElement->GetAttribute("isFiniteCheck"))) {
                        throw ticpp::Exception("---> String conversion in InclusionSolverSettings: isFiniteCheck failed");
                    }
                }

                std::string method = inclusionElement->GetAttribute("method");
                if(method == "JOR") {
                    inclusionSettings.m_eMethod = InclusionSolverSettingsType::JOR;
                } else if (method == "SOR") {
                    inclusionSettings.m_eMethod = InclusionSolverSettingsType::SOR_CONTACT;
                } else if (method == "SORContact") {
                    inclusionSettings.m_eMethod = InclusionSolverSettingsType::SOR_CONTACT;
                } else if (method == "SORFull") {
                    inclusionSettings.m_eMethod = InclusionSolverSettingsType::SOR_FULL;
                } else {
                    throw ticpp::Exception("---> String conversion in InclusionSolverSettings: method failed: not a valid setting");
                }


                if(!Utilities::stringToType<bool>(inclusionSettings.m_bUseGPU, inclusionElement->GetAttribute("useGPU"))) {
                    throw ticpp::Exception("---> String conversion in InclusionSolverSettings: useGPU failed");
                }

                if(inclusionElement->HasAttribute("useGPUID")) {
                    if(!Utilities::stringToType<int>(inclusionSettings.m_UseGPUDeviceId, inclusionElement->GetAttribute("useGPUID"))) {
                        throw ticpp::Exception("---> String conversion in InclusionSolverSettings: useGPU failed");
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
            parseContactParameterMap(sceneSettings);

            // Parse external forces
            parseExternalForces(sceneSettings);

        } // m_parseOnlyVisualizationProperties

        // Parse Global Geometries (also in playback manager)!
        parseGlobalGeometries(sceneSettings);

    }
    virtual void parseSceneSettings2( ticpp::Node *sceneSettings ) {

        LOG(m_pSimulationLog,"---> Parse SceneSettings2..."<<std::endl;);

        if(!m_parseOnlyVisualizationProperties) {
            LOG(m_pSimulationLog,"---> Parse GlobalInitialCondition..."<<std::endl;);
            parseGlobalInitialCondition(sceneSettings);
        }
    }

    virtual void parseContactParameterMap(ticpp::Node *sceneSettings) {


        ticpp::Node *paramMap = sceneSettings->FirstChild("ContactParameterMap",false);


        if(paramMap) {
            LOG(m_pSimulationLog,"---> Parse ContactParameterMap..."<<std::endl;);

            ticpp::Element * element = paramMap->FirstChild("ContactParameterStandard",true)->ToElement();
            if(element) {
                parseContactParameter(element, true);
            }


            ticpp::Iterator< ticpp::Element > valueElem("ContactParameter");
            for ( valueElem = valueElem.begin( paramMap->ToElement() ); valueElem != valueElem.end(); valueElem++) {
               parseContactParameter(&(*valueElem));
            }

        }


    }
    virtual void parseContactParameter(ticpp::Element * contactParam, bool stdMaterial=false){
        typename RigidBodyType::BodyMaterialType material1,material2;
        if(!stdMaterial){
            if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(material1, contactParam->GetAttribute("materialId1"))) {
                throw ticpp::Exception("---> String conversion in ContactParameter: materialId1 failed");
            }
            if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(material2, contactParam->GetAttribute("materialId2"))) {
                throw ticpp::Exception("---> String conversion in ContactParameter: materialId2 failed");
            }
        }

        std::string type = contactParam->GetAttribute("type");
        if(type == "UCF" || type == "UCFD" || type == "UCFDD"){

            PREC mu,epsilonN,epsilonT;
            ContactParameter contactParameter;

            if(!Utilities::stringToType<PREC>(mu, contactParam->GetAttribute("mu"))) {
                throw ticpp::Exception("---> String conversion in ContactParameter: mu failed");
            }
            if(!Utilities::stringToType<PREC>(epsilonN, contactParam->GetAttribute("epsilonN"))) {
                throw ticpp::Exception("---> String conversion in ContactParameter: epsilonN failed");
            }
            if(!Utilities::stringToType<PREC>(epsilonT, contactParam->GetAttribute("epsilonT"))) {
                throw ticpp::Exception("---> String conversion in ContactParameter: epsilonT failed");
            }

            if(type == "UCFD"){
                PREC invDampingN, invDampingT;
                if(!Utilities::stringToType<PREC>(invDampingN, contactParam->GetAttribute("invDampingN"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: invDampingN failed");
                }
                if(!Utilities::stringToType<PREC>(invDampingT, contactParam->GetAttribute("invDampingT"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: invDampingT failed");
                }

                contactParameter = ContactParameter::createParams_UCFD_ContactModel(epsilonN,epsilonT,mu,invDampingN,invDampingT);


            }else if(type == "UCFDD"){

                PREC invDampingN, gammaMax, epsilon, invDampingTFix;
                if(!Utilities::stringToType<PREC>(invDampingN, contactParam->GetAttribute("invDampingN"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: invDampingN failed");
                }

                if(!Utilities::stringToType<PREC>(invDampingTFix, contactParam->GetAttribute("invDampingTFix"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: invDampingTFix failed");
                }
                if(!Utilities::stringToType<PREC>(gammaMax, contactParam->GetAttribute("gammaMax"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: gamma_max failed");
                }
                if(!Utilities::stringToType<PREC>(epsilon, contactParam->GetAttribute("epsilon"))) {
                    throw ticpp::Exception("---> String conversion in ContactParameter: epsilon failed");
                }

                contactParameter = ContactParameter::createParams_UCFDD_ContactModel(epsilonN,epsilonT,mu,
                                                                                     invDampingN,invDampingTFix,
                                                                                     gammaMax,epsilon);
            }
            else{
                contactParameter = ContactParameter::createParams_UCF_ContactModel(epsilonN,epsilonT,mu);
            }

            if(stdMaterial){
                LOG(m_pSimulationLog,"---> Add ContactParameter standart"<<std::endl;);
                m_pDynSys->m_ContactParameterMap.setStandardValues(contactParameter);
            }
            else{
                LOG(m_pSimulationLog,"---> Add ContactParameter standart of id="<<material1<<" to id="<<material2<<std::endl;);
                if(!m_pDynSys->m_ContactParameterMap.addContactParameter(material1,material2,contactParameter)){
                    throw ticpp::Exception("---> Add ContactParameter failed");
                }
            }


        }
        else{
            throw ticpp::Exception("---> String conversion in ContactParameter: type failed");
        }
    }

    virtual void parseGlobalGeometries(ticpp::Node *sceneSettings) {

        ticpp::Node *globalGeom = sceneSettings->FirstChild("GlobalGeometries",false);
        if(globalGeom) {
            ticpp::Iterator< ticpp::Node > child;
            for ( child = child.begin( globalGeom ); child != child.end(); child++ ) {

                if( child->Value() == "Geometry") {
                    parseGeometry( &(*child) , true);
                }
            }
        }
    }
    virtual void parseExternalForces( ticpp::Node *sceneSettings ) {
        ticpp::Node *externalForces = sceneSettings->FirstChild("ExternalForces",false);
        if(externalForces ) {
            LOG(m_pSimulationLog,"---> Parse ExternalForces ..."<<std::endl;);
            ticpp::Iterator< ticpp::Node > child;
            for ( child = child.begin( externalForces ); child != child.end(); child++ ) {
                if( child->Value() == "ForceField") {
                    ticpp::Element* elem = (&(*child))->ToElement();
                    parseForceField( elem );
                }
            }
        }
    }
    virtual void parseForceField( ticpp::Element * forceField) {

        bool enabled = false;
        if(!Utilities::stringToType<bool>(enabled, forceField->GetAttribute("enabled"))) {
            throw ticpp::Exception("---> String conversion in parseForceField: enable failed");
        }
        if(enabled) {

            std::string apply  = forceField->GetAttribute("applyTo");

            std::vector<RigidBodyIdType > applyList;
            if( !(apply=="all" || apply=="All" || apply=="ALL" )) {
                //parse all applyTo bodies
            } else if (apply=="all" || apply=="All" || apply=="ALL" ) {
                // do nothing
            } else {
                throw ticpp::Exception("---> String conversion in parseForceField: applyTo failed");
            }

            std::string type = forceField->GetAttribute("type");
            if(type == "spatialspherical-timerandom") {

                unsigned int seed;
                if(!Utilities::stringToType<unsigned int>(seed, forceField->GetAttribute("seed"))) {
                    throw ticpp::Exception("---> String conversion in parseForceField: seed failed");
                }
                PREC boostTime;
                if(!Utilities::stringToType<PREC>(boostTime, forceField->GetAttribute("boostTime"))) {
                    throw ticpp::Exception("---> String conversion in parseForceField: boostTime failed");
                }
                PREC pauseTime;
                if(!Utilities::stringToType<PREC>(pauseTime, forceField->GetAttribute("pauseTime"))) {
                    throw ticpp::Exception("---> String conversion in parseForceField: pauseTime failed");
                }

                PREC startTime;
                if(!Utilities::stringToType<PREC>(startTime, forceField->GetAttribute("startTime"))) {
                    throw ticpp::Exception("---> String conversion in parseForceField: startTime failed");
                }

                PREC endTime;
                if(!Utilities::stringToType<PREC>(endTime, forceField->GetAttribute("endTime"))) {
                    throw ticpp::Exception("---> String conversion in parseForceField: endTime failed");
                }
                PREC amplitude;
                if(!Utilities::stringToType<PREC>(amplitude, forceField->GetAttribute("amplitude"))) {
                    throw ticpp::Exception("---> String conversion in parseForceField: amplitude failed");
                }

                Vector3 boxMin;
                if(!Utilities::stringToVector3<PREC>(boxMin, forceField->GetAttribute("minPoint"))) {
                    throw ticpp::Exception("---> String conversion in parseForceField: boxMin failed");
                }
                Vector3 boxMax;
                if(!Utilities::stringToVector3<PREC>(boxMax, forceField->GetAttribute("maxPoint"))) {
                    throw ticpp::Exception("---> String conversion in parseForceField: boxMax failed");
                }

                bool randomOn;
                if(!Utilities::stringToType<bool>(randomOn, forceField->GetAttribute("randomOn"))) {
                    throw ticpp::Exception("---> String conversion in parseForceField: randomOn failed");
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
                throw ticpp::Exception("---> String conversion in parseForceField: applyTo failed");
            }
        }
    }

    virtual void parseGlobalInitialCondition( ticpp::Node *sceneSettings) {
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

                boost::filesystem::path relpath = elem->GetAttribute("path");


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
    ///  ============================================================================================

    /// SceneObjects ==========================================================================================================
    virtual void parseSceneObjects( ticpp::Node *sceneObjects) {
        if(!m_parseSceneObjects){
            LOG(m_pSimulationLog,"---> Skip SceneObjects"<<std::endl;);
            return;
        }
        LOG(m_pSimulationLog,"---> Parse SceneObjects ..."<<std::endl;);

        ticpp::Iterator< ticpp::Node > child;

        for ( child = child.begin( sceneObjects ); child != child.end(); child++ ) {

            if( child->Value() == "RigidBodies") {
                parseRigidBodies( &(*child) );
            }

        }

        if( m_nSimBodies == 0) {
            throw ticpp::Exception("---> The scene in the XML contains no simulating bodies!");
        }

    }
    virtual void parseRigidBodies( ticpp::Node * rigidbodies ) {

        ticpp::Element* rigidBodiesEl = rigidbodies->ToElement();
        LOG(m_pSimulationLog,"---> Parse RigidBodies, group name: "<< rigidBodiesEl->GetAttribute<std::string>("name",false) << std::endl;);

        //Clear current body list;
        m_bodyListGroup.clear();

        // Check if this group has selecitveIdsOnly turned on to load only selective bodies
        bool hasSelectiveFlag = false;
        if(rigidBodiesEl->HasAttribute("enableSelectiveIds")){
           Utilities::stringToType<bool>(hasSelectiveFlag,rigidBodiesEl->GetAttribute("enableSelectiveIds"));
        }

        unsigned int instances = rigidbodies->ToElement()->GetAttribute<unsigned int>("instances");

        // Determine what DynamicState the group has:
        ticpp::Node * dynPropNode = rigidbodies->FirstChild("DynamicProperties");
        parseDynamicState(dynPropNode);

        // Determine GroupId (if specified, otherwise maximum)
        unsigned int groupId;
        if(rigidBodiesEl->HasAttribute("groupId")) {
            groupId = rigidBodiesEl->GetAttribute<unsigned int>("groupId");
            // Set new m_globalMaxGroupId;
            m_globalMaxGroupId = std::max(m_globalMaxGroupId,groupId);
        } else {
            m_globalMaxGroupId++;
            groupId = m_globalMaxGroupId;
        }

        // Get the latest body id for this group id
        unsigned int startBodyNr = 0;
        typename GroupToNBodyType::mapped_type * currGroupIdToNBodies;
        auto it = m_groupIdToNBodies.find(groupId);
        if( it == m_groupIdToNBodies.end()) {
            currGroupIdToNBodies = &m_groupIdToNBodies[groupId];
            *currGroupIdToNBodies = 0;
        }else{
            currGroupIdToNBodies = &(it->second);
            startBodyNr = *currGroupIdToNBodies;
        }

        // Skip group if we can:
        if( (!m_parseAllBodiesNonSelGroup && !hasSelectiveFlag)
            || ( m_eBodiesState != RigidBodyType::BodyState::SIMULATED  && m_parseOnlySimBodies)
            || (m_parseSelectiveBodyIds && hasSelectiveFlag && m_startRangeIdIt==m_bodyIdRange.end()) // out of m_bodyIdRange, no more id's which could be parsed in
             ) {
            LOG(m_pSimulationLog, "Skip Group" << std::endl;)
            // update the number of bodies in this group and skip this group xml node
            *currGroupIdToNBodies += instances;
            return;
        }

        // Full id range for this group would be [m_starBodyId , endBodyId ]
        m_startIdGroup = RigidBodyId::makeId(startBodyNr, groupId);
        RigidBodyIdType endBodyId = RigidBodyId::makeId(startBodyNr+instances-1, groupId);
        LOG(m_pSimulationLog,"---> Group range: [" << RigidBodyId::getBodyIdString(m_startIdGroup)
                             << "," << RigidBodyId::getBodyIdString(endBodyId) << "]" << std::endl;)

        typename BodyRange::iterator bodyIdIt;
        bool updateStartRange = false;

        if(m_parseSelectiveBodyIds && hasSelectiveFlag){
            // parse group selective , determine start iterator in bodyRang
            m_bodyIdRangePtr = &m_bodyIdRange;
            m_startRangeIdIt = std::lower_bound(m_startRangeIdIt,m_bodyIdRangePtr->end(),m_startIdGroup);
            bodyIdIt = m_startRangeIdIt;
            if( m_startRangeIdIt == m_bodyIdRangePtr->end()){ // no ids in the range
                *currGroupIdToNBodies += instances;
                LOG(m_pSimulationLog,"---> No ids in range: skip" << std::endl;)
                return;
            }
            LOG(m_pSimulationLog,"---> Selective range startId: " <<
                                  RigidBodyId::getBodyIdString(*m_startRangeIdIt) << std::endl;)
            updateStartRange = true;
        }else{
            // parse all bodies;
            //overwrite range containing all bodies, if we don't parse selective ids, parse all bodies!
            m_bodyIdRangePtr = &m_bodyIdRangeTmp;
            m_bodyIdRangeTmp = std::make_pair(m_startIdGroup,endBodyId+1);
            bodyIdIt = m_bodyIdRangePtr->begin();
            LOG(m_pSimulationLog,"---> overwrite selective range... " << std::endl;)
        }

        // Adding bodies in the range =============================
        // iterator bodyRange till the id is > endBodyId or we are out of the bodyIdRange
        auto itEnd = m_bodyIdRangePtr->end();
        m_parsedInstancesGroup = 0;
        for( /* nothing*/ ; (bodyIdIt != itEnd) && ( *bodyIdIt <= endBodyId); ++bodyIdIt )
        {
            LOG(m_pSimulationLog,"---> Added RigidBody Instance: "<<RigidBodyId::getBodyIdString(*bodyIdIt)<<std::endl);
            // Push new body
            if(m_allocateBodies){
                m_bodyListGroup.emplace_back( new RigidBodyType(*bodyIdIt), *bodyIdIt, Vector3(1,1,1));
            }else{
                // add no bodies for visualization stuff, we dont need it!
                m_bodyListGroup.emplace_back( nullptr, *bodyIdIt, Vector3(1,1,1));
            }


            m_parsedInstancesGroup++;
        }

        // Only update start range for selective parsing;
        if(updateStartRange){ m_startRangeIdIt = bodyIdIt;}

        LOG(m_pSimulationLog,"---> Added "<<m_parsedInstancesGroup<<" RigidBody Instances..."<<std::endl;);
        // =======================================================

        // Parse Geometry
        ticpp::Node * geometryNode = rigidbodies->FirstChild("Geometry");
        parseGeometry(geometryNode);

         //Parse DynamicsProperties
        parseDynamicProperties(dynPropNode);

        //Copy the pointers!
        if(m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {
            LOG(m_pSimulationLog,"---> Copy Simulated RigidBody References to DynamicSystem ..."<<std::endl;);
            bool added = addAllBodies<true,true>(m_pSimBodies, m_pInitStates);
            if(!added) {ERRORMSG("Could not add body to m_SimBodies!, some bodies exist already in map!");};
            m_nSimBodies += m_parsedInstancesGroup;
            m_nBodies += m_parsedInstancesGroup;
        } else if(m_eBodiesState == RigidBodyType::BodyState::STATIC) {
            LOG(m_pSimulationLog,"---> Copy Static RigidBody References to DynamicSystem ..."<<std::endl;);
            bool added = addAllBodies<true,false>(m_pBodies, m_pInitStates);
            if(!added) {ERRORMSG("Could not add body to m_Bodies!, some bodies exist already in map!");};
            m_nBodies += m_parsedInstancesGroup;
        } else {
            throw ticpp::Exception("---> Adding only simulated and not simulated objects supported!");
        }

        ticpp::Node * visualizationNode = rigidbodies->FirstChild("Visualization");
        parseVisualization( visualizationNode);
        // ===============================================================================================================


    }

    /** General helper template to add all result to the lists */
    template<bool addBodies, bool addInitState, typename BodyContainer, typename StateContainer>
    inline bool addAllBodies(BodyContainer * bodies, StateContainer * initStates){
        bool added = true;
        if(addBodies){
            if(bodies && !m_parseOnlyVisualizationProperties){
                for( auto & b : m_bodyListGroup){
                    added &= bodies->addBody(b.m_body);
                }
            }
        }
        if(addInitState){
            if(initStates){
                for( auto & b : m_bodyListGroup){
                    added &= initStates->emplace(b.m_initState.m_id, b.m_initState).second;
                }
            }
        }
        return added;
    }

    /// Geometries ==============================================================================
    virtual void parseGeometry( ticpp::Node * geometryNode, bool addToGlobalGeoms = false) {

        ASSERTMSG( (m_parseOnlyVisualizationProperties  && !addToGlobalGeoms) || (!m_parseOnlyVisualizationProperties) ,
        "parse only visualization, addToGlobalGeoms should be false")

        LOG(m_pSimulationLog,"---> Parse Geometry ..."<<std::endl;);
        if(geometryNode->FirstChild()->Value() == "Sphere") {
            parseSphereGeometry( geometryNode->FirstChild()->ToElement(),  addToGlobalGeoms);
        } else if(geometryNode->FirstChild()->Value() == "Halfspace") {
            parseHalfspaceGeometry( geometryNode->FirstChild()->ToElement(),  addToGlobalGeoms);
        } else if(geometryNode->FirstChild()->Value() == "Mesh") {
            parseMeshGeometry( geometryNode->FirstChild()->ToElement(),  addToGlobalGeoms);
        } else if(geometryNode->FirstChild()->Value() == "Box") {
            parseBoxGeometry( geometryNode->FirstChild()->ToElement(),  addToGlobalGeoms);
        } else if(geometryNode->FirstChild()->Value() == "GlobalGeomId") {
            parseGlobalGeomId(geometryNode->FirstChild()->ToElement());
        } else {
            throw ticpp::Exception("---> The geometry '" + std::string(geometryNode->FirstChild()->Value()) + std::string("' has no implementation in the parser"));
        }
    }
    virtual void parseSphereGeometry( ticpp::Element * sphere, bool addToGlobalGeoms = false) {
        std::string type = sphere->GetAttribute("distribute");

        if(type == "uniform") {
            double radius = sphere->GetAttribute<double>("radius");
            if(addToGlobalGeoms) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,sphere->GetAttribute("id"))) {
                    throw ticpp::Exception("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    throw ticpp::Exception("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                addToGlobalGeomList(id,std::shared_ptr<SphereGeometry >(new SphereGeometry(radius)));
            } else {
                Vector3 scale(radius,radius,radius);
                for(auto & b : m_bodyListGroup) {
                    b.m_scale = scale;
                    LOG(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_initState.m_id) << ", GeometryType: Sphere" << std::endl);
                    LOG(m_pSimulationLog, "\t\t---> radius: " << radius << std::endl; );
                    if(m_allocateBodies){
                        b.m_body->m_geometry = std::shared_ptr<SphereGeometry >(new SphereGeometry(radius));
                    }
                }
            }
        } else if(type == "random") {
            double minRadius;
            if(!Utilities::stringToType<double>(minRadius,sphere->GetAttribute("minRadius"))) {
                throw ticpp::Exception("---> String conversion in parseSphereGeometry: minRadius failed");
            }
            if( minRadius <= 0) {
                throw ticpp::Exception("---> In parseSphereGeometry: minRadius to small!");
            }

            double maxRadius;
            if(!Utilities::stringToType<double>(maxRadius,sphere->GetAttribute("maxRadius"))) {
                throw ticpp::Exception("---> String conversion in parseSphereGeometry: minRadius failed");
            }
            if( maxRadius <= minRadius) {
                throw ticpp::Exception("---> In parseSphereGeometry: maxRadius smaller or equal to minRadius!");
            }

            unsigned int seed;
            if(!Utilities::stringToType<unsigned int>(seed,sphere->GetAttribute("seed"))) {
                throw ticpp::Exception("---> String conversion in parseSphereGeometry: seed failed");
            }


            RandomGenType gen(seed);
            UniformDistType<PREC> uni(minRadius,maxRadius);

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
                    PREC radius = uni(gen);
                    addToGlobalGeomList(i, std::shared_ptr<SphereGeometry >(new SphereGeometry(radius)));
                }
            } else {


                RigidBodyIdType diffId = m_startIdGroup; // id to generate to correct amount of random values!
                double radius = uni(gen); // generate first value
                auto endIt = m_bodyListGroup.end();
                for(auto bodyIt = m_bodyListGroup.begin(); bodyIt != endIt; ++bodyIt) {

                    // Generate the intermediate random values if there are any
                    radius = Utilities::genRandomValues(radius,gen,uni,bodyIt->m_initState.m_id-diffId); // (id:16 - id:13 = 3 values, 13 is already generated)
                    diffId = bodyIt->m_initState.m_id; // update current diffId;

                    bodyIt->m_scale = Vector3(radius,radius,radius);
                    LOG(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(bodyIt->m_initState.m_id)<< ", GeometryType: Sphere" << std::endl);
                    LOG(m_pSimulationLog, "\t\t---> radius: " << radius << std::endl; );

                    if(m_allocateBodies){
                        bodyIt->m_body->m_geometry = std::shared_ptr<SphereGeometry >(new SphereGeometry(radius));
                    }

                }
            }



        } else {
            throw ticpp::Exception("---> The attribute 'distribute' '" + type + std::string("' of 'Sphere' has no implementation in the parser"));
        }


    }
    virtual void parseHalfspaceGeometry( ticpp::Element * halfspace, bool addToGlobalGeoms = false) {
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

            std::shared_ptr<HalfspaceGeometry > pHalfspaceGeom = std::shared_ptr<HalfspaceGeometry >(new HalfspaceGeometry(n,p));

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

                if(m_allocateBodies){
                    for(auto & b  : m_bodyListGroup) {
                        b.m_body->m_geometry = pHalfspaceGeom;
                    }
                }
            }

        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'Halfspace' has no implementation in the parser"));
        }
    }
    virtual void parseBoxGeometry( ticpp::Element * box, bool addToGlobalGeoms = false) {
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

            std::shared_ptr<BoxGeometry > pBoxGeom = std::shared_ptr<BoxGeometry >(new BoxGeometry(center,extent));

            Vector3 scale(extent(0),extent(1),extent(2));

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
                 for(auto & b  : m_bodyListGroup) {
                    b.m_scale = scale;
                    if(m_allocateBodies){
                        b.m_body->m_geometry = pBoxGeom;
                    }
                }
            }

        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'Box' has no implementation in the parser"));
        }
    }
    virtual void parseMeshGeometry( ticpp::Element * mesh, bool addToGlobalGeoms = false) {

        std::shared_ptr<MeshGeometry > pMeshGeom;

        std::string meshName = mesh->GetAttribute<std::string>("name");

        bool bInstantiate;
        if(!Utilities::stringToType<bool>(bInstantiate,mesh->GetAttribute("useInstance"))) {
            throw ticpp::Exception("---> String conversion in parseMeshGeometry: useInstance failed");
        }

        std::string type = mesh->GetAttribute("distribute");
        if(type == "uniform") {

            // import here an object model;
            // First make an meshinformatino structure

            boost::filesystem::path fileName =  mesh->GetAttribute<std::string>("file");
            checkFileExists(fileName);

            Vector3 scale_factor;
            if(!Utilities::stringToVector3<PREC>(scale_factor, mesh->GetAttribute("scale"))) {
                throw ticpp::Exception("---> String conversion in parseMeshGeometry failed: scale");
            }
            if(scale_factor.norm()==0) {
                throw ticpp::Exception("---> Wrong scale factor (=0) specified in parseMeshGeometry!");
            }

            Vector3 trans;
            if(!Utilities::stringToVector3<PREC>(trans, mesh->GetAttribute("translation"))) {
                throw ticpp::Exception("---> String conversion in parseMeshGeometry: translation failed: ");
            }

            Vector3 axis;
            if(!Utilities::stringToVector3<PREC>(axis, mesh->GetAttribute("rotationAxis"))) {
                throw ticpp::Exception("---> String conversion in parseMeshGeometry: rotationAxis failed");
            }

            PREC angle;

            if(mesh->HasAttribute("angleDegree")) {
                if(!Utilities::stringToType<PREC>(angle, mesh->GetAttribute("angleDegree"))) {
                    throw ticpp::Exception("---> String conversion in parseMeshGeometry: angleDegree failed");
                }
                angle = angle / 180 * M_PI;
            } else if(mesh->HasAttribute("angleRadian")) {
                if(!Utilities::stringToType<PREC>(angle, mesh->GetAttribute("angleRadian"))) {
                    throw ticpp::Exception("---> String conversion in parseMeshGeometry: angleRadian  failed");
                }
            } else {
                throw ticpp::Exception("---> No angle found in parseMeshGeometry");
            }

            Quaternion quat;
            setQuaternion(quat,axis,angle);


            Assimp::Importer importer;


            importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_LINE | aiPrimitiveType_POINT);
            importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_NORMALS | aiComponent_MESHES);
            // And have it read the given file with some example postparseing
            // Usually - if speed is not the most important aspect for you - you'll
            // propably to request more postparseing than we do in this example.
            const aiScene* scene = importer.ReadFile( fileName.string(),
                                   aiProcess_JoinIdenticalVertices  |
                                   aiProcess_SortByPType |
                                   aiProcess_Triangulate /*| aiProcess_GenNormals*/);

            // If the import failed, report it
            if(!scene) {
                throw ticpp::Exception("---> File import failed in parseMeshGeometry: for file" + fileName.string() );
            }

            MeshData * meshData = new MeshData();

            if(!meshData->setup(importer,scene, scale_factor,quat,trans)) {
                throw ticpp::Exception("---> Imported Mesh (with Assimp) could not be setup internally");
            }

            // Build Geometry
            pMeshGeom = std::shared_ptr<MeshGeometry >(new MeshGeometry(meshData));

            if(mesh->HasAttribute("writeToLog")) {
                bool writeToLog;
                if(!Utilities::stringToType<bool>(writeToLog, mesh->GetAttribute("writeToLog"))) {
                    throw ticpp::Exception("---> String conversion in parseMeshGeometry: angleDegree failed");
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
                 for(auto & b  : m_bodyListGroup) {
                    b.m_scale = scale_factor;
                    if(m_allocateBodies){
                        b.m_body->m_geometry = pMeshGeom;
                    }
                }
            }

        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'Mesh' has no implementation in the parser"));
        }
    }
    virtual void parseGlobalGeomId( ticpp::Element * globalGeomId ) {

        std::string distribute = globalGeomId->GetAttribute("distribute");
        if(distribute == "uniform") {

            unsigned int id;
            if(!Utilities::stringToType<unsigned int>(id,globalGeomId->GetAttribute("id"))) {
                throw ticpp::Exception("---> String conversion in parseGlobalGeomId: id failed");
            }

            auto it = findGlobalGeomId(id);
            // it->second is the GeometryType in RigidBody
            if(it == this->m_pGlobalGeometries->end()) {
                LOG(m_pSimulationLog,"---> Geometry with id: " << id << " not found in global geometry list!" <<std::endl;);
                throw ticpp::Exception("---> Geometry search in parseGlobalGeomId: failed!");
            }

            for(auto & b : m_bodyListGroup) {
                GetScaleOfGeomVisitor vis(b.m_scale);
                boost::apply_visitor(vis, it->second);
                b.m_body->m_geometry = it->second;
                b.m_body->m_globalGeomId = id;
                //LOG(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(m_bodyListGroup[i]) << ", GlobalGeomId: " << id <<  std::endl);
            }

        } else if(distribute == "linear") {

            unsigned int startId;
            if(!Utilities::stringToType<unsigned int>(startId,globalGeomId->GetAttribute("startId"))) {
                throw ticpp::Exception("---> String conversion in parseGlobalGeomId: startId failed");
            }

            if(startId == 0) {
                throw ticpp::Exception("---> parseGlobalGeomId: a global geometry startId: 0 is not allowed!");
                // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
            }

            for(auto & b : m_bodyListGroup) {
                unsigned int id = startId + (b.m_initState.m_id - m_startIdGroup); //make linear offset from start of this group
                auto it = findGlobalGeomId(id);
                // it->second is the GeometryType in RigidBody
                if(it == this->m_pGlobalGeometries->end()) {
                    LOG(m_pSimulationLog,"---> parseGlobalGeomId: Geometry with id: " << startId+id << " not found in global geometry list!" <<std::endl;);
                    throw ticpp::Exception("---> parseGlobalGeomId: Geometry search failed!");
                }

                GetScaleOfGeomVisitor vis(b.m_scale);
                boost::apply_visitor(vis, it->second);
                b.m_body->m_geometry = it->second;
                b.m_body->m_globalGeomId = id;
                LOG(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_body) << ", GlobalGeomId: " << id <<  std::endl);

            }


        } else if(distribute == "random") {

            unsigned int startId;
            if(!Utilities::stringToType<unsigned int>(startId,globalGeomId->GetAttribute("startId"))) {
                throw ticpp::Exception("---> String conversion in parseGlobalGeomId: startId failed");
            }

            if(startId == 0) {
                throw ticpp::Exception("---> parseGlobalGeomId: a global geometry startId: 0 is not allowed!");
                // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
            }

            unsigned int endId;
            if(!Utilities::stringToType<unsigned int>(endId,globalGeomId->GetAttribute("endId"))) {
                throw ticpp::Exception("---> String conversion in parseGlobalGeomId: endId failed");
            }
            if(startId > endId) {
                throw ticpp::Exception("---> addToGlobalGeomList:  startId > endId  is not allowed!");
                // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
            }
            unsigned int seed;
            if(!Utilities::stringToType<unsigned int>(seed,globalGeomId->GetAttribute("seed"))) {
                throw ticpp::Exception("---> String conversion in parseGlobalGeomId: seed failed");
            }

            RandomGenType gen(seed);
            std::uniform_int_distribution<unsigned int> uni(startId,endId);

            RigidBodyIdType diffId = m_startIdGroup; // id to generate the correct amount of random values!

            unsigned int id = uni(gen); // generate first value
            for(auto & b: m_bodyListGroup) {

                id = Utilities::genRandomValues(id,gen,uni,b.m_initState.m_id - diffId);
                auto it = findGlobalGeomId(id);
                // it->second is the GeometryType in RigidBody
                if(it == this->m_pGlobalGeometries->end()) {
                    LOG(m_pSimulationLog,"---> Geometry with id: " << id << " not found in global geometry list!" <<std::endl;);
                    throw ticpp::Exception("---> Geometry search in parseGlobalGeomId: failed!");
                }

                GetScaleOfGeomVisitor vis(b.m_scale);
                boost::apply_visitor(vis, it->second);
                b.m_body->m_geometry = it->second;
                b.m_body->m_globalGeomId = id;
                LOG(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_body) << ", GlobalGeomId: " << id <<  std::endl);
            }


        } else {
            throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'GlobalGeomId' has no implementation in the parser"));
        }



    }
    ///  ================================================================================ Geometries

    typename DynamicsSystemType::GlobalGeometryMapType::iterator findGlobalGeomId(unsigned int id) {
        return m_pGlobalGeometries->find(id);
    }

    template<typename T>
    void addToGlobalGeomList(unsigned int id,  std::shared_ptr<T> ptr) {



        std::pair<typename DynamicsSystemType::GlobalGeometryMapType::iterator, bool> ret =
            m_pGlobalGeometries->insert(typename DynamicsSystemType::GlobalGeometryMapType::value_type( id, ptr) );
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

    /// Dynamic Properties ==============================================================================
    virtual void parseDynamicProperties( ticpp::Node * dynProp) {
        LOG(m_pSimulationLog,"---> Parse DynamicProperties ..."<<std::endl;);

        // DynamicState has already been parsed for the group!

        // apply first to all bodies :-)
        for(auto & b: m_bodyListGroup) {
            b.m_body->m_eState = m_eBodiesState;
        }

        if(m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {
            parseDynamicPropertiesSimulated(dynProp);
        } else if(m_eBodiesState == RigidBodyType::BodyState::STATIC) {
            parseDynamicPropertiesNotSimulated(dynProp);
        }
    }
    virtual void parseDynamicState(ticpp::Node * dynProp){

        ticpp::Element * element = dynProp->FirstChild("DynamicState")->ToElement();
        std::string type = element->GetAttribute("type");
        if(type == "simulated") {
            m_eBodiesState =  RigidBodyType::BodyState::SIMULATED;
        } else if(type == "not simulated" || type == "static") {
            m_eBodiesState =  RigidBodyType::BodyState::STATIC;
        } else if(type == "animated") {
            m_eBodiesState =  RigidBodyType::BodyState::ANIMATED;
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
        }

    }
    virtual void parseDynamicPropertiesSimulated( ticpp::Node * dynProp) {
        ticpp::Element *element = nullptr;
        std::string distribute;

        if(!m_parseOnlyVisualizationProperties) {
            // First allocate a new SolverDate structure
            for(auto & b : m_bodyListGroup)  {
                b.m_body->m_pSolverData = new RigidBodySolverDataType();
            }
            // Mass ============================================================
            element = dynProp->FirstChild("Mass")->ToElement();
            distribute = element->GetAttribute("distribute");
            if(distribute == "uniform") {

                double mass = element->GetAttribute<double>("value");

                for(auto & b : m_bodyListGroup) {
                    b.m_body->m_mass = mass;
                }


            } else {
                throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'Mass' has no implementation in the parser"));
            }

            // InertiaTensor ============================================================
            element = dynProp->FirstChild("InertiaTensor")->ToElement();
            std::string type = element->GetAttribute("type");
            if(type == "homogen") {
                for(auto & b : m_bodyListGroup) {
                    InertiaTensor::calculateInertiaTensor(b.m_body);
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

                for(auto & b : m_bodyListGroup) {
                    b.m_body->m_eMaterial = eMaterial;
                }
            } else {
                throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'Material' has no implementation in the parser"));
            }
        }

        // InitialPosition ============================================================
        ticpp::Node * node = dynProp->FirstChild("InitialCondition",true);
        parseInitialCondition(node);
    }
    virtual void parseDynamicPropertiesNotSimulated( ticpp::Node * dynProp) {

        // InitialPosition ============================================================
        ticpp::Node * node = dynProp->FirstChild("InitialCondition",true);
        parseInitialCondition(node);


        ticpp::Element *element = nullptr;
        std::string distribute;

        if(!m_parseOnlyVisualizationProperties) {
            element = dynProp->FirstChild("Material")->ToElement();
            distribute = element->GetAttribute("distribute");
            if(distribute == "uniform") {
                typename RigidBodyType::BodyMaterialType eMaterial = 0;

                if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(eMaterial, element->GetAttribute("id"))) {
                    throw ticpp::Exception("---> String conversion in Material: id failed");
                }

                for(auto & b : m_bodyListGroup) {
                    b.m_body->m_eMaterial = eMaterial;
                }
            } else {
                throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'Material' has no implementation in the parser"));
            }
        }

    }

    virtual void parseInitialCondition(ticpp::Node * initCondNode) {
        ticpp::Element *element = nullptr;
        std::string distribute;

        element = initCondNode->ToElement();
        std::string type  = element->GetAttribute("type");
        if( type == "file") {
            throw ticpp::Exception(" Not yet implemented");
        } else if (type == "posvel") {

            element = initCondNode->FirstChild("InitialPosition")->ToElement();
            distribute = element->GetAttribute("distribute");

            if(distribute == "linear") {
                parseInitialPositionLinear(element);
            } else if(distribute == "grid") {
                parseInitialPositionGrid(element);
            } else if(distribute == "posaxisangle") {
                parseInitialPositionPosAxisAngle(element);
            } else if(distribute == "transforms") {
                parseInitialPositionTransforms(element);
            } else if(distribute == "generalized") {
                //parseInitialPositionGeneralized(element);
            } else if(distribute == "none") {
                // does nothing leaves the zero state pushed!
            } else {
                throw ticpp::Exception("---> The attribute 'distribute' '" + distribute + std::string("' of 'InitialPosition' has no implementation in the parser"));
            }

            //Initial Velocity
            if(!m_parseOnlyVisualizationProperties && m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {
                ticpp::Node * initVel = initCondNode->FirstChild("InitialVelocity",false);
                if(initVel) {
                    element = initVel->ToElement();
                    distribute = element->GetAttribute("distribute");

                    if(distribute == "transrot") {
                        parseInitialVelocityTransRot(element);
                    } else if(distribute == "generalized") {
                        //parseInitialVelocityGeneralized(element);
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

        if(m_allocateBodies){
            for(auto & b: m_bodyListGroup){
                std::cout <<"state::" << b.m_initState.m_q.transpose() << " , " << b.m_initState.m_u.transpose()  << std::endl;
                InitialConditionBodies::applyBodyStateTo(b.m_initState,b.m_body);
            }
        }
    }
    virtual void parseInitialPositionLinear(ticpp::Element * initCond) {

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

        InitialConditionBodies::setupPositionBodiesLinear(m_bodyListGroup,m_startIdGroup,pos,dir,dist,jitter,delta,seed);

    }
    virtual void parseInitialPositionGrid(ticpp::Element * initCond) {

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

        InitialConditionBodies::setupPositionBodiesGrid(m_bodyListGroup,m_startIdGroup,gridX,gridY,dist,trans,jitter,delta, seed);
    }
    virtual void parseInitialPositionFile(DynamicsState & state, ticpp::Element * initCond) {
//        m_SimBodyInitStates.push_back(DynamicsState((unsigned int)m_bodyListGroup.size()));
//
//        boost::filesystem::path name =  initCond->GetAttribute<std::string>("relpath");
//
//        boost::filesystem::path filePath = m_currentParseFileDir / name;
//        InitialConditionBodies::setupPositionBodiesFromFile(state,filePath);
    }
    virtual void parseInitialPositionPosAxisAngle(ticpp::Element * initCond) {

        unsigned int consumedValues = 0;
        Vector3 pos; Vector3 axis; PREC angle;

        auto bodyIt = m_bodyListGroup.begin();
        ASSERTMSG(bodyIt != m_bodyListGroup.end(),"no bodies in list");

        // Iterate over all values in the list
        ticpp::Iterator< ticpp::Element > valueElem("Value");
        for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++) {

            if(consumedValues >= m_bodyListGroup.size()) {
                LOG(m_pSimulationLog,"---> InitialPositionPosAxisAngle: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            }else if(consumedValues != bodyIt->m_initState.m_id - m_startIdGroup){
                // this value does not correspond to the linear offset from the start
                consumedValues++; continue;
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

            InitialConditionBodies::setupPositionBodyPosAxisAngle( bodyIt->m_initState, pos, axis, angle);
            ++consumedValues; ++bodyIt;
        }

        if(consumedValues < m_bodyListGroup.size()) {
            LOG(m_pSimulationLog,"---> InitialPositionPosAxisAngle: You specified to little values, -> applying last to all remainig bodies ..."<<std::endl;);
            auto itEnd = m_bodyListGroup.end();
            for(; bodyIt !=  itEnd; ++bodyIt) {
                InitialConditionBodies::setupPositionBodyPosAxisAngle(bodyIt->m_initState, pos, axis, angle);

            }
        }
    }
    virtual void parseInitialPositionTransforms(ticpp::Element * initCond) {

        unsigned int consumedValues = 0;

        auto bodyIt = m_bodyListGroup.begin();
        ASSERTMSG(bodyIt != m_bodyListGroup.end(), "no bodies in list");

        Quaternion q_KI, q_BK;
        Vector3 I_r_IK, K_r_KB;
        Matrix33 Rot_KI; // Temp

        // Iterate over all values in the list
        ticpp::Iterator< ticpp::Element > valueElem("Value");
        for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++) {

            if(consumedValues >= m_bodyListGroup.size()) {
                LOG(m_pSimulationLog,"---> InitialPositionTransforms: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            }else if(consumedValues != bodyIt->m_initState.m_id - m_startIdGroup){
                // this value does not correspond to the linear offset from the start
                ++consumedValues; continue;
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
            bodyIt->m_initState.m_q.head<3>() = I_r_IK;
            bodyIt->m_initState.m_q.tail<4>() = q_KI;

            ++consumedValues; ++bodyIt;
        }

        if(consumedValues < m_bodyListGroup.size()) {
            LOG(m_pSimulationLog,"---> InitialPositionTransforms: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
            auto itEnd = m_bodyListGroup.end();
            for(; bodyIt !=  itEnd; ++bodyIt) {
                bodyIt->m_initState.m_q.head<3>() = I_r_IK;
                bodyIt->m_initState.m_q.tail<4>() = q_KI;
            }
        }
    }
    virtual void parseInitialVelocityTransRot(ticpp::Element * initCond) {
        unsigned int consumedValues = 0;
        Vector3 transDir,rotDir;
        PREC rot,vel;

        auto bodyIt = m_bodyListGroup.begin();
        ASSERTMSG(bodyIt != m_bodyListGroup.end(),"no bodies in list");

        // Iterate over all values in the list
        ticpp::Iterator< ticpp::Element > valueElem("Value");
        for ( valueElem = valueElem.begin( initCond ); valueElem != valueElem.end(); valueElem++) {

             if(consumedValues >= m_bodyListGroup.size()) {
                LOG(m_pSimulationLog,"---> InitialPositionTransforms: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            }else if(consumedValues != bodyIt->m_initState.m_id - m_startIdGroup){
                // this value does not correspond to the linear offset from the start
                ++consumedValues; continue;
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

            bodyIt->m_initState.m_u.head<3>() = transDir*vel;
            bodyIt->m_initState.m_u.tail<3>() = rotDir*rot;

            ++consumedValues; ++bodyIt;
        }

        if(consumedValues < m_bodyListGroup.size()) {
            LOG(m_pSimulationLog,"---> InitialVelocityTransRot: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
            auto itEnd = m_bodyListGroup.end();
            for(; bodyIt !=  itEnd; ++bodyIt) {
                 bodyIt->m_initState.m_u.head<3>() = transDir*vel;
                 bodyIt->m_initState.m_u.tail<3>() = rotDir*rot;
            }
        }

    }
    ///  =============================================================================== Dynamic Properties

    // virtual
    virtual void parseVisualization( ticpp::Node * visualizationNode) {

    }
    ///  ====================================================================================================== SceneObjects


    virtual void checkFileExists(boost::filesystem::path file) {
        if( !boost::filesystem::exists(file) ) {
            throw ticpp::Exception("---> The file ' " + file.string() + "' does not exist!");
        }
    }


    // Parser Flags =====================================
    bool m_parseSceneSettings; ///< Parse SceneSettings (default= true)

    // SceneObjects
    bool m_parseSceneObjects;  ///< Parse SceneObjects, (default= true)

        bool m_parseSelectiveBodyIds;                ///< Use the m_bodyIdRange to only load the selective ids in the group which use enableSelectiveIds="true"
        bool m_parseAllBodiesNonSelGroup;       ///< Parse all bodies in groups where m_bodyIdRange is not applied (enableSelectiveIds="false) (default= true)

        bool m_parseOnlySimBodies;         ///< Parses only simulated bodies (default= false)
        bool m_parseOnlyVisualizationProperties;     ///< Parse only initial condition, add no bodies to m_pDynSys. Playback Manager also has this SceneParser but does not need DynamicsStuff.

    // ===================================================

    BodyRange m_bodyIdRangeTmp;                     ///< Range of bodies, temporary for load of all bodies

    BodyRange m_bodyIdRange;                        ///< Range of body ids, original list which is handed to parseScene

    typename BodyRange::iterator m_startRangeIdIt;  ///< Current iterator which marks the start for the current group in m_bodyIdRange

    BodyRange * m_bodyIdRangePtr;                   ///< Switches between m_bodyIdRangeTmp/m_bodyIdRange, depending on parsing state

    unsigned int m_parsedInstancesGroup;            ///< Number of instances generated in the current group
    RigidBodyIdType m_startIdGroup;                  ///< Start id of the current group smaller or equal to *m_startRangeIdIt


    std::shared_ptr<DynamicsSystemType> m_pDynSys;

    boost::filesystem::path m_currentParseFilePath;
    boost::filesystem::path m_currentParseFileDir;
    ticpp::Document m_xmlDoc;
    const ticpp::Node * m_xmlRootNode;

    Logging::Log * m_pSimulationLog;
    std::stringstream logstream;

    unsigned int m_nSimBodies, m_nBodies;
    typedef std::unordered_map<unsigned int,unsigned int> GroupToNBodyType;
    GroupToNBodyType m_groupIdToNBodies;
    unsigned int m_globalMaxGroupId; // Group Id used to build a unique id!

    // Temprary structures for each sublist (groupid=?) of rigid bodies
    typename RigidBodyType::BodyState m_eBodiesState;     ///< Used to parse a RigidBody Node

    struct BodyData{
        BodyData( RigidBodyType * p, const RigidBodyIdType & id, const Vector3 & s = Vector3(1,1,1))
            : m_body(p),m_scale(s),m_initState(id){}
        RigidBodyType* m_body ; // might be also zero (if we dont need the whole body for visualization only)
        Vector3 m_scale;
        RigidBodyState m_initState;
    };

    typename std::vector<BodyData> m_bodyListGroup; ///< Used to parse a RigidBody Node
    bool m_allocateBodies;

    // Pointer to maps where to insert the results, can be overwritten in derived classes
    GlobalGeometryMapType        * m_pGlobalGeometries = nullptr;
    RigidBodyStatesContainerType * m_pInitStates = nullptr;
    RigidBodySimContainerType    * m_pSimBodies = nullptr;
    RigidBodyStaticContainerType * m_pBodies = nullptr;

    // Random Distribution Types
    template<typename T>
    using UniformDistType = std::uniform_real_distribution<T>;

};


#endif
