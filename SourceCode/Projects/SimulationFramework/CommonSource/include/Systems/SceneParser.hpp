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

//#define TIXML_USE_TICPP
//#include "ticpp/ticpp.h"
//#include "tinyxml.h"

#include "pugixml.hpp"

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"
#include "LogDefines.hpp"
#include "Exception.hpp"

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


#define CHECK_XMLNODE( _node_ , _nodename_ ) \
    if( ! _node_ ){ \
        THROWEXCEPTION("XML Node: " << _nodename_ << " does not exist!");  \
    }

#define GET_XMLCHILDNODE_CHECK( _childnode_ , _childname_ , _node_ ) \
    _childnode_ = _node_.child( _childname_ ); \
    CHECK_XMLNODE( _childnode_ , _childname_)


#define SKIPLOG( _logptr_ , _message_ )  ( * (_logptr_) ) <<  _message_  ;

#define  DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE  \
    using ParserType = TParser; \
    using XMLNodeType = typename TParser::XMLNodeType;\
    using XMLAttributeType = typename TParser::XMLAttributeType;\
    using DynamicsSystemType = typename ParserType::DynamicsSystemType; \
    template<typename T> using UniformDistType = typename ParserType::template UniformDistType<T>;\


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

namespace ParserModules {


/** Parses TimestepperSettings, InclusionSolverSettings, RecorderSettings */
template<typename TParser>
class SettingsModule {
private:

    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
    DEFINE_LAYOUT_CONFIG_TYPES

    using RecorderSettingsType        = typename DynamicsSystemType::RecorderSettingsType;
    using TimeStepperSettingsType     = typename DynamicsSystemType::TimeStepperSettingsType;
    using InclusionSolverSettingsType = typename DynamicsSystemType::InclusionSolverSettingsType;

    RecorderSettingsType * m_recorderSettings;
    TimeStepperSettingsType * m_timestepperSettings;
    InclusionSolverSettingsType * m_inclusionSettings;

    ParserType * m_parser;

    void checkFileExists(boost::filesystem::path file) {
        if( !boost::filesystem::exists(file) ) {
            THROWEXCEPTION("---> The file ' " + file.string() + "' does not exist!");
        }
    }

public:

    SettingsModule(ParserType * p, RecorderSettingsType * r, TimeStepperSettingsType * t, InclusionSolverSettingsType * i)
        :m_parser(p),m_recorderSettings(r),m_timestepperSettings(t), m_inclusionSettings(i) {};

    void parse(XMLNodeType sceneSettings) {

        LOG(m_parser->m_pSimulationLog, "---> SettingsModule: parsing ..."<<std::endl;)

        XMLNodeType node;
        XMLAttributeType att;

        XMLNodeType timestepNode = sceneSettings.child("TimeStepperSettings");

        if( m_timestepperSettings ) {

            CHECK_XMLNODE(timestepNode,"TimeStepperSettings");

            if(!Utilities::stringToType<PREC>(m_timestepperSettings->m_deltaT, timestepNode.attribute("deltaT").value())) {
                THROWEXCEPTION("---> String conversion in SceneSettings: deltaT failed");
            }
            if(m_inclusionSettings) {
                m_inclusionSettings->m_deltaT = m_timestepperSettings->m_deltaT;
            }
            if(!Utilities::stringToType<PREC>(m_timestepperSettings->m_endTime, timestepNode.attribute("endTime").value())) {
                THROWEXCEPTION("---> String conversion in SceneSettings: endTime failed");
            }

            auto node = timestepNode.child("SimulateFromReference");
            if(node) {
                bool enabled = false;
                if(!Utilities::stringToType<bool>(enabled, node.attribute("enabled").value())) {
                    THROWEXCEPTION("---> String conversion in SimulateFromReference: enable failed");
                }
                if(enabled) {
                    std::string type = node.attribute("type").value();
                    if(type == "useStates") {
                        m_timestepperSettings->m_eSimulateFromReference = TimeStepperSettings::USE_STATES;
                    } else if(type == "continue") {
                        m_timestepperSettings->m_eSimulateFromReference = TimeStepperSettings::CONTINUE;
                    } else {
                        THROWEXCEPTION("---> String conversion in SimulateFromReference: type. The type '" + type + std::string("' has no implementation in the parser"));
                    }
                    m_timestepperSettings->m_simStateReferenceFile = node.attribute("file").value();
                    checkFileExists(m_timestepperSettings->m_simStateReferenceFile);
                } else {
                    m_timestepperSettings->m_eSimulateFromReference = TimeStepperSettings::NONE;
                }
            }
        } else {
            SKIPLOG(m_parser->m_pSimulationLog, "---> SettingsModule: skipped TimeStepperSettings ..."<<std::endl;)
        }


        if(m_inclusionSettings) {
            auto node = timestepNode.child("InclusionSolverSettings");
            CHECK_XMLNODE(node,"InclusionSolverSettings");

            if(!Utilities::stringToType<PREC>(m_inclusionSettings->m_alphaJORProx, node.attribute("alphaJORProx").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: alphaJORProx failed");
            }
            if(!Utilities::stringToType<PREC>(m_inclusionSettings->m_alphaSORProx, node.attribute("alphaSORProx").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: alphaJORProx failed");
            }
            if(!Utilities::stringToType<unsigned int>(m_inclusionSettings->m_MaxIter, node.attribute("maxIter").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: maxIter failed");
            }

            att = node.attribute("minIter");
            if(att) {
                if(!Utilities::stringToType<unsigned int>(m_inclusionSettings->m_MinIter, att.value())) {
                    THROWEXCEPTION("---> String conversion in InclusionSolverSettings: minIter failed");
                }
            } else {
                m_inclusionSettings->m_MinIter = 0;
            }

            att = node.attribute("convergenceMethod");
            if(att) {
                std::string method = att.value();
                if(method == "InLambda") {
                    m_inclusionSettings->m_eConvergenceMethod = InclusionSolverSettingsType::InLambda;
                } else if (method == "InVelocity") {
                    m_inclusionSettings->m_eConvergenceMethod = InclusionSolverSettingsType::InVelocity;
                } else if (method == "InVelocityLocal") {
                    m_inclusionSettings->m_eConvergenceMethod = InclusionSolverSettingsType::InVelocityLocal;
                } else if (method == "InEnergyVelocity") {
                    m_inclusionSettings->m_eConvergenceMethod = InclusionSolverSettingsType::InEnergyVelocity;
                } else if (method == "InEnergyLocalMix") {
                    m_inclusionSettings->m_eConvergenceMethod = InclusionSolverSettingsType::InEnergyLocalMix;
                } else {
                    THROWEXCEPTION("---> String conversion in InclusionSolverSettings: convergenceMethod failed: not a valid setting");
                }
            } else {
                m_inclusionSettings->m_eConvergenceMethod = InclusionSolverSettingsType::InVelocity;
            }

            if(!Utilities::stringToType<PREC>(m_inclusionSettings->m_AbsTol, node.attribute("absTol").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: absTol failed");
            }
            if(!Utilities::stringToType<PREC>(m_inclusionSettings->m_RelTol, node.attribute("relTol").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: relTol failed");
            }

            att = node.attribute("computeResidual");
            if(att) {
                if(!Utilities::stringToType<bool>(m_inclusionSettings->m_bComputeResidual, att.value())) {
                    THROWEXCEPTION("---> String conversion in InclusionSolverSettings: computeResidual failed");
                }
            }

            att = node.attribute("isFiniteCheck");
            if(att) {
                if(!Utilities::stringToType<bool>(m_inclusionSettings->m_bIsFiniteCheck, att.value())) {
                    THROWEXCEPTION("---> String conversion in InclusionSolverSettings: isFiniteCheck failed");
                }
            }

            std::string method = node.attribute("method").value();
            if(method == "JOR") {
                m_inclusionSettings->m_eMethod = InclusionSolverSettingsType::JOR;
            } else if (method == "SOR") {
                m_inclusionSettings->m_eMethod = InclusionSolverSettingsType::SOR_CONTACT;
            } else if (method == "SORContact") {
                m_inclusionSettings->m_eMethod = InclusionSolverSettingsType::SOR_CONTACT;
            } else if (method == "SORFull") {
                m_inclusionSettings->m_eMethod = InclusionSolverSettingsType::SOR_FULL;
            } else {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: method failed: not a valid setting");
            }

            if(!Utilities::stringToType<bool>(m_inclusionSettings->m_bUseGPU, node.attribute("useGPU").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: useGPU failed");
            }

            att = node.attribute("useGPUID");
            if(att) {
                if(!Utilities::stringToType<int>(m_inclusionSettings->m_UseGPUDeviceId, att.value())) {
                    THROWEXCEPTION("---> String conversion in InclusionSolverSettings: useGPU failed");
                }
                if(m_inclusionSettings->m_UseGPUDeviceId <0) {
                    m_inclusionSettings->m_UseGPUDeviceId = 0;
                }
            }
        } else {
            SKIPLOG(m_parser->m_pSimulationLog, "---> SettingsModule: skipped InclusionSolverSettings ..."<<std::endl;)
        }


        if(m_recorderSettings) {
            node = sceneSettings.child("RecorderSettings");
            CHECK_XMLNODE(node,"RecorderSettings");
            std::string method = node.attribute("recorderMode").value();
            if(method == "everyTimeStep") {
                m_recorderSettings->setMode(RecorderSettings::RECORD_EVERY_STEP);
            } else if (method == "everyXTimeStep") {
                m_recorderSettings->setMode(RecorderSettings::RECORD_EVERY_X_STEP);
                PREC fps;
                if(!Utilities::stringToType<double>(fps, node.attribute("statesPerSecond").value())) {
                    THROWEXCEPTION("---> String conversion in RecorderSettings: statesPerSecond failed");
                }
                m_recorderSettings->setEveryXTimestep(fps,m_timestepperSettings->m_deltaT);
            } else if (method == "noOutput" || method=="none" || method=="nothing") {
                m_recorderSettings->setMode(RecorderSettings::RECORD_NOTHING);
            } else {
                THROWEXCEPTION("---> String conversion in RecorderSettings: recorderMode failed: not a valid setting");
            }

        } else {
            SKIPLOG(m_parser->m_pSimulationLog, "---> SettingsModule: skipped RecorderSettings ..."<<std::endl;)
        }
    }
};



template<typename TParser>
class GlobalGeomModule {
private:
    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
    DEFINE_LAYOUT_CONFIG_TYPES

    using GlobalGeometryMapType = typename DynamicsSystemType::GlobalGeometryMapType;
    GlobalGeometryMapType * m_globalGeometries;
public:
    GlobalGeomModule(ParserType * p, GlobalGeometryMapType * g): m_globalGeometries(g) {};
    void parse() {
        std::cout << "parse GlobalGeomModule: @" << this << std::endl;
    }
};

template<typename TParser>
class ContactParamModule {
private:

    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
    DEFINE_LAYOUT_CONFIG_TYPES

    using ContactParameterMapType = typename DynamicsSystemType::ContactParameterMapType;
    ContactParameterMapType * m_contactParams;

    using RigidBodyType = typename DynamicsSystemType::RigidBodyType;

    ParserType * m_parser;

public:
    ContactParamModule(ParserType * p, ContactParameterMapType * c): m_parser(p), m_contactParams(c) {};

    void parse(XMLNodeType sceneSettings){

        LOG(m_parser->m_pSimulationLog,"---> ContactParamModule: parsing ..."<<std::endl;);

        if( !m_contactParams ){
            SKIPLOG(m_parser->m_pSimulationLog, "---> ContactParamModule: skipping ..."<<std::endl;)
            return;
        }

        XMLNodeType paramMap = sceneSettings.child("ContactParameterMap");

        XMLNodeType node;
        if(paramMap) {
            node = paramMap.child("ContactParameterStandard");
            if(node) {
                parseContactParameter(node, true);
            }
            for (XMLNodeType it = paramMap.child("ContactParameter"); it; it = it.next_sibling("ContactParameter")){
               parseContactParameter(it);
            }
        }
    }

    void parseContactParameter(XMLNodeType contactParam, bool stdMaterial=false){


        typename RigidBodyType::BodyMaterialType material1,material2;
        if(!stdMaterial){
            if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(material1, contactParam.attribute("materialId1").value())) {
                THROWEXCEPTION("---> String conversion in ContactParameter: materialId1 failed");
            }
            if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(material2, contactParam.attribute("materialId2").value())) {
                THROWEXCEPTION("---> String conversion in ContactParameter: materialId2 failed");
            }
        }
        //LOG(m_parser->m_pSimulationLog,"---> matId1: " << material1 << " matId2: " << material2 <<std::endl;);
//
//        std::string type = contactParam.attribute("type").value();
//        if(type == "UCF" || type == "UCFD" || type == "UCFDD"){
//
//            PREC mu,epsilonN,epsilonT;
//            ContactParameter contactParameter;
//
//            if(!Utilities::stringToType<PREC>(mu, contactParam.attribute("mu").value())) {
//                throw ticpp::Exception("---> String conversion in ContactParameter: mu failed");
//            }
//            if(!Utilities::stringToType<PREC>(epsilonN, contactParam.attribute("epsilonN").value())) {
//                throw ticpp::Exception("---> String conversion in ContactParameter: epsilonN failed");
//            }
//            if(!Utilities::stringToType<PREC>(epsilonT, contactParam.attribute("epsilonT").value())) {
//                throw ticpp::Exception("---> String conversion in ContactParameter: epsilonT failed");
//            }
//
//            if(type == "UCFD"){
//                PREC invDampingN, invDampingT;
//                if(!Utilities::stringToType<PREC>(invDampingN, contactParam.attribute("invDampingN").value())) {
//                    throw ticpp::Exception("---> String conversion in ContactParameter: invDampingN failed");
//                }
//                if(!Utilities::stringToType<PREC>(invDampingT, contactParam.attribute("invDampingT").value())) {
//                    throw ticpp::Exception("---> String conversion in ContactParameter: invDampingT failed");
//                }
//
//                contactParameter = ContactParameter::createParams_UCFD_ContactModel(epsilonN,epsilonT,mu,invDampingN,invDampingT);
//
//
//            }else if(type == "UCFDD"){
//
//                PREC invDampingN, gammaMax, epsilon, invDampingTFix;
//                if(!Utilities::stringToType<PREC>(invDampingN, contactParam.attribute("invDampingN").value())) {
//                    throw ticpp::Exception("---> String conversion in ContactParameter: invDampingN failed");
//                }
//
//                if(!Utilities::stringToType<PREC>(invDampingTFix, contactParam.attribute("invDampingTFix").value())) {
//                    throw ticpp::Exception("---> String conversion in ContactParameter: invDampingTFix failed");
//                }
//                if(!Utilities::stringToType<PREC>(gammaMax, contactParam.attribute("gammaMax").value())) {
//                    throw ticpp::Exception("---> String conversion in ContactParameter: gamma_max failed");
//                }
//                if(!Utilities::stringToType<PREC>(epsilon, contactParam.attribute("epsilon").value())) {
//                    throw ticpp::Exception("---> String conversion in ContactParameter: epsilon failed");
//                }
//
//                contactParameter = ContactParameter::createParams_UCFDD_ContactModel(epsilonN,epsilonT,mu,
//                                                                                     invDampingN,invDampingTFix,
//                                                                                     gammaMax,epsilon);
//            }
//            else{
//                contactParameter = ContactParameter::createParams_UCF_ContactModel(epsilonN,epsilonT,mu);
//            }
//
//            if(stdMaterial){
//                LOG(m_pSimulationLog,"---> Add ContactParameter standart"<<std::endl;);
//                m_pDynSys->m_ContactParameterMap.setStandardValues(contactParameter);
//            }
//            else{
//                LOG(m_pSimulationLog,"---> Add ContactParameter standart of id="<<material1<<" to id="<<material2<<std::endl;);
//                if(!m_pDynSys->m_ContactParameterMap.addContactParameter(material1,material2,contactParameter)){
//                    throw ticpp::Exception("---> Add ContactParameter failed");
//                }
//            }
//
//
//        }
//        else{
//            throw ticpp::Exception("---> String conversion in ContactParameter: type failed");
//        }
    }

};


template<typename TParser>
class BodyVisModule {
private:
    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
    DEFINE_LAYOUT_CONFIG_TYPES

public:
    BodyVisModule(ParserType * p) {};
    void parse() {
        std::cout << "parse BodyVisModule: @" << this << std::endl;
    }
};
template<typename TParser>
class InitStatesModule {
private:
    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
    DEFINE_LAYOUT_CONFIG_TYPES

    using RigidBodyStatesContainerType = typename DynamicsSystemType::RigidBodyStatesContainerType;
    RigidBodyStatesContainerType * m_initStates;
public:
    InitStatesModule(ParserType * p,RigidBodyStatesContainerType * c): m_initStates(c) {};
    void parse() {
        std::cout << "parse InitStatesModule: @" << this << std::endl;
    }
};



/** SceneParser Options */
struct BodyModuleParserOptions {
    BodyModuleParserOptions() = default;
    BodyModuleParserOptions(const BodyModuleParserOptions& o) = default;
    BodyModuleParserOptions(BodyModuleParserOptions&& o) = default;
    BodyModuleParserOptions& operator=(const BodyModuleParserOptions& o) = default;
    BodyModuleParserOptions& operator=(BodyModuleParserOptions&& o) = default;

    using BodyRangeType = Range<RigidBodyIdType>;
    BodyRangeType m_bodyIdRange;       ///< Range of body ids, original list which is handed to parseScene
    bool m_parseAllBodiesNonSelGroup = true;  ///< Parse all bodies in groups where m_bodyIdRange is not applied (enableSelectiveIds="false) (default= true)
    bool m_parseOnlySimBodies = false;         ///< Parses only simulated bodies (default= false)
};

template<typename TParser,
         typename TGlobalGeomModule,
         typename TInitStatesModule,
         typename TVisModule>
class BodyModule {
private:

    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
    DEFINE_LAYOUT_CONFIG_TYPES

    using GlobalGeomModType  = TGlobalGeomModule;
    using InitStatesModType  = TInitStatesModule;

    using RigidBodyType = typename DynamicsSystemType::RigidBodyType;
    using RigidBodySimContainerType = typename DynamicsSystemType::RigidBodySimContainerType ;

public:

    using VisModType = TVisModule ;

    BodyModule(ParserType * p, const GlobalGeomModType * g,  InitStatesModType * is, std::unique_ptr<VisModType> i,
               RigidBodySimContainerType * c)
        : pGeomMod(g),pVisMod(std::move(i)),pInitStatesMod(is), pSimBodies(c) {};

    void parse() {
        std::cout << "parse BodyModule: @" << this << std::endl;
        if(pGeomMod) {
            pGeomMod->access();
        }
        if(pVisMod) {
            pVisMod->parse();
        }
        if(pInitStatesMod) {
            pInitStatesMod->parse();
        }
    }

    template<typename TParserOptions>
    void setParsingOptions(TParserOptions&& o) {
        m_parsingOptions = std::forward<TParserOptions>(o);

        if(m_parsingOptions.m_bodyIdRange.empty()) {
            m_parseSelectiveBodyIds = false;
        } else {
            m_parseSelectiveBodyIds = true;
        }

        m_allocateBodies = pSimBodies ?  true : false;

        m_startRangeIdIt = m_parsingOptions.m_bodyIdRange.begin();

    }

private:

    BodyModuleParserOptions m_parsingOptions;
    using BodyRangeType = typename BodyModuleParserOptions::BodyRangeType;

    bool m_parseSelectiveBodyIds;      ///< Use the m_bodyIdRange to only load the selective ids in the group which use enableSelectiveIds="true"
    bool m_parseOnlyVisualizationProperties;

    /// Parsing helpers
    BodyRangeType m_bodyIdRangeTmp;                     ///< Range of bodies, temporary for load of all bodies
    BodyRangeType m_bodyIdRange;                        ///< Range of body ids, original list which is handed to parseScene
    typename BodyRangeType::iterator m_startRangeIdIt;  ///< Current iterator which marks the start for the current group in m_bodyIdRange
    BodyRangeType * m_bodyIdRangePtr;                   ///< Switches between m_bodyIdRangeTmp/m_bodyIdRange, depending on parsing state

    unsigned int m_parsedInstancesGroup;            ///< Number of instances generated in the current group
    RigidBodyIdType m_startIdGroup;                  ///< Start id of the current group smaller or equal to *m_startRangeIdIt


    unsigned int m_nSimBodies, m_nBodies;
    typedef std::unordered_map<unsigned int,unsigned int> GroupToNBodyType;
    GroupToNBodyType m_groupIdToNBodies;
    unsigned int m_globalMaxGroupId; // Group Id used to build a unique id!

    /// Temprary structures for each sublist (groupid=?) of rigid bodies
    typename RigidBodyType::BodyState m_eBodiesState;     ///< Used to parse a RigidBody Node

    struct BodyData {
        BodyData( RigidBodyType * p, const RigidBodyIdType & id, const Vector3 & s = Vector3(1,1,1))
            : m_body(p),m_scale(s),m_initState(id) {}
        RigidBodyType* m_body ; // might be also zero (if we dont need the whole body for visualization only)
        Vector3 m_scale;
        RigidBodyState m_initState;
    };

    typename std::vector<BodyData> m_bodyListGroup; ///< Used to parse a RigidBody Node
    bool m_allocateBodies;


    /// Other Modules
    const GlobalGeomModType * pGeomMod;
    InitStatesModType * pInitStatesMod;
    std::unique_ptr<VisModType> pVisMod;

    RigidBodySimContainerType * pSimBodies;

};

template<typename TParser>
class ExternalForcesModule {
private:

    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
    DEFINE_LAYOUT_CONFIG_TYPES

public:
    ExternalForcesModule(ParserType * p) {};
    void parse() {
        std::cout << "parse ExternalForcesModule: @" << this << std::endl;
    }
};


};


/** SceneParser Options */
struct SceneParserOptions {
    bool m_parseSceneSettings = true; ///< Parse SceneSettings (default= true)
    bool m_parseSceneObjects  = true;  ///< Parse SceneObjects, (default= true)
};

template<typename TDynamicsSystem>
class SceneParser {
public:
    using DynamicsSystemType = TDynamicsSystem;

    using SettingsModuleType    = ParserModules::SettingsModule<SceneParser>;
    using GlobalGeomModuleType  = ParserModules::GlobalGeomModule<SceneParser>;
    using ContactParamModuleType= ParserModules::ContactParamModule<SceneParser>;

    using InitStatesModuleType  = ParserModules::InitStatesModule<SceneParser> ;
    using BodyVisModuleType     = ParserModules::BodyVisModule<SceneParser>;
    using BodyModuleType        = ParserModules::BodyModule< SceneParser, GlobalGeomModuleType , InitStatesModuleType, BodyVisModuleType > ;
    using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<SceneParser> ;
private:

    friend class ParserModules::SettingsModule<SceneParser>;
    friend class ParserModules::GlobalGeomModule<SceneParser>;
    friend class ParserModules::ContactParamModule<SceneParser>;

    friend class ParserModules::InitStatesModule<SceneParser>;
    friend class ParserModules::BodyVisModule<SceneParser>;
    friend class ParserModules::BodyModule< SceneParser, GlobalGeomModuleType , InitStatesModuleType, BodyVisModuleType > ;
    friend class ParserModules::ExternalForcesModule<SceneParser>;

    using XMLNodeType = pugi::xml_node;
    using XMLAttributeType = pugi::xml_attribute;
public:


    template<typename T>
    using UniformDistType = std::uniform_real_distribution<T>;



    SceneParser( std::shared_ptr<DynamicsSystemType> pDynSys):m_pDynSys(pDynSys) {
        // Get all Modules from DynamicsSystem
        std::tie(m_pSettingsModule, m_pBodyModule, m_pInitStatesModule, m_pGlobalGeomModule, m_pExternalForcesModule, m_pContactParamModule )
            = m_pDynSys->template createParserModules<SceneParser>(this);

        // Set log
        m_pSimulationLog = nullptr;
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
        ASSERTMSG(m_pSimulationLog, "There is no SimulationLog in the LogManager!");
        setStandartValues();

    }

    /**
    * range is only applied to the groups with the attribute enableSelectiveIds="true"
    */
    template<typename TParserOptions = SceneParserOptions, typename TBodyParserOptions = ParserModules::BodyModuleParserOptions >
    bool parseScene( const boost::filesystem::path & file,
                     TParserOptions&& opt = TParserOptions(),
                     TBodyParserOptions&& optBody = TBodyParserOptions()
                   ) {
        // Forward all settings
        m_parsingOptions = std::forward<TParserOptions>(opt);

        if(m_pBodyModule) {
            m_pBodyModule->setParsingOptions(std::forward<TBodyParserOptions>(optBody));
        }

        parseSceneIntern(file);
    }

    virtual void cleanUp() {
        // Delegate all cleanUp stuff to the modules!
    }

    boost::filesystem::path getParsedSceneFile() {
        return m_currentParseFilePath;
    }

private:

    bool parseSceneIntern(const boost::filesystem::path & file) {

        // Setting path
        bool reparse =false;
        if(file != m_currentParseFilePath) {
            reparse = true;
        }
        m_currentParseFilePath = file;
        m_currentParseFileDir = m_currentParseFilePath.parent_path();


        LOG( m_pSimulationLog, "---> Scene Input file: "  << file.string() <<std::endl; );

        if(!boost::filesystem::exists(m_currentParseFilePath)) {
            ERRORMSG("Scene Input file does not exist!");
        }


        try {

            if(reparse) {
                pugi::xml_parse_result result = m_xmlDoc.load_file(m_currentParseFilePath.c_str());
                if (result) {
                    LOG(m_pSimulationLog, "---> Loaded XML [" << m_currentParseFilePath.string() << "] without errors!" << std::endl;);
                } else {
                    THROWEXCEPTION( "Loaded XML [" << m_currentParseFilePath.string() << "] with errors!" << std::endl
                                    << "Error description: " << result.description() << std::endl
                                    << "Error offset: " << result.offset )
                }
            }

            LOG(m_pSimulationLog, "---> Try to parse the scene ..."<<std::endl;);

            GET_XMLCHILDNODE_CHECK( m_xmlRootNode, "DynamicsSystem" , m_xmlDoc );

            XMLNodeType node;
            GET_XMLCHILDNODE_CHECK( node , "SceneSettings",  m_xmlRootNode);
            parseSceneSettingsPre(node);

//                  node = m_xmlRootNode.child("SceneObjects");
//                  parseSceneObjects(node);
//
//                  node = m_xmlRootNode.child("SceneSettings");
//                  parseSceneSettingsAfter(node);
//
//                  parseOtherOptions(m_xmlRootNode);


        } catch(Exception& ex) {
            LOG(m_pSimulationLog,  "Scene XML error: "  << ex.what() <<std::endl);
            ERRORMSG( "Scene XML error: "  << ex.what() );
        }

    }


    virtual void parseSceneSettingsPre( XMLNodeType sceneSettings ) {

        if(!m_parsingOptions.m_parseSceneSettings) {
            LOG(m_pSimulationLog,"---> Skip SceneSettings"<<std::endl;);
            return;
        }

        LOG(m_pSimulationLog,"---> Parse SceneSettings..."<<std::endl;);

        if(m_pSettingsModule) {
            m_pSettingsModule->parse(sceneSettings);
        }

        if(m_pContactParamModule) {
            m_pContactParamModule->parse(sceneSettings);
        }


    }

    SceneParserOptions m_parsingOptions;

    void setStandartValues() {}

    std::shared_ptr<DynamicsSystemType> m_pDynSys;

    boost::filesystem::path m_currentParseFilePath;
    boost::filesystem::path m_currentParseFileDir;

    /** XML Declarations */
    pugi::xml_document m_xmlDoc;
    pugi::xml_node m_xmlRootNode;

    /** Log */
    Logging::Log * m_pSimulationLog;
    std::stringstream logstream;

    /** Modules */
    std::unique_ptr< SettingsModuleType>       m_pSettingsModule;
    std::unique_ptr< GlobalGeomModuleType>     m_pGlobalGeomModule;
    std::unique_ptr< ExternalForcesModuleType> m_pExternalForcesModule;
    std::unique_ptr< ContactParamModuleType>   m_pContactParamModule;

    std::unique_ptr< BodyModuleType>           m_pBodyModule;
    std::unique_ptr< InitStatesModuleType>     m_pInitStatesModule;



};




#endif

