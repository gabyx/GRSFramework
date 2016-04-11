// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

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
#define CHECK_XMLATTRIBUTE( _node_ , _nodename_ ) \
    if( ! _node_ ){ \
        THROWEXCEPTION("XML Attribute: " << _nodename_ << " does not exist!");  \
    }

#define GET_XMLCHILDNODE_CHECK( _childnode_ , _childname_ , _node_ ) \
    _childnode_ = _node_.child( _childname_ ); \
    CHECK_XMLNODE( _childnode_ , _childname_)

#define GET_XMLATTRIBUTE_CHECK( _att_ , _attname_ , _node_ ) \
    _att_ = _node_.attribute( _attname_ ); \
    CHECK_XMLATTRIBUTE( _att_ , _attname_ )

#define  DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE  \
    using ParserType = TParser; \
    using XMLNodeType = typename ParserType::XMLNodeType;\
    using XMLAttributeType = typename ParserType::XMLAttributeType;\
    using DynamicsSystemType = typename ParserType::DynamicsSystemType; \
    using RandomGenType = typename ParserType::RandomGenType; \
    template<typename T> using UniformDistType = typename ParserType::template UniformDistType<T>;\
    \
    using SettingsModuleType     = typename ParserType::SettingsModuleType;\
    using GeometryModuleType     = typename ParserType::GeometryModuleType;\
    using ContactParamModuleType = typename ParserType::ContactParamModuleType;\
    \
    using InitStatesModuleType       = typename ParserType::InitStatesModuleType ;\
    using VisModuleType              = typename ParserType::VisModuleType;\
    using BodyModuleType             = typename ParserType::BodyModuleType;\
    using ExternalForcesModuleType   = typename ParserType::ExternalForcesModuleType ;\

#define  DEFINE_PARSER_CONFIG_TYPES_OF_BASE( BaseParser ) \
    using XMLNodeType = typename BaseParser::XMLNodeType;\
    using XMLAttributeType = typename BaseParser::XMLAttributeType;\
    using RandomGenType = typename BaseParser::RandomGenType; \
    template<typename T> using UniformDistType = typename BaseParser::template UniformDistType<T>;\


#define DEFINE_MODULETYPES_AND_FRIENDS( _ModuleTraitsType_ )  \
public: \
    using SettingsModuleType       = typename _ModuleTraitsType_::SettingsModuleType;\
    using ExternalForcesModuleType = typename _ModuleTraitsType_::ExternalForcesModuleType;\
    using ContactParamModuleType   = typename _ModuleTraitsType_::ContactParamModuleType;\
    using InitStatesModuleType     = typename _ModuleTraitsType_::InitStatesModuleType;\
\
    using BodyModuleType           = typename _ModuleTraitsType_::BodyModuleType;\
    using GeometryModuleType       = typename _ModuleTraitsType_::GeometryModuleType;\
\
    using VisModuleType            = typename _ModuleTraitsType_::VisModuleType;\
protected: \
    friend typename _ModuleTraitsType_::SettingsModuleType;\
    friend typename _ModuleTraitsType_::ExternalForcesModuleType;\
    friend typename _ModuleTraitsType_::ContactParamModuleType;\
    friend typename _ModuleTraitsType_::InitStatesModuleType;\
\
    friend typename _ModuleTraitsType_::BodyModuleType;\
    friend typename _ModuleTraitsType_::GeometryModuleType;\
\
    friend typename _ModuleTraitsType_::VisModuleType;\






class DynamicsSystemGUI {
public:
    template<typename TParser>
    std::tuple< std::unique_ptr<typename TParser::SettingsModuleType >,
        std::unique_ptr<typename TParser::ExternalForcesModuleType >,
        std::unique_ptr<typename TParser::ContactParamModuleType>,
        std::unique_ptr<typename TParser::InitStatesModuleType >,
        std::unique_ptr<typename TParser::BodyModuleType >,
        std::unique_ptr<typename TParser::GeometryModuleType >,
        std::unique_ptr<typename TParser::VisModuleType>
        >
    createParserModules(TParser * p) {

        using SettingsModuleType       = typename TParser::SettingsModuleType ;
        using ContactParamModuleType   = typename TParser::ContactParamModuleType;
        using GeometryModuleType       = typename TParser::GeometryModuleType ;
        using InitStatesModuleType     = typename TParser::InitStatesModuleType ;
        using ExternalForcesModuleType = typename TParser::ExternalForcesModuleType ;
        using BodyModuleType           = typename TParser::BodyModuleType ;
        using VisModuleType            = typename TParser::VisModuleType ;


        auto sett = std::unique_ptr<SettingsModuleType >(new SettingsModuleType(p, &m_SettingsRecorder,
                    &m_SettingsTimestepper,
                    &m_SettingsInclusionSolver));

        auto geom = std::unique_ptr<GeometryModuleType >(new GeometryModuleType(p, &this->m_globalGeometries) );

        auto is  = std::unique_ptr<InitStatesModuleType >(new InitStatesModuleType(p,&this->m_bodiesInitStates, sett.get()));
        auto vis = std::unique_ptr<VisModuleType>( new VisModuleType(p,&m_SceneNodeSimBodies, &m_SceneNodeBodies, m_pBaseNode, m_pBodiesNode, m_pSceneMgr.get()) ); // no visualization needed
        auto bm  = std::unique_ptr<BodyModuleType>(new BodyModuleType(p,  geom.get(), is.get(), vis.get() , &this->m_SimBodies, &this->m_Bodies )) ;
        auto es  = std::unique_ptr<ExternalForcesModuleType >(new ExternalForcesModuleType(p, &this->m_externalForces));
        auto con = std::unique_ptr<ContactParamModuleType>(new ContactParamModuleType(p,&this->m_ContactParameterMap));

        return std::make_tuple(std::move(sett),std::move(es),std::move(con),std::move(is),std::move(bm),std::move(geom),std::move(vis));
    }

public:
//    using RigidBodyGraphicsType = RigidBodyGraphics;
//
//    using RigidBodyGraphicsContType = RigidBodyGraphicsContainer<RigidBodyGraphicsType>;
//
//    RigidBodyGraphicsContType	m_SceneNodeSimBodies;
//    RigidBodyGraphicsContType	m_SceneNodeBodies;

private:
//    std::shared_ptr<Ogre::SceneManager> m_pSceneMgr;
//
//    Ogre::SceneNode * m_pBodiesNode;
//    Ogre::SceneNode * m_pBaseNode;
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

    RecorderSettingsType        * m_recorderSettings;
    TimeStepperSettingsType     * m_timestepperSettings;
    InclusionSolverSettingsType * m_inclusionSettings;

    ParserType * m_parser;

public:

    TimeStepperSettingsType* getTimeStepperSettings() {
        return m_timestepperSettings;
    }

    SettingsModule(ParserType * p, RecorderSettingsType * r, TimeStepperSettingsType * t, InclusionSolverSettingsType * i)
        :m_parser(p),m_recorderSettings(r),m_timestepperSettings(t), m_inclusionSettings(i) {};

    void parse(XMLNodeType sceneSettings) {

        LOGSCLEVEL1(m_parser->m_pSimulationLog, "==== SettingsModule: parsing ======================================"<<std::endl;)

        XMLNodeType node;
        XMLAttributeType att;

        XMLNodeType timestepNode = sceneSettings.child("TimeStepperSettings");

        if( m_timestepperSettings ) {
            LOGSCLEVEL1(m_parser->m_pSimulationLog,"---> TimeStepperSettings ..." << std::endl;)
            CHECK_XMLNODE(timestepNode,"TimeStepperSettings");

            if(!Utilities::stringToType(m_timestepperSettings->m_deltaT, timestepNode.attribute("deltaT").value())) {
                THROWEXCEPTION("---> String conversion in SceneSettings: deltaT failed");
            }
            if(m_inclusionSettings) {
                m_inclusionSettings->m_deltaT = m_timestepperSettings->m_deltaT;
            }
            if(!Utilities::stringToType(m_timestepperSettings->m_endTime, timestepNode.attribute("endTime").value())) {
                THROWEXCEPTION("---> String conversion in SceneSettings: endTime failed");
            }

            auto node = timestepNode.child("SimulateFromReference");
            if(node) {
                bool enabled = false;
                if(!Utilities::stringToType(enabled, node.attribute("enabled").value())) {
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
                    m_parser->checkFileExists(m_timestepperSettings->m_simStateReferenceFile);
                } else {
                    m_timestepperSettings->m_eSimulateFromReference = TimeStepperSettings::NONE;
                }
            }
        } else {
            SKIPLOGSC(m_parser->m_pSimulationLog, "---> SettingsModule: skipped TimeStepperSettings ..."<<std::endl;)
        }


        if(m_inclusionSettings) {
            LOGSCLEVEL1(m_parser->m_pSimulationLog,"---> InclusionSolverSettings ..." << std::endl;)
            auto node = timestepNode.child("InclusionSolverSettings");
            CHECK_XMLNODE(node,"InclusionSolverSettings");

            if(!Utilities::stringToType(m_inclusionSettings->m_alphaJORProx, node.attribute("alphaJORProx").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: alphaJORProx failed");
            }
            if(!Utilities::stringToType(m_inclusionSettings->m_alphaSORProx, node.attribute("alphaSORProx").value())) {
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

            if(!Utilities::stringToType(m_inclusionSettings->m_AbsTol, node.attribute("absTol").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: absTol failed");
            }
            if(!Utilities::stringToType(m_inclusionSettings->m_RelTol, node.attribute("relTol").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: relTol failed");
            }

            att = node.attribute("computeResidual");
            if(att) {
                if(!Utilities::stringToType(m_inclusionSettings->m_bComputeResidual, att.value())) {
                    THROWEXCEPTION("---> String conversion in InclusionSolverSettings: computeResidual failed");
                }
            }

            att = node.attribute("isFiniteCheck");
            if(att) {
                if(!Utilities::stringToType(m_inclusionSettings->m_bIsFiniteCheck, att.value())) {
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

            if(!Utilities::stringToType(m_inclusionSettings->m_bUseGPU, node.attribute("useGPU").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: useGPU failed");
            }

            att = node.attribute("useGPUID");
            if(att) {
                if(!Utilities::stringToType(m_inclusionSettings->m_UseGPUDeviceId, att.value())) {
                    THROWEXCEPTION("---> String conversion in InclusionSolverSettings: useGPU failed");
                }
                if(m_inclusionSettings->m_UseGPUDeviceId <0) {
                    m_inclusionSettings->m_UseGPUDeviceId = 0;
                }
            }
        } else {
            SKIPLOGSC(m_parser->m_pSimulationLog, "---> SettingsModule: skipped InclusionSolverSettings ..."<<std::endl;)
        }


        if(m_recorderSettings) {
            LOGSCLEVEL1(m_parser->m_pSimulationLog,"---> RecorderSettings ..." << std::endl;)
            node = sceneSettings.child("RecorderSettings");
            CHECK_XMLNODE(node,"RecorderSettings");
            std::string method = node.attribute("recorderMode").value();
            if(method == "everyTimeStep") {
                m_recorderSettings->setMode(RecorderSettings::RECORD_EVERY_STEP);
            } else if (method == "everyXTimeStep") {
                m_recorderSettings->setMode(RecorderSettings::RECORD_EVERY_X_STEP);
                PREC fps;
                if(!Utilities::stringToType(fps, node.attribute("statesPerSecond").value())) {
                    THROWEXCEPTION("---> String conversion in RecorderSettings: statesPerSecond failed");
                }
                m_recorderSettings->setEveryXTimestep(fps,m_timestepperSettings->m_deltaT);
            } else if (method == "noOutput" || method=="none" || method=="nothing") {
                m_recorderSettings->setMode(RecorderSettings::RECORD_NOTHING);
            } else {
                THROWEXCEPTION("---> String conversion in RecorderSettings: recorderMode failed: not a valid setting");
            }

        } else {
            SKIPLOGSC(m_parser->m_pSimulationLog, "---> SettingsModule: skipped RecorderSettings ..."<<std::endl;)
        }

        LOGSCLEVEL1(m_parser->m_pSimulationLog, "==================================================================="<<std::endl;)
    }
};



template<typename TParser>
class GeometryModule {
private:
    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
    DEFINE_LAYOUT_CONFIG_TYPES

    using GlobalGeometryMapType = typename DynamicsSystemType::GlobalGeometryMapType;

    ParserType * m_parser;

    using BodyListType = typename BodyModuleType::BodyListType;

    BodyListType * m_bodyListGroup = nullptr;
    bool m_addToGlobalGeoms = false;
    RigidBodyIdType m_startIdGroup;

    GlobalGeometryMapType * m_globalGeometries;

public:
    GeometryModule(ParserType * p, GlobalGeometryMapType * g): m_parser(p),m_globalGeometries(g) {
        GRSF_ASSERTMSG(m_globalGeometries, "this should not be null")
    };

    void parseGlobalGeometries(XMLNodeType sceneSettings) {
        LOGSCLEVEL1(m_parser->m_pSimulationLog, "==== GeometryModule: parsing (GlobalGeometry) ====================="<<std::endl;)

        if(m_globalGeometries) {
            XMLNodeType globalGeom = sceneSettings.child("GlobalGeometries");
            if(globalGeom) {
                for (XMLNodeType n : globalGeom.children()) {
                    parseGeometry_imp(n);
                }
            }
        } else {
            SKIPLOGSC(m_parser->m_pSimulationLog, "---> GeometryModule: skipped GlobalGeometries ..."<<std::endl;)
        }

        LOGSCLEVEL1(m_parser->m_pSimulationLog, "==================================================================="<<std::endl;)
    }

    void parseGeometry( XMLNodeType geometryNode,  BodyListType * bodyList, RigidBodyIdType startId) {
        LOGSCLEVEL1(m_parser->m_pSimulationLog, "---> GeometryModule: parsing (BodyGeometry)"<<std::endl;)
        XMLNodeType geom= geometryNode.first_child(); // get the first geometry (sphere, box , mesh ...)
        CHECK_XMLNODE(geom, "(Sphere/Box/Mesh/Halfspace)");
        parseGeometry_imp(geom, bodyList, startId);
    }


private:

    void parseGeometry_imp( XMLNodeType geometryNode,  BodyListType * bodyList=nullptr, RigidBodyIdType startId=0) {
        m_startIdGroup = startId;
        m_bodyListGroup = bodyList;
        m_addToGlobalGeoms = m_bodyListGroup? false : true;

        if(std::strcmp(geometryNode.name() , "Sphere")==0) {
            parseSphereGeometry(geometryNode);

        } else if(std::strcmp(geometryNode.name() , "Halfspace")==0) {
            parseHalfspaceGeometry( geometryNode);

        } else if(std::strcmp(geometryNode.name() , "Mesh")==0) {
            parseMeshGeometry( geometryNode);

        } else if(std::strcmp(geometryNode.name() , "Box")==0) {
            parseBoxGeometry( geometryNode);

        } else if(std::strcmp(geometryNode.name() , "GlobalGeomId")==0) {
            parseGlobalGeomId(geometryNode);

        } else {
            THROWEXCEPTION("---> The geometry '" << geometryNode.name() << "' has no implementation in the parser");
        }
    }



    template<typename T>
    void addToGlobalGeomList(unsigned int id,  std::shared_ptr<T> ptr) {

        auto ret = m_globalGeometries->insert(typename GlobalGeometryMapType::value_type( id, ptr) );
        if(ret.second == false) {
            THROWEXCEPTION("---> addToGlobalGeomList: geometry with id: " <<  id<< " exists already!");
        }

        // Print some details:
        LOGSCLEVEL2(m_parser->m_pSimulationLog,"\t---> Added GlobalGeomId: " << id;);
        if(2<=SCENEPARSER_LOGLEVEL){
            PrintGeometryDetailsVisitor(m_parser->m_pSimulationLog, ret.first->second, ", ");
        }
    }

    /// Geometries ==============================================================================
    void parseSphereGeometry( XMLNodeType sphere) {
        std::string type = sphere.attribute("distribute").value();

        if(type == "uniform") {
            PREC radius;
            if(!Utilities::stringToType(radius,sphere.attribute("radius").value())) {
                THROWEXCEPTION("---> String conversion in addToGlobalGeomList: radius failed");
            }

            if(m_addToGlobalGeoms) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,sphere.attribute("id").value())) {
                    THROWEXCEPTION("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    THROWEXCEPTION("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                addToGlobalGeomList(id,std::shared_ptr<SphereGeometry >(new SphereGeometry(radius)));
            } else {
                Vector3 scale(radius,radius,radius);
                for(auto & b : *m_bodyListGroup) {
                    b.m_scale = scale;
                    LOGSCLEVEL3(m_parser->m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_initState.m_id) << ", GeometryType: Sphere" << std::endl);
                    LOGSCLEVEL3(m_parser->m_pSimulationLog, "\t\t---> radius: " << radius << std::endl; );
                    if(b.m_body) {
                        b.m_body->m_geometry = std::shared_ptr<SphereGeometry >(new SphereGeometry(radius));
                    }
                }
            }
        } else if(type == "random") {
            double minRadius;
            if(!Utilities::stringToType(minRadius,sphere.attribute("minRadius").value())) {
                THROWEXCEPTION("---> String conversion in parseSphereGeometry: minRadius failed");
            }
            if( minRadius <= 0) {
                THROWEXCEPTION("---> In parseSphereGeometry: minRadius to small!");
            }

            double maxRadius;
            if(!Utilities::stringToType(maxRadius,sphere.attribute("maxRadius").value())) {
                THROWEXCEPTION("---> String conversion in parseSphereGeometry: minRadius failed");
            }
            if( maxRadius <= minRadius) {
                THROWEXCEPTION("---> In parseSphereGeometry: maxRadius smaller or equal to minRadius!");
            }

            unsigned int seed;
            if(!Utilities::stringToType<unsigned int>(seed,sphere.attribute("seed").value())) {
                THROWEXCEPTION("---> String conversion in parseSphereGeometry: seed failed");
            }


            RandomGenType gen(seed);
            UniformDistType<PREC> uni(minRadius,maxRadius);

            if(m_addToGlobalGeoms) {

                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,sphere.attribute("id").value())) {
                    THROWEXCEPTION("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    THROWEXCEPTION("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }

                unsigned int instances = 1;
                if(sphere.attribute("instances")) {

                    if(!Utilities::stringToType<unsigned int>(instances,sphere.attribute("instances").value())) {
                        THROWEXCEPTION("---> String conversion in addToGlobalGeomList: instances failed");
                    }
                }

                for(int i = id; i < id + instances; i++) {
                    PREC radius = uni(gen);
                    addToGlobalGeomList(i, std::shared_ptr<SphereGeometry >(new SphereGeometry(radius)));
                }
            } else {


                RigidBodyIdType diffId = m_startIdGroup; // id to generate to correct amount of random values!
                double radius = uni(gen); // generate first value
                auto endIt = m_bodyListGroup->end();
                for(auto bodyIt = m_bodyListGroup->begin(); bodyIt != endIt; ++bodyIt) {

                    // Generate the intermediate random values if there are any
                    radius = Utilities::genRandomValues(radius,gen,uni,bodyIt->m_initState.m_id-diffId); // (id:16 - id:13 = 3 values, 13 is already generated)
                    diffId = bodyIt->m_initState.m_id; // update current diffId;

                    bodyIt->m_scale = Vector3(radius,radius,radius);
                    LOGSCLEVEL3(m_parser->m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(bodyIt->m_initState.m_id)<< ", GeometryType: Sphere" << std::endl);
                    LOGSCLEVEL3(m_parser->m_pSimulationLog, "\t\t---> radius: " << radius << std::endl; );

                    if(bodyIt->m_body) {
                        bodyIt->m_body->m_geometry = std::shared_ptr<SphereGeometry >(new SphereGeometry(radius));
                    }

                }
            }
        } else {
            THROWEXCEPTION("---> The attribute 'distribute' '" + type + std::string("' of 'Sphere' has no implementation in the parser"));
        }


    }
    void parseHalfspaceGeometry( XMLNodeType halfspace) {
        std::string type = halfspace.attribute("distribute").value();
        if(type == "uniform") {

            Vector3 n;
            if(!Utilities::stringToVector3(n, halfspace.attribute("normal").value())) {
                THROWEXCEPTION("---> String conversion in HalfsphereGeometry: normal failed");
            }

            Vector3 p;
            if(!Utilities::stringToVector3(p, halfspace.attribute("position").value())) {
                THROWEXCEPTION("---> String conversion in HalfsphereGeometry: position failed");
            }

            std::shared_ptr<HalfspaceGeometry > pHalfspaceGeom = std::shared_ptr<HalfspaceGeometry >(new HalfspaceGeometry(n,p));

            if(m_addToGlobalGeoms) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,halfspace.attribute("id").value())) {
                    THROWEXCEPTION("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    THROWEXCEPTION("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                addToGlobalGeomList(id, pHalfspaceGeom);
            } else {

                if(m_bodyListGroup->size() != 0 && m_bodyListGroup->begin()->m_body != nullptr ) {
                    for(auto & b  : *m_bodyListGroup) {
                        b.m_body->m_geometry = pHalfspaceGeom;
                    }
                }
            }

        } else {
            THROWEXCEPTION("---> The attribute 'type' '" + type + std::string("' of 'Halfspace' has no implementation in the parser"));
        }
    }
    void parseBoxGeometry( XMLNodeType box) {
        std::string type = box.attribute("distribute").value();
        if(type == "uniform") {

            Vector3 extent;
            if(!Utilities::stringToVector3(extent, box.attribute("extent").value())) {
                THROWEXCEPTION("---> String conversion in BoxGeometry: extent failed");
            }

            Vector3 center;
            if(!Utilities::stringToVector3(center, box.attribute("center").value())) {
                THROWEXCEPTION("---> String conversion in BoxGeometry: position failed");
            }

            std::shared_ptr<BoxGeometry > pBoxGeom = std::shared_ptr<BoxGeometry >(new BoxGeometry(center,extent));

            Vector3 scale(extent(0),extent(1),extent(2));

            if(m_addToGlobalGeoms) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,box.attribute("id").value())) {
                    THROWEXCEPTION("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    THROWEXCEPTION("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                addToGlobalGeomList(id, pBoxGeom);
            } else {
                for(auto & b  : *m_bodyListGroup) {
                    b.m_scale = scale;
                    if(b.m_body) {
                        b.m_body->m_geometry = pBoxGeom;
                    }
                }
            }

        } else {
            THROWEXCEPTION("---> The attribute 'type' '" + type + std::string("' of 'Box' has no implementation in the parser"));
        }
    }
    void parseMeshGeometry( XMLNodeType mesh) {

        std::shared_ptr<MeshGeometry > pMeshGeom;

        std::string meshName = mesh.attribute("name").value();

        bool bInstantiate;
        if(!Utilities::stringToType(bInstantiate,mesh.attribute("useInstance").value())) {
            THROWEXCEPTION("---> String conversion in parseMeshGeometry: useInstance failed");
        }

        std::string type = mesh.attribute("distribute").value();
        if(type == "uniform") {

            // import here an object model;
            // First make an meshinformatino structure

            boost::filesystem::path fileName =  mesh.attribute("file").value();
            m_parser->checkFileExists(fileName);

            Vector3 scale_factor;
            if(!Utilities::stringToVector3(scale_factor, mesh.attribute("scale").value())) {
                THROWEXCEPTION("---> String conversion in parseMeshGeometry failed: scale");
            }
            if(scale_factor.norm()==0) {
                THROWEXCEPTION("---> Wrong scale factor (=0) specified in parseMeshGeometry!");
            }

            Vector3 trans;
            if(!Utilities::stringToVector3(trans, mesh.attribute("translation").value())) {
                THROWEXCEPTION("---> String conversion in parseMeshGeometry: translation failed: ");
            }

            Vector3 axis;
            if(!Utilities::stringToVector3(axis, mesh.attribute("rotationAxis").value())) {
                THROWEXCEPTION("---> String conversion in parseMeshGeometry: rotationAxis failed");
            }

            PREC angle;

            if(mesh.attribute("angleDegree")) {
                if(!Utilities::stringToType(angle, mesh.attribute("angleDegree").value())) {
                    THROWEXCEPTION("---> String conversion in parseMeshGeometry: angleDegree failed");
                }
                angle = angle / 180 * M_PI;
            } else if(mesh.attribute("angleRadian")) {
                if(!Utilities::stringToType(angle, mesh.attribute("angleRadian").value())) {
                    THROWEXCEPTION("---> String conversion in parseMeshGeometry: angleRadian  failed");
                }
            } else {
                THROWEXCEPTION("---> No angle found in parseMeshGeometry");
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
                THROWEXCEPTION("---> File import failed in parseMeshGeometry: for file" + fileName.string() );
            }

            MeshData * meshData = new MeshData();

            if(!meshData->setup(importer,scene, scale_factor,quat,trans)) {
                THROWEXCEPTION("---> Imported Mesh (with Assimp) could not be setup internally");
            }

            // Build Geometry
            pMeshGeom = std::shared_ptr<MeshGeometry >(new MeshGeometry(meshData));

            if(mesh.attribute("writeToLog")) {
                bool writeToLog;
                if(!Utilities::stringToType(writeToLog, mesh.attribute("writeToLog").value())) {
                    THROWEXCEPTION("---> String conversion in parseMeshGeometry: angleDegree failed");
                }
                if(writeToLog) {
                    meshData->writeToLog(fileName.string(), m_parser->m_pSimulationLog);
                }
            }



            // Assign Geometry
            if(m_addToGlobalGeoms) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,mesh.attribute("id").value())) {
                    THROWEXCEPTION("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    THROWEXCEPTION("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                addToGlobalGeomList(id, pMeshGeom);
            } else {
                for(auto & b  : *m_bodyListGroup) {
                    b.m_scale = scale_factor;
                    if(b.m_body) {
                        b.m_body->m_geometry = pMeshGeom;
                    }
                }
            }

        } else {
            THROWEXCEPTION("---> The attribute 'type' '" + type + std::string("' of 'Mesh' has no implementation in the parser"));
        }
    }
    void parseGlobalGeomId( XMLNodeType globalGeomId ) {

        std::string distribute = globalGeomId.attribute("distribute").value();
        if(distribute == "uniform") {

            unsigned int id;
            if(!Utilities::stringToType<unsigned int>(id,globalGeomId.attribute("id").value())) {
                THROWEXCEPTION("---> String conversion in parseGlobalGeomId: id failed");
            }

            auto it = m_globalGeometries->find(id);
            // it->second is the GeometryType in RigidBody
            if(it == m_globalGeometries->end()) {
                THROWEXCEPTION("---> Geometry search in parseGlobalGeomId: failed!");
            }

            for(auto & b : *m_bodyListGroup) {
                GetScaleOfGeomVisitor vis(b.m_scale);
                boost::apply_visitor(vis, it->second);
                if(b.m_body){
                    b.m_body->m_geometry = it->second;
                    b.m_body->m_globalGeomId = id;
                    LOGSCLEVEL3(m_parser->m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_body) << ", GlobalGeomId: " << id <<  std::endl);
                }
            }

        } else if(distribute == "linear") {

            unsigned int startId;
            if(!Utilities::stringToType<unsigned int>(startId,globalGeomId.attribute("startId").value())) {
                THROWEXCEPTION("---> String conversion in parseGlobalGeomId: startId failed");
            }

            if(startId == 0) {
                THROWEXCEPTION("---> parseGlobalGeomId: a global geometry startId: 0 is not allowed!");
                // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
            }

            for(auto & b : *m_bodyListGroup) {
                unsigned int id = startId + (b.m_initState.m_id - m_startIdGroup); //make linear offset from start of this group
                auto it = m_globalGeometries->find(id);
                // it->second is the GeometryType in RigidBody
                if(it == m_globalGeometries->end()) {
                    THROWEXCEPTION("---> parseGlobalGeomId: Geometry search failed!");
                }

                GetScaleOfGeomVisitor vis(b.m_scale);
                boost::apply_visitor(vis, it->second);
                if(b.m_body){
                    b.m_body->m_geometry = it->second;
                    b.m_body->m_globalGeomId = id;
                    LOGSCLEVEL3(m_parser->m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_body) << ", GlobalGeomId: " << id <<  std::endl);
                }

            }


        } else if(distribute == "random") {

            unsigned int startId;
            if(!Utilities::stringToType<unsigned int>(startId,globalGeomId.attribute("startId").value())) {
                THROWEXCEPTION("---> String conversion in parseGlobalGeomId: startId failed");
            }

            if(startId == 0) {
                THROWEXCEPTION("---> parseGlobalGeomId: a global geometry startId: 0 is not allowed!");
                // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
            }

            unsigned int endId;
            if(!Utilities::stringToType<unsigned int>(endId,globalGeomId.attribute("endId").value())) {
                THROWEXCEPTION("---> String conversion in parseGlobalGeomId: endId failed");
            }
            if(startId > endId) {
                THROWEXCEPTION("---> addToGlobalGeomList:  startId > endId  is not allowed!");
                // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
            }
            unsigned int seed;
            if(!Utilities::stringToType<unsigned int>(seed,globalGeomId.attribute("seed").value())) {
                THROWEXCEPTION("---> String conversion in parseGlobalGeomId: seed failed");
            }

            RandomGenType gen(seed);
            std::uniform_int_distribution<unsigned int> uni(startId,endId);

            RigidBodyIdType diffId = m_startIdGroup; // id to generate the correct amount of random values!

            unsigned int id = uni(gen); // generate first value
            for(auto & b: *m_bodyListGroup) {

                id = Utilities::genRandomValues(id,gen,uni,b.m_initState.m_id - diffId);
                auto it = m_globalGeometries->find(id);
                // it->second is the GeometryType in RigidBody
                if(it == m_globalGeometries->end()) {
                    THROWEXCEPTION("---> Geometry search in parseGlobalGeomId: failed!");
                }

                GetScaleOfGeomVisitor vis(b.m_scale);
                boost::apply_visitor(vis, it->second);
                if(b.m_body){
                    b.m_body->m_geometry = it->second;
                    b.m_body->m_globalGeomId = id;
                    LOGSCLEVEL3(m_parser->m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_body) << ", GlobalGeomId: " << id <<  std::endl);
                }
            }


        } else {
            THROWEXCEPTION("---> The attribute 'distribute' '" + distribute + std::string("' of 'GlobalGeomId' has no implementation in the parser"));
        }



    }
    ///  ================================================================================ Geometries


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

    void parse(XMLNodeType sceneSettings) {

        LOGSCLEVEL1(m_parser->m_pSimulationLog, "---> ContactParamModule: parsing =================================="<<std::endl;)

        if( !m_contactParams ) {
            SKIPLOGSC(m_parser->m_pSimulationLog, "---> ContactParamModule: skipping ..."<<std::endl;)
            return;
        }

        XMLNodeType paramMap = sceneSettings.child("ContactParameterMap");

        XMLNodeType node;
        if(paramMap) {
            node = paramMap.child("ContactParameterStandard");
            if(node) {
                parseContactParameter(node, true);
            }
            for (XMLNodeType it : paramMap.children("ContactParameter")) {
                parseContactParameter(it);
            }
        }

        LOGSCLEVEL1(m_parser->m_pSimulationLog, "==================================================================="<<std::endl;)
    }

private:
    void parseContactParameter(XMLNodeType contactParam, bool stdMaterial=false) {


        typename RigidBodyType::BodyMaterialType material1,material2;
        if(!stdMaterial) {
            if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(material1, contactParam.attribute("materialId1").value())) {
                THROWEXCEPTION("---> String conversion in ContactParameter: materialId1 failed");
            }
            if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(material2, contactParam.attribute("materialId2").value())) {
                THROWEXCEPTION("---> String conversion in ContactParameter: materialId2 failed");
            }
        }


        std::string type = contactParam.attribute("type").value();
        if(type == "UCF" || type == "UCFD" || type == "UCFDD") {

            PREC mu,epsilonN,epsilonT;
            ContactParameter contactParameter;

            if(!Utilities::stringToType(mu, contactParam.attribute("mu").value())) {
                THROWEXCEPTION("---> String conversion in ContactParameter: mu failed");
            }
            if(!Utilities::stringToType(epsilonN, contactParam.attribute("epsilonN").value())) {
                THROWEXCEPTION("---> String conversion in ContactParameter: epsilonN failed");
            }
            if(!Utilities::stringToType(epsilonT, contactParam.attribute("epsilonT").value())) {
                THROWEXCEPTION("---> String conversion in ContactParameter: epsilonT failed");
            }

            if(type == "UCFD") {
                PREC invDampingN, invDampingT;
                if(!Utilities::stringToType(invDampingN, contactParam.attribute("invDampingN").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: invDampingN failed");
                }
                if(!Utilities::stringToType(invDampingT, contactParam.attribute("invDampingT").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: invDampingT failed");
                }

                contactParameter = ContactParameter::createParams_UCFD_ContactModel(epsilonN,epsilonT,mu,invDampingN,invDampingT);


            } else if(type == "UCFDD") {

                PREC invDampingN, gammaMax, epsilon, invDampingTFix;
                if(!Utilities::stringToType(invDampingN, contactParam.attribute("invDampingN").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: invDampingN failed");
                }

                if(!Utilities::stringToType(invDampingTFix, contactParam.attribute("invDampingTFix").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: invDampingTFix failed");
                }
                if(!Utilities::stringToType(gammaMax, contactParam.attribute("gammaMax").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: gamma_max failed");
                }
                if(!Utilities::stringToType(epsilon, contactParam.attribute("epsilon").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: epsilon failed");
                }

                contactParameter = ContactParameter::createParams_UCFDD_ContactModel(epsilonN,epsilonT,mu,
                                   invDampingN,invDampingTFix,
                                   gammaMax,epsilon);
            } else {
                contactParameter = ContactParameter::createParams_UCF_ContactModel(epsilonN,epsilonT,mu);
            }

            if(stdMaterial) {
                LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> Add ContactParameter standart"<<std::endl;);
                m_contactParams->setStandardValues(contactParameter);
            } else {
                LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> Add ContactParameter standart of id="<<material1<<" to id="<<material2<<std::endl;);
                if(!m_contactParams->addContactParameter(material1,material2,contactParameter)) {
                    THROWEXCEPTION("---> Add ContactParameter failed");
                }
            }


        } else {
            THROWEXCEPTION("---> String conversion in ContactParameter: type failed");
        }
    }

};

template<typename TParser>
class ExternalForcesModule {
private:

    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
    DEFINE_LAYOUT_CONFIG_TYPES

    using ExternalForceListType = typename DynamicsSystemType::ExternalForceListType;
    ExternalForceListType * m_forceList;

    using RigidBodyType = typename DynamicsSystemType::RigidBodyType;

    ParserType * m_parser;

public:
    ExternalForcesModule(ParserType * p, ExternalForceListType * f): m_parser(p), m_forceList(f) {};

    void parse(XMLNodeType sceneSettings) {
        LOGSCLEVEL1(m_parser->m_pSimulationLog, "==== ExternalForcesModule: parsing ================================"<<std::endl;)

        XMLNodeType externalForces = sceneSettings.child("ExternalForces");
        if(externalForces) {
            for (XMLNodeType n : externalForces.children()) {
                if( std::string(n.name()) == "ForceField") {
                    parseForceField( n );
                }
            }
        } else {
            SKIPLOGSC(m_parser->m_pSimulationLog, "---> ExternalForcesModule: skipping ..."<<std::endl;)
        }
        LOGSCLEVEL1(m_parser->m_pSimulationLog, "==================================================================="<<std::endl;)
    }

private:

    void parseForceField( XMLNodeType forceField) {

        bool enabled = false;
        if(!Utilities::stringToType(enabled, forceField.attribute("enabled").value())) {
            THROWEXCEPTION("---> String conversion in parseForceField: enable failed");
        }
        if(enabled) {

            std::string apply  = forceField.attribute("applyTo").value();
            //std::vector<RigidBodyIdType > applyList;
            if( !(apply=="all" || apply=="All" || apply=="ALL" )) {
                //parse all applyTo bodies
            } else if (apply=="all" || apply=="All" || apply=="ALL" ) {
                // do nothing
            } else {
                THROWEXCEPTION("---> String conversion in parseForceField: applyTo failed");
            }

            std::string type = forceField.attribute("type").value();
            if(type == "spatialspherical-timerandom") {

                unsigned int seed;
                if(!Utilities::stringToType<unsigned int>(seed, forceField.attribute("seed").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: seed failed");
                }
                PREC boostTime;
                if(!Utilities::stringToType(boostTime, forceField.attribute("boostTime").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: boostTime failed");
                }
                PREC pauseTime;
                if(!Utilities::stringToType(pauseTime, forceField.attribute("pauseTime").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: pauseTime failed");
                }

                PREC startTime;
                if(!Utilities::stringToType(startTime, forceField.attribute("startTime").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: startTime failed");
                }

                PREC endTime;
                if(!Utilities::stringToType(endTime, forceField.attribute("endTime").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: endTime failed");
                }
                PREC amplitude;
                if(!Utilities::stringToType(amplitude, forceField.attribute("amplitude").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: amplitude failed");
                }

                Vector3 boxMin;
                if(!Utilities::stringToVector3(boxMin, forceField.attribute("minPoint").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: boxMin failed");
                }
                Vector3 boxMax;
                if(!Utilities::stringToVector3(boxMax, forceField.attribute("maxPoint").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: boxMax failed");
                }

                bool randomOn;
                if(!Utilities::stringToType(randomOn, forceField.attribute("randomOn").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: randomOn failed");
                }


                m_forceList->addExternalForceCalculation(
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
                LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> added SpatialSphericalTimeRandomForceField ..."<<std::endl;);

            } else if(type == "gravity") {
                PREC abs;
                if(!Utilities::stringToType(abs, forceField.attribute("value").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: value failed");
                }
                Vector3 dir;
                if(!Utilities::stringToVector3(dir, forceField.attribute("direction").value())) {
                    THROWEXCEPTION("---> String conversion in SceneSettings: gravity failed");
                }
                dir.normalize();
                dir *= abs;

                m_forceList->addExternalForceCalculation(new GravityForceField(dir));

                LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> added GravityForceField ..."<<std::endl;);
            } else {
                THROWEXCEPTION("---> String conversion in parseForceField: type failed");
            }
        }
    }
};


template<typename TParser>
class VisModuleDummy {
private:
    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
public:
    VisModuleDummy(ParserType * p, BodyModuleType * b) {};

    template<typename... Args>
    void parse(Args&&... args) {
         GRSF_ERRORMSG("This is the standard BodyVisModule which does nothing! This function should not be called!");
    }
    template<typename... Args>
    void parseSceneSettingsPost(Args&&... args) {
         GRSF_ERRORMSG("This is the standard BodyVisModule which does nothing! This function should not be called!");
    }
};


template<typename TParser>
class InitStatesModule {
private:
    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
    DEFINE_LAYOUT_CONFIG_TYPES

    using RigidBodyType = typename DynamicsSystemType::RigidBodyType;
    using RigidBodyStatesContainerType = typename DynamicsSystemType::RigidBodyStatesContainerType;
    RigidBodyStatesContainerType * m_initStates;

    SettingsModuleType * m_settings;
    ParserType * m_parser;

    using BodyListType = typename BodyModuleType::BodyListType;
    BodyListType * m_bodyListGroup;
    RigidBodyIdType m_startIdGroup;


public:
    InitStatesModule(ParserType * p, RigidBodyStatesContainerType * c, SettingsModuleType * s): m_parser(p),m_initStates(c), m_settings(s) {
        GRSF_ASSERTMSG(m_initStates, "should not be null");
    };


    void parseGlobalInitialCondition( XMLNodeType sceneSettings) {
        LOGSCLEVEL1(m_parser->m_pSimulationLog, "---> InitStatesModule: parsing (GlobalInitCondition) =============="<<std::endl;)
        XMLNodeType initCond = sceneSettings.child("GlobalInitialCondition");
        if(initCond) {
            bool enabled = false;
            if(!Utilities::stringToType(enabled, initCond.attribute("enabled").value())) {
                THROWEXCEPTION("---> String conversion in GlobalInitialCondition: enable failed");
            }
            if(enabled) {
                double time = -1;
                short which = 2;
                std::string str = initCond.attribute("whichState").value();
                if( str == "end" || str == "END" || str== "End") {
                    which = 2;
                } else if(str == "beg" || str == "begin" || str== "BEG" || str == "Beg") {
                    which = 0;
                } else if(str == "time" || str =="TIME") {
                    which = 1;
                    if(!Utilities::stringToType(time, initCond.attribute("time").value())) {
                        THROWEXCEPTION("---> String conversion in GlobalInitialCondition: time failed");
                    }
                } else {
                    THROWEXCEPTION("---> String conversion in GlobalInitialCondition: whichState failed");
                }

                boost::filesystem::path relpath = initCond.attribute("path").value();


                setupInitialConditionBodiesFromFile_imp(relpath, time, which);

                bool useTime = false;
                if(!Utilities::stringToType(useTime, initCond.attribute("useTimeToContinue").value())) {
                    THROWEXCEPTION("---> String conversion in GlobalInitialCondition: useTimeToContinue failed");
                }

                // Set the time in the dynamics system timestepper settings
                if(useTime) {
                    m_settings->getTimeStepperSettings()->m_startTime = time;
                }


            }

        }
        LOGSCLEVEL1(m_parser->m_pSimulationLog, "==================================================================="<<std::endl;)
    }

    void parseInitialCondition(XMLNodeType initCondNode, BodyListType * bodyList,
                               RigidBodyIdType startId, bool parseVelocity = true , bool addToInitList = true) {

        LOGSCLEVEL1(m_parser->m_pSimulationLog, "---> InitStatesModule: parsing (BodyInitState)"<<std::endl;)
        GRSF_ASSERTMSG(bodyList, "Should not be null!")

        m_bodyListGroup = bodyList;
        m_startIdGroup = startId;

        std::string distribute;
        XMLNodeType node;
        std::string s  = initCondNode.attribute("type").value();
        if( s == "file") {
            THROWEXCEPTION(" Not yet implemented");
        } else if (s == "posvel") {

            GET_XMLCHILDNODE_CHECK(node,"InitialPosition",initCondNode);

            s = node.attribute("distribute").value();
            if(s == "linear") {
                parseInitialPositionLinear(node);
            } else if(s == "grid") {
                parseInitialPositionGrid(node);
            } else if(s == "posaxisangle") {
                parseInitialPositionPosAxisAngle(node);
            } else if(s == "transforms") {
                parseInitialPositionTransforms(node);
            } else if(s == "generalized") {
                THROWEXCEPTION("Not yet implemented!");
            } else if(s == "none") {
                // does nothing leaves the zero state pushed!
            } else {
                THROWEXCEPTION("---> The attribute 'distribute' '" << s << "' of 'InitialPosition' has no implementation in the parser");
            }

            //Initial Velocity
            if(parseVelocity) {
                XMLNodeType  initVel = initCondNode.child("InitialVelocity");
                if(initVel) {
                    s = initVel.attribute("distribute").value();

                    if(s == "transrot") {
                        parseInitialVelocityTransRot(initVel);
                    } else if(s == "generalized") {
                        THROWEXCEPTION("Not yet implemented!");
                    } else if(s == "none") {
                        // does nothing leaves the zero state pushed!
                    } else {
                        THROWEXCEPTION("---> The attribute 'distribute' '" << s << "' of 'InitialVelocity' has no implementation in the parser");
                    }
                }
            }

        } else {
            THROWEXCEPTION("---> The attribute 'type' '" << s <<"' of 'InitialCondition' has no implementation in the parser");
        }

        bool added = true;
        for(auto & b: *m_bodyListGroup) {
            LOGSCLEVEL3(m_parser->m_pSimulationLog, "\t---> InitState:" << b.m_initState.m_q.transpose()
                            << " , " << b.m_initState.m_u.transpose()  << std::endl;)
            if(b.m_body) {
                LOGSCLEVEL3(m_parser->m_pSimulationLog, "\t---> apply to body" << std::endl;)
                b.m_body->template applyBodyState<true>(b.m_initState);
            }
            if(addToInitList){
                added &= m_initStates->emplace(b.m_initState.m_id, b.m_initState).second;
            }
        }
        if(!added && addToInitList) {THROWEXCEPTION("Could not add init state to m_initStates!, some bodies exist already in map!");};

        //LOGSCLEVEL1(m_parser->m_pSimulationLog, "==================================================================="<<std::endl;)
    }


private:
    void setupInitialConditionBodiesFromFile_imp(boost::filesystem::path relpath, double &time , short which ) {

        InitialConditionBodies::setupInitialConditionBodiesFromFile(relpath,*m_initStates,time,true,true,which);
        LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> Found time: "<< time << " in " << relpath << std::endl;);

    }

    void parseInitialPositionLinear(XMLNodeType initCond) {

        Vector3 pos;
        if(!Utilities::stringToVector3(pos, initCond.attribute("position").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionLinear: position Linear failed");
        }
        Vector3 dir;
        if(!Utilities::stringToVector3(dir, initCond.attribute("direction").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionLinear: direction Linear failed");
        }
        PREC dist;
        if(!Utilities::stringToType(dist, initCond.attribute("distance").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionLinear: distance  Linear failed");
        }
        bool jitter;
        if(!Utilities::stringToType(jitter, initCond.attribute("jitter").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionLinear: jitter Linear failed");
        }

        PREC delta;
        if(!Utilities::stringToType(delta, initCond.attribute("delta").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionLinear: delta Linear failed");
        }

        unsigned int seed = 5;
        if(initCond.attribute("seed")) {
            if(!Utilities::stringToType(seed, initCond.attribute("seed").value())) {
                THROWEXCEPTION("---> String conversion in InitialPositionGrid: jitter seed failed");
            }
        }

        InitialConditionBodies::setupPositionBodiesLinear(*m_bodyListGroup,m_startIdGroup,pos,dir,dist,jitter,delta,seed);

    }
    void parseInitialPositionGrid(XMLNodeType initCond) {

        Vector3 trans;
        if(!Utilities::stringToVector3(trans, initCond.attribute("translation").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionGrid: translation failed");
        }
        int gridX;
        if(!Utilities::stringToType(gridX, initCond.attribute("gridSizeX").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionGrid: gridSizeX failed");
        }
        int gridY;
        if(!Utilities::stringToType(gridY, initCond.attribute("gridSizeY").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionGrid: gridSizeY failed");
        }
        PREC dist;
        if(!Utilities::stringToType(dist, initCond.attribute("distance").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionGrid: distance failed");
        }
        bool jitter;
        if(!Utilities::stringToType(jitter, initCond.attribute("jitter").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionGrid: jitter failed");
        }
        int seed = 5;
        if(initCond.attribute("seed")) {
            if(!Utilities::stringToType(seed, initCond.attribute("seed").value())) {
                THROWEXCEPTION("---> String conversion in InitialPositionGrid: jitter seed failed");
            }
        }
        double delta;
        if(!Utilities::stringToType(delta, initCond.attribute("delta").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionGrid: delta failed");
        }

        InitialConditionBodies::setupPositionBodiesGrid(*m_bodyListGroup,m_startIdGroup,gridX,gridY,dist,trans,jitter,delta, seed);
    }
    void parseInitialPositionFile(DynamicsState & state, XMLNodeType initCond) {
//        m_SimBodyInitStates.push_back(DynamicsState((unsigned int)m_bodyListGroup->size()));
//
//        boost::filesystem::path name =  initCond->GetAttribute<std::string>("relpath");
//
//        boost::filesystem::path filePath = m_currentParseFileDir / name;
//        InitialConditionBodies::setupPositionBodiesFromFile(state,filePath);
        THROWEXCEPTION("Not implemented")
    }
    void parseInitialPositionPosAxisAngle(XMLNodeType initCond) {

        unsigned int consumedValues = 0;
        Vector3 pos;
        Vector3 axis;
        PREC angle;

        auto bodyIt = m_bodyListGroup->begin();
        GRSF_ASSERTMSG(bodyIt != m_bodyListGroup->end(),"no bodies in list");

        // Iterate over all values in the list

        for ( XMLNodeType & node : initCond.children("Value")) {

            if(consumedValues >= m_bodyListGroup->size()) {
                LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> InitialPositionPosAxisAngle: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            } else if(consumedValues != bodyIt->m_initState.m_id - m_startIdGroup) {
                // this value does not correspond to the linear offset from the start
                consumedValues++;
                continue;
            }


            if(!Utilities::stringToVector3(pos, node.attribute("position").value())) {
                THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: position failed");
            }

            if(!Utilities::stringToVector3(axis, node.attribute("axis").value())) {
                THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: axis failed");
            }

            if( axis.norm() == 0) {
                THROWEXCEPTION("---> Specified wrong axis in InitialPositionPosAxisAngle");
            }

            auto att = node.attribute("angleDegree");
            if(att) {
                if(!Utilities::stringToType(angle, att.value())) {
                    THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: angleDegree failed");
                }
                angle = angle / 180 * M_PI;
            } else {
                att = node.attribute("angleRadian");
                if(att){
                    if(!Utilities::stringToType(angle, att.value())) {
                        THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: angleRadian failed");
                    }
                }
                else{
                    THROWEXCEPTION("---> No angle found in InitialPositionPosAxisAngle");
                }
            }

            InitialConditionBodies::setupPositionBodyPosAxisAngle( bodyIt->m_initState, pos, axis, angle);
            ++consumedValues;
            ++bodyIt;
        }

        if(consumedValues < m_bodyListGroup->size()) {
            LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> InitialPositionPosAxisAngle: You specified to little values, -> applying last to all remainig bodies ..."<<std::endl;);
            auto itEnd = m_bodyListGroup->end();
            for(; bodyIt !=  itEnd; ++bodyIt) {
                InitialConditionBodies::setupPositionBodyPosAxisAngle(bodyIt->m_initState, pos, axis, angle);
            }
        }
    }
    void parseInitialPositionTransforms(XMLNodeType initCond) {

        unsigned int consumedValues = 0;

        auto bodyIt = m_bodyListGroup->begin();
        GRSF_ASSERTMSG(bodyIt != m_bodyListGroup->end(), "no bodies in list");

        Quaternion q_KI, q_BK;
        Vector3 I_r_IK, K_r_KB;
        Matrix33 Rot_KI; // Temp

        // Iterate over all values in the list
        for ( XMLNodeType & node : initCond.children("Value")) {

            if(consumedValues >= m_bodyListGroup->size()) {
                LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> InitialPositionTransforms: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            } else if(consumedValues != bodyIt->m_initState.m_id - m_startIdGroup) {
                // this value does not correspond to the linear offset from the start
                ++consumedValues;
                continue;
            }


            setQuaternionZero(q_KI);
            I_r_IK.setZero();


            // Iterate over all transforms an successfully applying the total trasnformation!
            for ( XMLNodeType & transf : initCond.children("Transform")) {

                Vector3 trans;
                if(!Utilities::stringToVector3(trans, transf.attribute("translation").value())) {
                    THROWEXCEPTION("---> String conversion in InitialPositionTransforms: translation failed");
                }
                Vector3 axis;
                if(!Utilities::stringToVector3(axis, transf.attribute("rotationAxis").value())) {
                    THROWEXCEPTION("---> String conversion in InitialPositionTransforms: rotationAxis failed");
                }

                if( axis.norm() == 0) {
                    THROWEXCEPTION("---> Specified wrong axis in InitialPositionTransforms");
                }

                PREC angle;
                auto att = transf.attribute("angleDegree");
                if(att) {
                    if(!Utilities::stringToType(angle, att.value())) {
                        THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: angleDegree failed");
                    }
                    angle = angle / 180 * M_PI;
                } else {
                    att = transf.attribute("angleRadian");
                    if(att){
                        if(!Utilities::stringToType(angle, att.value())) {
                            THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: angleRadian failed");
                        }
                    }
                    else{
                        THROWEXCEPTION("---> No angle found in InitialPositionPosAxisAngle");
                    }
                }





                setQuaternion(q_BK,axis,angle);
                K_r_KB = trans;
                Rot_KI = getRotFromQuaternion<PREC>(q_KI);
                I_r_IK += Rot_KI * K_r_KB; // Transforms like A_IK * A_r_AB;
                q_KI = quatMult(q_KI,q_BK); // Sequential (aktiv) rotation

            }

            // Apply overall transformation!
            bodyIt->m_initState.m_q.template head<3>() = I_r_IK;
            bodyIt->m_initState.m_q.template tail<4>() = q_KI;

            ++consumedValues;
            ++bodyIt;
        }

        if(consumedValues < m_bodyListGroup->size()) {
            LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> InitialPositionTransforms: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
            auto itEnd = m_bodyListGroup->end();
            for(; bodyIt !=  itEnd; ++bodyIt) {
                bodyIt->m_initState.m_q.template head<3>() = I_r_IK;
                bodyIt->m_initState.m_q.template tail<4>() = q_KI;
            }
        }
    }
    void parseInitialVelocityTransRot(XMLNodeType initCond) {
        unsigned int consumedValues = 0;
        Vector3 transDir,rotDir;
        PREC rot,vel;

        auto bodyIt = m_bodyListGroup->begin();
        GRSF_ASSERTMSG(bodyIt != m_bodyListGroup->end(),"no bodies in list");

        // Iterate over all values in the list
        for ( XMLNodeType & node : initCond.children("Value")) {

            if(consumedValues >= m_bodyListGroup->size()) {
                LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> InitialPositionTransforms: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            } else if(consumedValues != bodyIt->m_initState.m_id - m_startIdGroup) {
                // this value does not correspond to the linear offset from the start
                ++consumedValues;
                continue;
            }

            if(!Utilities::stringToVector3(transDir, node.attribute("transDir").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: transDir failed");
            }
            transDir.normalize();

            if(!Utilities::stringToType(vel, node.attribute("absTransVel").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: absTransVel failed");
            }

            if(!Utilities::stringToVector3(rotDir, node.attribute("rotDir").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: transDir failed");
            }
            rotDir.normalize();

            if(!Utilities::stringToType(rot, node.attribute("absRotVel").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: absTransVel failed");
            }

            bodyIt->m_initState.m_u.template head<3>() = transDir*vel;
            bodyIt->m_initState.m_u.template tail<3>() = rotDir*rot;

            ++consumedValues;
            ++bodyIt;
        }

        if(consumedValues < m_bodyListGroup->size()) {
            LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> InitialVelocityTransRot: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
            auto itEnd = m_bodyListGroup->end();
            for(; bodyIt !=  itEnd; ++bodyIt) {
                bodyIt->m_initState.m_u.template head<3>() = transDir*vel;
                bodyIt->m_initState.m_u.template tail<3>() = rotDir*rot;
            }
        }

    }

};



/** SceneParser Options */
struct BodyModuleOptions {
    BodyModuleOptions() = default;
    BodyModuleOptions(const BodyModuleOptions& o) = default;
    BodyModuleOptions(BodyModuleOptions&& o) = default;
    BodyModuleOptions& operator=(const BodyModuleOptions& o) = default;
    BodyModuleOptions& operator=(BodyModuleOptions&& o) = default;

    using BodyRangeType = Range<RigidBodyIdType>;
    BodyRangeType m_bodyIdRange;       ///< Range of body ids, original list which is handed to parseScene
    bool m_parseAllBodiesNonSelGroup = true;  ///< Parse all bodies in groups where m_bodyIdRange is not applied (enableSelectiveIds="false) (default= true)
    bool m_parseOnlySimBodies = false;         ///< Parses only simulated bodies (default= false)
    bool m_allocateBodies = true;
    bool m_parseOnlyVisualizationProperties = false;
};

template<typename TParser>
class BodyModule {
private:

    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
    DEFINE_LAYOUT_CONFIG_TYPES

    using RigidBodyType = typename DynamicsSystemType::RigidBodyType;
    using RigidBodySimContainerType = typename DynamicsSystemType::RigidBodySimContainerType ;
    using RigidBodyStaticContainerType = typename DynamicsSystemType::RigidBodyStaticContainerType ;

    RigidBodySimContainerType    * m_pSimBodies = nullptr;
    RigidBodyStaticContainerType * m_pBodies    = nullptr;

    ParserType * m_parser;

public:

    using OptionsType = BodyModuleOptions;

    BodyModule(ParserType * p, GeometryModuleType * g,  InitStatesModuleType * is, VisModuleType * i,
               RigidBodySimContainerType * simBodies, RigidBodyStaticContainerType * bodies )
        : m_parser(p), m_pGeomMod(g), m_pVisMod(i), m_pInitStatesMod(is), m_pSimBodies(simBodies), m_pBodies(bodies) {
            GRSF_ASSERTMSG(is,"should not be null");
            GRSF_ASSERTMSG(g,"should not be null");
        };


    void parse(XMLNodeType & sceneObjects){
        LOGSCLEVEL1(m_parser->m_pSimulationLog, "=== BodyModule: parsing ==========================================="<<std::endl;)

        GRSF_ASSERTMSG(m_parsingOptions.m_allocateBodies || m_parsingOptions.m_parseOnlyVisualizationProperties, " You should not allocate any bodies (m_allocateBodies: "<<m_parsingOptions.m_allocateBodies
                                                <<" and parse dynamic properties (m_parseOnlyVisualizationProperties: " << m_parsingOptions.m_parseOnlyVisualizationProperties <<
                                                ")")

        LOGSCLEVEL1( m_parser->m_pSimulationLog, "---> BodyModule Options: " <<std::endl
                        <<"\t parse only simulated bodies:"<<m_parsingOptions.m_parseOnlySimBodies << std::endl
                        <<"\t parse all bodies in group with disabled selective ids:"<<m_parsingOptions.m_parseAllBodiesNonSelGroup << std::endl
                        <<"\t parse selective ids: "<< m_parseSelectiveBodyIds << std::endl
                        <<"\t allocate bodies: "<< m_parsingOptions.m_allocateBodies << std::endl;)

        for ( XMLNodeType & node  : sceneObjects.children("RigidBodies")) {
                parseRigidBodies(node);
        }
        LOGSCLEVEL1(m_parser->m_pSimulationLog, "==================================================================="<<std::endl;)
    }


    void setParsingOptions(OptionsType o) {
        m_parsingOptions = std::move(o);

        if(m_parsingOptions.m_bodyIdRange.empty()) {
            m_parseSelectiveBodyIds = false;
        } else {
            m_parseSelectiveBodyIds = true;
        }



        m_startRangeIdIt = m_parsingOptions.m_bodyIdRange.begin();

    }

private:

    void parseRigidBodies( XMLNodeType  rigidbodies ) {

        LOGSCLEVEL1(m_parser->m_pSimulationLog,"==================================" << std::endl <<
                    "---> Parse RigidBodies, group name: "<< rigidbodies.attribute("name").value() << std::endl;);

        //Clear current body list;
        m_bodyListGroup.clear();

        // Check if this group has selecitveIdsOnly turned on to load only selective bodies
        bool hasSelectiveFlag = false;
        auto att = rigidbodies.attribute("enableSelectiveIds");
        if(att){
            if(!Utilities::stringToType(hasSelectiveFlag, att.value())) {
                    THROWEXCEPTION("---> String conversion in parseRigidBodies: enableSelectiveIds failed");
            }
        }

        unsigned int instances;
        if(!Utilities::stringToType(instances, rigidbodies.attribute("instances").value())) {
                THROWEXCEPTION("---> String conversion in parseRigidBodies: instances failed");
        }
        // Determine what DynamicState the group has:
        XMLNodeType  dynPropNode;
        GET_XMLCHILDNODE_CHECK(dynPropNode,"DynamicProperties",rigidbodies);
        parseDynamicState(dynPropNode);

        // Determine GroupId (if specified, otherwise maximum)
        unsigned int groupId;
        att = rigidbodies.attribute("groupId");
        if(att) {
            if(!Utilities::stringToType<unsigned int>(groupId, att.value())) {
                THROWEXCEPTION("---> String conversion in parseRigidBodies: groupId failed");
            }
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
        if( (!m_parsingOptions.m_parseAllBodiesNonSelGroup && !hasSelectiveFlag)
            || ( m_eBodiesState != RigidBodyType::BodyMode::SIMULATED  && m_parsingOptions.m_parseOnlySimBodies)
            || (m_parseSelectiveBodyIds && hasSelectiveFlag && m_startRangeIdIt== m_parsingOptions.m_bodyIdRange.end()) // out of m_bodyIdRange, no more id's which could be parsed in
             ) {
            LOGSCLEVEL2(m_parser->m_pSimulationLog, "---> Skip Group" << std::endl;)
            // update the number of bodies in this group and skip this group xml node
            *currGroupIdToNBodies += instances;
            return;
        }

        // Full id range for this group would be [m_starBodyId , endBodyId ]
        m_startIdGroup = RigidBodyId::makeId(startBodyNr, groupId);
        RigidBodyIdType endBodyId = RigidBodyId::makeId(startBodyNr+instances-1, groupId);
        LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> Group range: [" << RigidBodyId::getBodyIdString(m_startIdGroup)
                             << "," << RigidBodyId::getBodyIdString(endBodyId) << "]" << std::endl;)

        typename BodyRangeType::iterator bodyIdIt;
        bool updateStartRange = false;

        if(m_parseSelectiveBodyIds && hasSelectiveFlag){
            // parse group selective , determine start iterator in bodyRang
            m_bodyIdRangePtr = &m_parsingOptions.m_bodyIdRange;
            m_startRangeIdIt = std::lower_bound(m_startRangeIdIt,m_bodyIdRangePtr->end(),m_startIdGroup);
            bodyIdIt = m_startRangeIdIt;
            if( m_startRangeIdIt == m_bodyIdRangePtr->end()){ // no ids in the range
                *currGroupIdToNBodies += instances;
                LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> No ids in range: skip" << std::endl;)
                return;
            }
            LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> Selective range startId: " <<
                                  RigidBodyId::getBodyIdString(*m_startRangeIdIt) << std::endl;)
            updateStartRange = true;
        }else{
            // parse all bodies;
            //overwrite range containing all bodies, if we don't parse selective ids, parse all bodies!
            m_bodyIdRangePtr = &m_bodyIdRangeTmp;
            m_bodyIdRangeTmp = std::make_pair(m_startIdGroup,endBodyId+1);
            bodyIdIt = m_bodyIdRangePtr->begin();
            LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> overwrite selective range... " << std::endl;)
        }

        // Adding bodies in the range =============================
        // iterator bodyRange till the id is > endBodyId or we are out of the bodyIdRange
        auto itEnd = m_bodyIdRangePtr->end();
        m_parsedInstancesGroup = 0;

        m_bodyListGroup.resize(m_bodyIdRangePtr->size());

        auto bodyIt = m_bodyListGroup.begin();
        for( /* nothing*/ ; (bodyIdIt != itEnd) && ( *bodyIdIt <= endBodyId); ++bodyIdIt )
        {
            LOGSCLEVEL3(m_parser->m_pSimulationLog,"---> Added RigidBody Instance: "<<RigidBodyId::getBodyIdString(*bodyIdIt)<<std::endl);
            // Push new body
            if(m_parsingOptions.m_allocateBodies){
                *bodyIt = BodyData(new RigidBodyType(*bodyIdIt), *bodyIdIt, Vector3(1,1,1));
            }else{
                // add no bodies for visualization stuff, we dont need it!
                *bodyIt = BodyData( nullptr, *bodyIdIt, Vector3(1,1,1));
            }
            ++m_parsedInstancesGroup;
            ++bodyIt;
        }

        // Only update start range for selective parsing;
        if(updateStartRange){ m_startRangeIdIt = bodyIdIt;}

        LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> Added "<<m_parsedInstancesGroup<<" RigidBody Instances..."<<std::endl;);
        // =======================================================

        // Parse Geometry
        XMLNodeType geometryNode;
        GET_XMLCHILDNODE_CHECK(geometryNode, "Geometry",rigidbodies);
        if(m_pGeomMod){
            m_pGeomMod->parseGeometry(geometryNode, &m_bodyListGroup, m_startIdGroup);
        }

         //Parse DynamicsProperties
        parseDynamicProperties(dynPropNode);

        //Copy the pointers!
        if(m_eBodiesState == RigidBodyType::BodyMode::SIMULATED) {
            LOGSCLEVEL1(m_parser->m_pSimulationLog,"---> Copy Simulated RigidBody References to DynamicSystem ..."<<std::endl;);
            bool added = addAllBodies(m_pSimBodies);
            if(!added) {THROWEXCEPTION("Could not add body to m_SimBodies!, some bodies exist already in map!");};
            m_nSimBodies += m_parsedInstancesGroup;
            m_nBodies += m_parsedInstancesGroup;
        } else if(m_eBodiesState == RigidBodyType::BodyMode::STATIC) {
            LOGSCLEVEL1(m_parser->m_pSimulationLog,"---> Copy Static RigidBody References to DynamicSystem ..."<<std::endl;);
            bool added = addAllBodies(m_pBodies);
            if(!added) {THROWEXCEPTION("Could not add body to m_Bodies!, some bodies exist already in map!");};
            m_nBodies += m_parsedInstancesGroup;
        } else {
            THROWEXCEPTION("---> Adding only simulated and not simulated objects supported!");
        }

        XMLNodeType  visualizationNode = rigidbodies.child("Visualization");
        if(m_pVisMod){
            m_pVisMod->parse(visualizationNode, &m_bodyListGroup, m_startIdGroup, m_eBodiesState );
        }
        // ===============================================================================================================


        LOGSCLEVEL1(m_parser->m_pSimulationLog, "==================================" << std::endl;);
    }

    void parseDynamicProperties( XMLNodeType  dynProp) {
        LOGSCLEVEL2(m_parser->m_pSimulationLog,"---> Parse DynamicProperties ..."<<std::endl;);

        // DynamicState has already been parsed for the group!
        if(m_eBodiesState == RigidBodyType::BodyMode::SIMULATED) {
            parseDynamicPropertiesSimulated(dynProp);
        } else if(m_eBodiesState == RigidBodyType::BodyMode::STATIC) {
            parseDynamicPropertiesStatic(dynProp);
        }
    }
    void parseDynamicState(XMLNodeType  dynProp){

        std::string type =  dynProp.child("DynamicState").attribute("type").value();
        if(type == "simulated") {
            m_eBodiesState =  RigidBodyType::BodyMode::SIMULATED;
        } else if(type == "not simulated" || type == "static") {
            m_eBodiesState =  RigidBodyType::BodyMode::STATIC;
        } else if(type == "animated") {
            m_eBodiesState =  RigidBodyType::BodyMode::ANIMATED;
            THROWEXCEPTION("---> The attribute 'type' '" << type << "' of 'DynamicState' has no implementation in the parser");
        } else {
            THROWEXCEPTION("---> The attribute 'type' '" << type << "' of 'DynamicState' has no implementation in the parser");
        }

    }
    void parseDynamicPropertiesSimulated( XMLNodeType  dynProp) {

        std::string s;
        XMLNodeType n;
        if(!m_parsingOptions.m_parseOnlyVisualizationProperties) {
            // First allocate a new SolverDate structure
            for(auto & b : m_bodyListGroup)  {
                b.m_body->m_pSolverData = new typename RigidBodyType::RigidBodySolverDataType();
                // apply first to all bodies :-)
                b.m_body->m_eState = m_eBodiesState;
            }

            // Mass ============================================================
            n = dynProp.child("Mass");
            s = n.attribute("distribute").value();
            if(s == "uniform") {
                PREC mass;
                if(!Utilities::stringToType(mass, n.attribute("value").value())) {
                    THROWEXCEPTION("---> String conversion in Material: id failed");
                }
                for(auto & b : m_bodyListGroup) {
                    b.m_body->m_mass = mass;
                }
            } else {
                THROWEXCEPTION("---> The attribute 'distribute' '" << s << "' of 'Mass' has no implementation in the parser");
            }

            // Material
            XMLNodeType n = dynProp.child("Material");
            parseMaterial(n);

            // InertiaTensor ============================================================
            s= dynProp.child("InertiaTensor").attribute("type").value();
            if(s == "homogen") {
                for(auto & b : m_bodyListGroup) {
                    InertiaTensor::calculateInertiaTensor(b.m_body);
                }
            } else {
                THROWEXCEPTION("---> The attribute 'type' '" << s <<"' of 'InertiaTensor' has no implementation in the parser");
            }
        }

        // InitialPosition ============================================================
        GET_XMLCHILDNODE_CHECK(n,"InitialCondition",dynProp)
        bool parseVel = (m_parsingOptions.m_parseOnlyVisualizationProperties)? false : true;
        if(m_pInitStatesMod){
            m_pInitStatesMod->parseInitialCondition(n,&m_bodyListGroup,m_startIdGroup,parseVel);
        }

    }
    void parseDynamicPropertiesStatic( XMLNodeType  dynProp) {
        XMLNodeType n ;
        if(!m_parsingOptions.m_parseOnlyVisualizationProperties) {
            n = dynProp.child("Material");
            parseMaterial(n);
        }

         // InitialPosition ============================================================
        GET_XMLCHILDNODE_CHECK(n,"InitialCondition",dynProp)
        bool parseVel = false;
        if(m_pInitStatesMod){
            m_pInitStatesMod->parseInitialCondition(n,&m_bodyListGroup,m_startIdGroup,parseVel,false);
        }


    }

    void parseMaterial(XMLNodeType n){
        std::string s = n.attribute("distribute").value();
        if(s == "uniform") {
            typename RigidBodyType::BodyMaterialType eMaterial = 0;

            if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(eMaterial, n.attribute("id").value())) {
                THROWEXCEPTION("---> String conversion in Material: id failed");
            }

            for(auto & b : m_bodyListGroup) {
                b.m_body->m_eMaterial = eMaterial;
            }
        } else {
            THROWEXCEPTION("---> The attribute 'distribute' '" << s  <<"' of 'Material' has no implementation in the parser");
        }
    }



    /** General helper template to add all result to the lists */
    template<typename BodyContainer>
    inline bool addAllBodies(BodyContainer * bodies){
        bool added = true;
        if(bodies){
            for( auto & b : m_bodyListGroup){
                added &= bodies->addBody(b.m_body);
            }
        }
        return added;
    }



    BodyModuleOptions m_parsingOptions;
    using BodyRangeType = typename BodyModuleOptions::BodyRangeType;
    bool m_parseSelectiveBodyIds;      ///< Use the m_bodyIdRange to only load the selective ids in the group which use enableSelectiveIds="true"

    /// Parsing helpers
    BodyRangeType m_bodyIdRangeTmp;                     ///< Range of bodies, temporary for load of all bodies
    typename BodyRangeType::iterator m_startRangeIdIt;  ///< Current iterator which marks the start for the current group in m_bodyIdRange
    BodyRangeType * m_bodyIdRangePtr;                   ///< Switches between m_bodyIdRangeTmp/m_bodyIdRange, depending on parsing state

    unsigned int m_parsedInstancesGroup;            ///< Number of instances generated in the current group
    RigidBodyIdType m_startIdGroup;                  ///< Start id of the current group smaller or equal to *m_startRangeIdIt


    unsigned int m_nSimBodies, m_nBodies;
    typedef std::unordered_map<unsigned int,unsigned int> GroupToNBodyType;
    GroupToNBodyType m_groupIdToNBodies;
    unsigned int m_globalMaxGroupId; // Group Id used to build a unique id!

    /// Temprary structures for each sublist (groupid=?) of rigid bodies
    typename RigidBodyType::BodyMode m_eBodiesState;     ///< Used to parse a RigidBody Node

    struct BodyData {
        BodyData(): m_body(nullptr),m_scale(Vector3(1,1,1)){}
        BodyData( RigidBodyType * p, const RigidBodyIdType & id, const Vector3 & s = Vector3(1,1,1))
            : m_body(p),m_scale(s),m_initState(id) {}

        RigidBodyType* m_body; // might be also zero (if we dont need the whole body for visualization only)
        Vector3 m_scale;
        RigidBodyState m_initState;
    };

public:
    using BodyListType = std::vector<BodyData>;
private:

    BodyListType m_bodyListGroup; ///< Used to parse a RigidBody Node


    /// Other Modules
    GeometryModuleType * m_pGeomMod;
    InitStatesModuleType * m_pInitStatesMod;
    VisModuleType * m_pVisMod;

    RigidBodySimContainerType * pSimBodies;

};


};


/** SceneParser Options */
struct SceneParserOptions {
    bool m_parseSceneSettings = true; ///< Parse SceneSettings (default= true)
    bool m_parseSceneObjects  = true;  ///< Parse SceneObjects, (default= true)
};


/** These module types are defined when there is no derivation from scene parser */
template<typename TSceneParser>
struct SceneParserModuleTraits{
    using SettingsModuleType         = ParserModules::SettingsModule<TSceneParser>;
    using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<TSceneParser>;
    using ContactParamModuleType     = ParserModules::ContactParamModule<TSceneParser>;
    using InitStatesModuleType       = ParserModules::InitStatesModule<TSceneParser> ;

    using BodyModuleType             = ParserModules::BodyModule< TSceneParser > ;
    using GeometryModuleType         = ParserModules::GeometryModule<TSceneParser>;

    using VisModuleType              = ParserModules::VisModuleDummy<TSceneParser>;

};

template<typename TDynamicsSystem, template<typename P> class TModuleTraits = SceneParserModuleTraits, typename TDerived = void >
class SceneParser {
public:
    using DynamicsSystemType = TDynamicsSystem;

    using XMLNodeType = pugi::xml_node;
    using XMLAttributeType = pugi::xml_attribute;

    using RandomGenType = typename DynamicsSystemType::RandomGenType;
    template<typename T>
    using UniformDistType = std::uniform_real_distribution<T>;

    /** Modules defintions
    * This type traits define the module types of the derived class if any exist , otherwise it defines the standart module types
    */
    using DerivedType = typename std::conditional< std::is_same<TDerived,void>::value, SceneParser, TDerived>::type;
    using ModuleTypeTraits = TModuleTraits<DerivedType>;
    DEFINE_MODULETYPES_AND_FRIENDS(ModuleTypeTraits)

    using BodyModuleOptionsType = typename BodyModuleType::OptionsType;

public:

    SceneParser( std::shared_ptr<DynamicsSystemType> pDynSys):m_pDynSys(pDynSys) {
        // Get all Modules from DynamicsSystem
        std::tie(m_pSettingsModule, m_pExternalForcesModule, m_pContactParamModule,m_pInitStatesModule, m_pBodyModule, m_pGeometryModule, m_pVisModule)
            = m_pDynSys->template createParserModules<DerivedType>( static_cast<DerivedType*>(this));

//        // Set log
//        m_pSimulationLog = nullptr;
//        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
//        GRSF_ASSERTMSG(m_pSimulationLog, "There is no SimulationLog in the LogManager!");
//        setStandartValues();

    }

    /**
    * range is only applied to the groups with the attribute enableSelectiveIds="true"
    */
    template<typename TParserOptions = SceneParserOptions, typename TBodyParserOptions = BodyModuleOptionsType >
    bool parseScene( const boost::filesystem::path & file,
                     TParserOptions&& opt = TParserOptions(),
                     TBodyParserOptions&& optBody = TBodyParserOptions()
                   ) {
//        // Forward all settings
//        m_parsingOptions = std::forward<TParserOptions>(opt);
//
//        if(m_pBodyModule) {
//            m_pBodyModule->setParsingOptions(std::forward<TBodyParserOptions>(optBody));
//        }
//
//        parseSceneIntern(file);
    }

    virtual void cleanUp() {
        // Delegate all cleanUp stuff to the modules!
    }

    boost::filesystem::path getParsedSceneFile() {
//        return m_currentParseFilePath;
    }

    void checkFileExists(boost::filesystem::path file) {
//        if( !boost::filesystem::exists(file) ) {
//            THROWEXCEPTION("---> The file ' " + file.string() + "' does not exist!");
//        }
    }

protected:

    bool parseSceneIntern(const boost::filesystem::path & file) {
//
//        LOGSCLEVEL1( m_pSimulationLog, "---> SceneParser Options: " <<std::endl <<
//                        "\t parse scene settings:"<<m_parsingOptions.m_parseSceneSettings << std::endl<<
//                        "\t parse scene objects:"<<m_parsingOptions.m_parseSceneObjects << std::endl;);
//
//        // Setting path
//        bool reparse =false;
//        if(file != m_currentParseFilePath) {
//            reparse = true;
//        }
//        m_currentParseFilePath = file;
//        m_currentParseFileDir = m_currentParseFilePath.parent_path();
//
//
//        LOGSCLEVEL1( m_pSimulationLog, "---> Scene Input file: "  << file.string() <<std::endl; );
//
//        if(!boost::filesystem::exists(m_currentParseFilePath)) {
//            GRSF_ERRORMSG("Scene Input file does not exist!");
//        }
//
//
//        try {
//
//            if(reparse) {
//                pugi::xml_parse_result result = m_xmlDoc.load_file(m_currentParseFilePath.c_str());
//                if (result) {
//                    LOGSCLEVEL1(m_pSimulationLog, "---> Loaded XML [" << m_currentParseFilePath.string() << "] without errors!" << std::endl;);
//                } else {
//                    THROWEXCEPTION( "Loaded XML [" << m_currentParseFilePath.string() << "] with errors!" << std::endl
//                                    << "Error description: " << result.description() << std::endl
//                                    << "Error offset: " << result.offset )
//                }
//            }
//
//            LOGSCLEVEL1(m_pSimulationLog, "---> Try to parse the scene ..."<<std::endl;);
//
//            GET_XMLCHILDNODE_CHECK( m_xmlRootNode, "DynamicsSystem" , m_xmlDoc );
//
//            XMLNodeType node;
//            GET_XMLCHILDNODE_CHECK( node , "SceneSettings",  m_xmlRootNode);
//            parseSceneSettingsPre(node);
//
//            node = m_xmlRootNode.child("SceneObjects");
//            parseSceneObjects(node);
//
//            node = m_xmlRootNode.child("SceneSettings");
//            parseSceneSettingsPost(node);
//
//            //parseOtherOptions(m_xmlRootNode);
//
//
//        } catch(Exception& ex) {
//            LOGSCLEVEL1(m_pSimulationLog,  "Scene XML error: "  << ex.what() <<std::endl);
//            GRSF_ERRORMSG( "Scene XML error: "  << ex.what() );
//        }

    }


    virtual void parseSceneSettingsPre( XMLNodeType sceneSettings ) {
//
//        if(!m_parsingOptions.m_parseSceneSettings) {
//            LOGSCLEVEL1(m_pSimulationLog,"---> Skip SceneSettings"<<std::endl;);
//            return;
//        }
//
//        LOGSCLEVEL1(m_pSimulationLog,"---> Parse SceneSettings..."<<std::endl;);
//
//        if(m_pSettingsModule) {
//            m_pSettingsModule->parse(sceneSettings);
//        }
//
//        if(m_pContactParamModule) {
//            m_pContactParamModule->parse(sceneSettings);
//        }
//
//        if(m_pExternalForcesModule) {
//            m_pExternalForcesModule->parse(sceneSettings);
//        }
//
//        if(m_pGeometryModule) {
//            m_pGeometryModule->parseGlobalGeometries(sceneSettings);
//        }
    }

    virtual void parseSceneSettingsPost( XMLNodeType sceneSettings ) {
//        if(m_pInitStatesModule) {
//            m_pInitStatesModule->parseGlobalInitialCondition(sceneSettings);
//        }
//
//        if(m_pVisModule){
//            m_pVisModule->parseSceneSettingsPost(sceneSettings);
//        }
    }

    virtual void parseSceneObjects( XMLNodeType sceneObjects) {
//        if(m_pBodyModule){
//            m_pBodyModule->parse(sceneObjects);
//        }
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
    std::unique_ptr< GeometryModuleType>       m_pGeometryModule;
    std::unique_ptr< ExternalForcesModuleType> m_pExternalForcesModule;
    std::unique_ptr< ContactParamModuleType>   m_pContactParamModule;

    std::unique_ptr< BodyModuleType>           m_pBodyModule;
    std::unique_ptr< InitStatesModuleType>     m_pInitStatesModule;

    std::unique_ptr< VisModuleType>            m_pVisModule;
};



#endif


