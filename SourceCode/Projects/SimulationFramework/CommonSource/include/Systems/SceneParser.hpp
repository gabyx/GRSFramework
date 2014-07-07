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
    using RandomGenType = typename ParserType::RandomGenType; \
    template<typename T> using UniformDistType = typename ParserType::template UniformDistType<T>;\
    \
    using SettingsModuleType    = typename ParserType::SettingsModuleType;\
    using GeometryModuleType    = typename ParserType::GeometryModuleType;\
    using ContactParamModuleType= typename ParserType::ContactParamModuleType;\
    \
    using InitStatesModuleType  = typename ParserType::InitStatesModuleType ;\
    using BodyVisModuleType     = typename ParserType::BodyVisModuleType;\
    using BodyModuleType        = typename ParserType::BodyModuleType;\
    using ExternalForcesModuleType   = typename ParserType::ExternalForcesModuleType ;\


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
                    m_parser->checkFileExists(m_timestepperSettings->m_simStateReferenceFile);
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
    GeometryModule(ParserType * p, GlobalGeometryMapType * g): m_parser(p),m_globalGeometries(g) {};

    void parseGlobalGeometries(XMLNodeType sceneSettings) {
        LOG(m_parser->m_pSimulationLog,"---> Parse GlobalGeometry ..."<<std::endl;);
        if(m_globalGeometries) {
            XMLNodeType globalGeom = sceneSettings.child("GlobalGeometries");
            if(globalGeom) {
                for (XMLNodeType n : globalGeom.children()) {
                    parseGeometry(n);
                }
            }
        } else {
            SKIPLOG(m_parser->m_pSimulationLog, "---> GeometryModule: skipped GlobalGeometries ..."<<std::endl;)
        }
    }


    void parseGeometry( XMLNodeType geometryNode,  BodyListType * bodyList=nullptr, RigidBodyIdType startId=0) {
        m_startIdGroup = startId;
        m_bodyListGroup = bodyList;
        m_addToGlobalGeoms = m_bodyListGroup? false : true;

        if(m_addToGlobalGeoms)

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


private:


    template<typename T>
    void addToGlobalGeomList(unsigned int id,  std::shared_ptr<T> ptr) {



        auto ret = m_globalGeometries->insert(typename GlobalGeometryMapType::value_type( id, ptr) );
        if(ret.second == false) {
            THROWEXCEPTION("---> addToGlobalGeomList: geometry with id: " <<  id<< " exists already!");
        }
        LOG(m_parser->m_pSimulationLog,"---> Added geometry with id: " <<  id << " to global geometry list" <<std::endl;);

        // Print some details:
        LOG(m_parser->m_pSimulationLog,"\t---> GlobalGeomId: " << id <<std::endl);
        PrintGeometryDetailsVisitor(m_parser->m_pSimulationLog, ret.first->second, "\t\t--->");


    }

    /// Geometries ==============================================================================
    void parseSphereGeometry( XMLNodeType sphere) {
        std::string type = sphere.attribute("distribute").value();

        if(type == "uniform") {
            PREC radius;
            if(!Utilities::stringToType<PREC>(radius,sphere.attribute("radius").value())) {
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
                    LOG(m_parser->m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_initState.m_id) << ", GeometryType: Sphere" << std::endl);
                    LOG(m_parser->m_pSimulationLog, "\t\t---> radius: " << radius << std::endl; );
                    if(b.m_body) {
                        b.m_body->m_geometry = std::shared_ptr<SphereGeometry >(new SphereGeometry(radius));
                    }
                }
            }
        } else if(type == "random") {
            double minRadius;
            if(!Utilities::stringToType<double>(minRadius,sphere.attribute("minRadius").value())) {
                THROWEXCEPTION("---> String conversion in parseSphereGeometry: minRadius failed");
            }
            if( minRadius <= 0) {
                THROWEXCEPTION("---> In parseSphereGeometry: minRadius to small!");
            }

            double maxRadius;
            if(!Utilities::stringToType<double>(maxRadius,sphere.attribute("maxRadius").value())) {
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
                    LOG(m_parser->m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(bodyIt->m_initState.m_id)<< ", GeometryType: Sphere" << std::endl);
                    LOG(m_parser->m_pSimulationLog, "\t\t---> radius: " << radius << std::endl; );

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
            if(!Utilities::stringToVector3<PREC>(n, halfspace.attribute("normal").value())) {
                THROWEXCEPTION("---> String conversion in HalfsphereGeometry: normal failed");
            }

            Vector3 p;
            if(!Utilities::stringToVector3<PREC>(p, halfspace.attribute("position").value())) {
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

                if(m_bodyListGroup->begin() != m_bodyListGroup->end() && m_bodyListGroup->begin()->m_body != nullptr ) {
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
            if(!Utilities::stringToVector3<PREC>(extent, box.attribute("extent").value())) {
                THROWEXCEPTION("---> String conversion in BoxGeometry: extent failed");
            }

            Vector3 center;
            if(!Utilities::stringToVector3<PREC>(center, box.attribute("center").value())) {
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
        if(!Utilities::stringToType<bool>(bInstantiate,mesh.attribute("useInstance").value())) {
            THROWEXCEPTION("---> String conversion in parseMeshGeometry: useInstance failed");
        }

        std::string type = mesh.attribute("distribute").value();
        if(type == "uniform") {

            // import here an object model;
            // First make an meshinformatino structure

            boost::filesystem::path fileName =  mesh.attribute("file").value();
            m_parser->checkFileExists(fileName);

            Vector3 scale_factor;
            if(!Utilities::stringToVector3<PREC>(scale_factor, mesh.attribute("scale").value())) {
                THROWEXCEPTION("---> String conversion in parseMeshGeometry failed: scale");
            }
            if(scale_factor.norm()==0) {
                THROWEXCEPTION("---> Wrong scale factor (=0) specified in parseMeshGeometry!");
            }

            Vector3 trans;
            if(!Utilities::stringToVector3<PREC>(trans, mesh.attribute("translation").value())) {
                THROWEXCEPTION("---> String conversion in parseMeshGeometry: translation failed: ");
            }

            Vector3 axis;
            if(!Utilities::stringToVector3<PREC>(axis, mesh.attribute("rotationAxis").value())) {
                THROWEXCEPTION("---> String conversion in parseMeshGeometry: rotationAxis failed");
            }

            PREC angle;

            if(mesh.attribute("angleDegree")) {
                if(!Utilities::stringToType<PREC>(angle, mesh.attribute("angleDegree").value())) {
                    THROWEXCEPTION("---> String conversion in parseMeshGeometry: angleDegree failed");
                }
                angle = angle / 180 * M_PI;
            } else if(mesh.attribute("angleRadian")) {
                if(!Utilities::stringToType<PREC>(angle, mesh.attribute("angleRadian").value())) {
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
                if(!Utilities::stringToType<bool>(writeToLog, mesh.attribute("writeToLog").value())) {
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
                LOG(m_parser->m_pSimulationLog,"---> Geometry with id: " << id << " not found in global geometry list!" <<std::endl;);
                THROWEXCEPTION("---> Geometry search in parseGlobalGeomId: failed!");
            }

            for(auto & b : *m_bodyListGroup) {
                GetScaleOfGeomVisitor vis(b.m_scale);
                boost::apply_visitor(vis, it->second);
                b.m_body->m_geometry = it->second;
                b.m_body->m_globalGeomId = id;
                //LOG(m_parser->m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(*m_bodyListGroup[i]) << ", GlobalGeomId: " << id <<  std::endl);
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
                    LOG(m_parser->m_pSimulationLog,"---> parseGlobalGeomId: Geometry with id: " << startId+id << " not found in global geometry list!" <<std::endl;);
                    THROWEXCEPTION("---> parseGlobalGeomId: Geometry search failed!");
                }

                GetScaleOfGeomVisitor vis(b.m_scale);
                boost::apply_visitor(vis, it->second);
                b.m_body->m_geometry = it->second;
                b.m_body->m_globalGeomId = id;
                LOG(m_parser->m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_body) << ", GlobalGeomId: " << id <<  std::endl);

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
                    LOG(m_parser->m_pSimulationLog,"---> Geometry with id: " << id << " not found in global geometry list!" <<std::endl;);
                    THROWEXCEPTION("---> Geometry search in parseGlobalGeomId: failed!");
                }

                GetScaleOfGeomVisitor vis(b.m_scale);
                boost::apply_visitor(vis, it->second);
                b.m_body->m_geometry = it->second;
                b.m_body->m_globalGeomId = id;
                LOG(m_parser->m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_body) << ", GlobalGeomId: " << id <<  std::endl);
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

        LOG(m_parser->m_pSimulationLog,"---> ContactParamModule: parsing ..."<<std::endl;);

        if( !m_contactParams ) {
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
            for (XMLNodeType it : paramMap.children("ContactParameter")) {
                parseContactParameter(it);
            }
        }
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

            if(!Utilities::stringToType<PREC>(mu, contactParam.attribute("mu").value())) {
                THROWEXCEPTION("---> String conversion in ContactParameter: mu failed");
            }
            if(!Utilities::stringToType<PREC>(epsilonN, contactParam.attribute("epsilonN").value())) {
                THROWEXCEPTION("---> String conversion in ContactParameter: epsilonN failed");
            }
            if(!Utilities::stringToType<PREC>(epsilonT, contactParam.attribute("epsilonT").value())) {
                THROWEXCEPTION("---> String conversion in ContactParameter: epsilonT failed");
            }

            if(type == "UCFD") {
                PREC invDampingN, invDampingT;
                if(!Utilities::stringToType<PREC>(invDampingN, contactParam.attribute("invDampingN").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: invDampingN failed");
                }
                if(!Utilities::stringToType<PREC>(invDampingT, contactParam.attribute("invDampingT").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: invDampingT failed");
                }

                contactParameter = ContactParameter::createParams_UCFD_ContactModel(epsilonN,epsilonT,mu,invDampingN,invDampingT);


            } else if(type == "UCFDD") {

                PREC invDampingN, gammaMax, epsilon, invDampingTFix;
                if(!Utilities::stringToType<PREC>(invDampingN, contactParam.attribute("invDampingN").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: invDampingN failed");
                }

                if(!Utilities::stringToType<PREC>(invDampingTFix, contactParam.attribute("invDampingTFix").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: invDampingTFix failed");
                }
                if(!Utilities::stringToType<PREC>(gammaMax, contactParam.attribute("gammaMax").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: gamma_max failed");
                }
                if(!Utilities::stringToType<PREC>(epsilon, contactParam.attribute("epsilon").value())) {
                    THROWEXCEPTION("---> String conversion in ContactParameter: epsilon failed");
                }

                contactParameter = ContactParameter::createParams_UCFDD_ContactModel(epsilonN,epsilonT,mu,
                                   invDampingN,invDampingTFix,
                                   gammaMax,epsilon);
            } else {
                contactParameter = ContactParameter::createParams_UCF_ContactModel(epsilonN,epsilonT,mu);
            }

            if(stdMaterial) {
                LOG(m_parser->m_pSimulationLog,"---> Add ContactParameter standart"<<std::endl;);
                m_contactParams->setStandardValues(contactParameter);
            } else {
                LOG(m_parser->m_pSimulationLog,"---> Add ContactParameter standart of id="<<material1<<" to id="<<material2<<std::endl;);
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
        XMLNodeType externalForces = sceneSettings.child("ExternalForces");
        if(externalForces) {
            for (XMLNodeType n : externalForces.children()) {
                if( std::string(n.name()) == "ForceField") {
                    parseForceField( n );
                }
            }
        } else {
            SKIPLOG(m_parser->m_pSimulationLog, "---> ExternalForcesModule: skipping ..."<<std::endl;)
        }
    }

private:

    void parseForceField( XMLNodeType forceField) {

        bool enabled = false;
        if(!Utilities::stringToType<bool>(enabled, forceField.attribute("enabled").value())) {
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
                if(!Utilities::stringToType<PREC>(boostTime, forceField.attribute("boostTime").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: boostTime failed");
                }
                PREC pauseTime;
                if(!Utilities::stringToType<PREC>(pauseTime, forceField.attribute("pauseTime").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: pauseTime failed");
                }

                PREC startTime;
                if(!Utilities::stringToType<PREC>(startTime, forceField.attribute("startTime").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: startTime failed");
                }

                PREC endTime;
                if(!Utilities::stringToType<PREC>(endTime, forceField.attribute("endTime").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: endTime failed");
                }
                PREC amplitude;
                if(!Utilities::stringToType<PREC>(amplitude, forceField.attribute("amplitude").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: amplitude failed");
                }

                Vector3 boxMin;
                if(!Utilities::stringToVector3<PREC>(boxMin, forceField.attribute("minPoint").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: boxMin failed");
                }
                Vector3 boxMax;
                if(!Utilities::stringToVector3<PREC>(boxMax, forceField.attribute("maxPoint").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: boxMax failed");
                }

                bool randomOn;
                if(!Utilities::stringToType<bool>(randomOn, forceField.attribute("randomOn").value())) {
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
                LOG(m_parser->m_pSimulationLog,"---> added SpatialSphericalTimeRandomForceField ..."<<std::endl;);

            } else if(type == "gravity") {
                PREC abs;
                if(!Utilities::stringToType<PREC>(abs, forceField.attribute("value").value())) {
                    THROWEXCEPTION("---> String conversion in parseForceField: value failed");
                }
                Vector3 dir;
                if(!Utilities::stringToVector3<PREC>(dir, forceField.attribute("direction").value())) {
                    THROWEXCEPTION("---> String conversion in SceneSettings: gravity failed");
                }
                dir.normalize();
                dir *= abs;

                m_forceList->addExternalForceCalculation(new GravityForceField(dir));

                LOG(m_parser->m_pSimulationLog,"---> added GravityForceField ..."<<std::endl;);
            } else {
                THROWEXCEPTION("---> String conversion in parseForceField: type failed");
            }
        }
    }
};


template<typename TParser>
class BodyVisModuleDummy {
private:
    DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE
public:
    BodyVisModuleDummy(ParserType * p, BodyModuleType * b) {};
    void parse() {
        ERRORMSG("This is the standard BodyVisModule which does nothing!");
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

    typename RigidBodyType::BodyState m_eBodiesState;

public:
    InitStatesModule(ParserType * p, RigidBodyStatesContainerType * c, SettingsModuleType * s): m_parser(p),m_initStates(c), m_settings(s) {
        ASSERTMSG(m_initStates, "should not be null");
    };


    void parseGlobalInitialCondition( XMLNodeType sceneSettings) {
        XMLNodeType initCond = sceneSettings.child("GlobalInitialCondition");
        if(initCond) {
            bool enabled = false;
            if(!Utilities::stringToType<bool>(enabled, initCond.attribute("enabled").value())) {
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
                    if(!Utilities::stringToType<double>(time, initCond.attribute("time").value())) {
                        THROWEXCEPTION("---> String conversion in GlobalInitialCondition: time failed");
                    }
                } else {
                    THROWEXCEPTION("---> String conversion in GlobalInitialCondition: whichState failed");
                }

                boost::filesystem::path relpath = initCond.attribute("path").value();


                setupInitialConditionBodiesFromFile_imp(relpath, time, which);

                bool useTime = false;
                if(!Utilities::stringToType<bool>(useTime, initCond.attribute("useTimeToContinue").value())) {
                    THROWEXCEPTION("---> String conversion in GlobalInitialCondition: useTimeToContinue failed");
                }

                // Set the time in the dynamics system timestepper settings
                if(useTime) {
                    m_settings->getTimeStepperSettings()->m_startTime = time;
                }


            }

        }
    }

    void parseInitialCondition(XMLNodeType initCondNode, BodyListType * bodyList,
                               RigidBodyIdType startId, typename RigidBodyType::BodyState state, bool parseVelocity = true ) {
        ASSERTMSG(bodyList, "Should not be null!")

        m_eBodiesState = state;
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
            if(parseVelocity && m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {
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
            std::cout <<"state::" << b.m_initState.m_q.transpose() << " , " << b.m_initState.m_u.transpose()  << std::endl;
            if(b.m_body) {
                InitialConditionBodies::applyBodyStateTo(b.m_initState,b.m_body);
            }

            added &= m_initStates->emplace(b.m_initState.m_id, b.m_initState).second;
        }
        if(!added) {THROWEXCEPTION("Could not add init state to m_initStates!, some bodies exist already in map!");};
    }


private:
    void setupInitialConditionBodiesFromFile_imp(boost::filesystem::path relpath, double &time , short which ) {

        InitialConditionBodies::setupInitialConditionBodiesFromFile(relpath,*m_initStates,time,true,true,which);
        LOG(m_parser->m_pSimulationLog,"---> Found time: "<< time << " in " << relpath << std::endl;);

    }

    void parseInitialPositionLinear(XMLNodeType initCond) {

        Vector3 pos;
        if(!Utilities::stringToVector3<PREC>(pos, initCond.attribute("position").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionLinear: position Linear failed");
        }
        Vector3 dir;
        if(!Utilities::stringToVector3<PREC>(dir, initCond.attribute("direction").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionLinear: direction Linear failed");
        }
        PREC dist;
        if(!Utilities::stringToType<PREC>(dist, initCond.attribute("distance").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionLinear: distance  Linear failed");
        }
        bool jitter;
        if(!Utilities::stringToType(jitter, initCond.attribute("jitter").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionLinear: jitter Linear failed");
        }

        PREC delta;
        if(!Utilities::stringToType<PREC>(delta, initCond.attribute("delta").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionLinear: delta Linear failed");
        }

        unsigned int seed = 5;
        if(initCond.attribute("seed")) {
            if(!Utilities::stringToType(seed, initCond.attribute("seed").value())) {
                THROWEXCEPTION("---> String conversion in InitialPositionGrid: jitter seed failed");
            }
        }

        InitialConditionBodies::setupPositionBodiesLinear(m_bodyListGroup,m_startIdGroup,pos,dir,dist,jitter,delta,seed);

    }
    void parseInitialPositionGrid(XMLNodeType initCond) {

        Vector3 trans;
        if(!Utilities::stringToVector3<PREC>(trans, initCond.attribute("translation").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionGrid: translation failed");
        }
        int gridX;
        if(!Utilities::stringToType<int>(gridX, initCond.attribute("gridSizeX").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionGrid: gridSizeX failed");
        }
        int gridY;
        if(!Utilities::stringToType<int>(gridY, initCond.attribute("gridSizeY").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionGrid: gridSizeY failed");
        }
        PREC dist;
        if(!Utilities::stringToType<PREC>(dist, initCond.attribute("distance").value())) {
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
        if(!Utilities::stringToType<double>(delta, initCond.attribute("delta").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionGrid: delta failed");
        }

        InitialConditionBodies::setupPositionBodiesGrid(m_bodyListGroup,m_startIdGroup,gridX,gridY,dist,trans,jitter,delta, seed);
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
        ASSERTMSG(bodyIt != m_bodyListGroup->end(),"no bodies in list");

        // Iterate over all values in the list

        for ( XMLNodeType & node : initCond.children("Value")) {

            if(consumedValues >= m_bodyListGroup->size()) {
                LOG(m_parser->m_pSimulationLog,"---> InitialPositionPosAxisAngle: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            } else if(consumedValues != bodyIt->m_initState.m_id - m_startIdGroup) {
                // this value does not correspond to the linear offset from the start
                consumedValues++;
                continue;
            }


            if(!Utilities::stringToVector3<PREC>(pos, node.attribute("position").value())) {
                THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: position failed");
            }

            if(!Utilities::stringToVector3<PREC>(axis, node.attribute("axis").value())) {
                THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: axis failed");
            }

            if( axis.norm() == 0) {
                THROWEXCEPTION("---> Specified wrong axis in InitialPositionPosAxisAngle");
            }

            auto att = node.attribute("angleDegree");
            if(att) {
                if(!Utilities::stringToType<PREC>(angle, att.value())) {
                    THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: angleDegree failed");
                }
                angle = angle / 180 * M_PI;
            } else {
                att = node.attribute("angleRadian");
                if(att){
                    if(!Utilities::stringToType<PREC>(angle, att.value())) {
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
            LOG(m_parser->m_pSimulationLog,"---> InitialPositionPosAxisAngle: You specified to little values, -> applying last to all remainig bodies ..."<<std::endl;);
            auto itEnd = m_bodyListGroup->end();
            for(; bodyIt !=  itEnd; ++bodyIt) {
                InitialConditionBodies::setupPositionBodyPosAxisAngle(bodyIt->m_initState, pos, axis, angle);
            }
        }
    }
    void parseInitialPositionTransforms(XMLNodeType initCond) {

        unsigned int consumedValues = 0;

        auto bodyIt = m_bodyListGroup->begin();
        ASSERTMSG(bodyIt != m_bodyListGroup->end(), "no bodies in list");

        Quaternion q_KI, q_BK;
        Vector3 I_r_IK, K_r_KB;
        Matrix33 Rot_KI; // Temp

        // Iterate over all values in the list
        for ( XMLNodeType & node : initCond.children("Value")) {

            if(consumedValues >= m_bodyListGroup->size()) {
                LOG(m_parser->m_pSimulationLog,"---> InitialPositionTransforms: You specified to many transforms, -> neglecting ..."<<std::endl;);
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
                if(!Utilities::stringToVector3<PREC>(trans, transf.attribute("translation").value())) {
                    THROWEXCEPTION("---> String conversion in InitialPositionTransforms: translation failed");
                }
                Vector3 axis;
                if(!Utilities::stringToVector3<PREC>(axis, transf.attribute("rotationAxis").value())) {
                    THROWEXCEPTION("---> String conversion in InitialPositionTransforms: rotationAxis failed");
                }

                if( axis.norm() == 0) {
                    THROWEXCEPTION("---> Specified wrong axis in InitialPositionTransforms");
                }

                PREC angle;
                auto att = transf.attribute("angleDegree");
                if(att) {
                    if(!Utilities::stringToType<PREC>(angle, att.value())) {
                        THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: angleDegree failed");
                    }
                    angle = angle / 180 * M_PI;
                } else {
                    att = transf.attribute("angleRadian");
                    if(att){
                        if(!Utilities::stringToType<PREC>(angle, att.value())) {
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
            LOG(m_parser->m_pSimulationLog,"---> InitialPositionTransforms: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
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
        ASSERTMSG(bodyIt != m_bodyListGroup->end(),"no bodies in list");

        // Iterate over all values in the list
        for ( XMLNodeType & node : initCond.children("Value")) {

            if(consumedValues >= m_bodyListGroup->size()) {
                LOG(m_parser->m_pSimulationLog,"---> InitialPositionTransforms: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            } else if(consumedValues != bodyIt->m_initState.m_id - m_startIdGroup) {
                // this value does not correspond to the linear offset from the start
                ++consumedValues;
                continue;
            }

            if(!Utilities::stringToVector3<PREC>(transDir, node.attribute("transDir").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: transDir failed");
            }
            transDir.normalize();

            if(!Utilities::stringToType<PREC>(vel, node.attribute("absTransVel").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: absTransVel failed");
            }

            if(!Utilities::stringToVector3<PREC>(rotDir, node.attribute("rotDir").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: transDir failed");
            }
            rotDir.normalize();

            if(!Utilities::stringToType<PREC>(rot, node.attribute("absRotVel").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: absTransVel failed");
            }

            bodyIt->m_initState.m_u.template head<3>() = transDir*vel;
            bodyIt->m_initState.m_u.template tail<3>() = rotDir*rot;

            ++consumedValues;
            ++bodyIt;
        }

        if(consumedValues < m_bodyListGroup->size()) {
            LOG(m_parser->m_pSimulationLog,"---> InitialVelocityTransRot: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
            auto itEnd = m_bodyListGroup->end();
            for(; bodyIt !=  itEnd; ++bodyIt) {
                bodyIt->m_initState.m_u.template head<3>() = transDir*vel;
                bodyIt->m_initState.m_u.template tail<3>() = rotDir*rot;
            }
        }

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
         typename TVisModule>
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

    using VisModType = TVisModule ;

    BodyModule(ParserType * p, GeometryModuleType * g,  InitStatesModuleType * is, std::unique_ptr<VisModType> i,
               RigidBodySimContainerType * simBodies, RigidBodyStaticContainerType * bodies )
        : m_parser(p), m_pGeomMod(g), m_pVisMod(std::move(i)), m_pInitStatesMod(is), m_pSimBodies(simBodies), m_pBodies(bodies) {};


    void parse(XMLNodeType & sceneObjects){

        m_parseOnlyVisualizationProperties = (!m_pSimBodies || !m_pBodies)? true : false;
        m_allocateBodies = !m_parseOnlyVisualizationProperties;

        for ( XMLNodeType & node  : sceneObjects.children("RigidBodies")) {
                parseRigidBodies(node);
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

    void parseRigidBodies( XMLNodeType  rigidbodies ) {

        LOG(m_parser->m_pSimulationLog,"---> Parse RigidBodies, group name: "<< rigidbodies.attribute("name").value() << std::endl;);

        //Clear current body list;
        m_bodyListGroup.clear();

        // Check if this group has selecitveIdsOnly turned on to load only selective bodies
        bool hasSelectiveFlag = false;

        auto att = rigidbodies.attribute("enableSelectiveIds");
        if(!Utilities::stringToType<bool>(hasSelectiveFlag, att.value())) {
                THROWEXCEPTION("---> String conversion in parseRigidBodies: enableSelectiveIds failed");
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
            || ( m_eBodiesState != RigidBodyType::BodyState::SIMULATED  && m_parsingOptions.m_parseOnlySimBodies)
            || (m_parseSelectiveBodyIds && hasSelectiveFlag && m_startRangeIdIt==m_bodyIdRange.end()) // out of m_bodyIdRange, no more id's which could be parsed in
             ) {
            LOG(m_parser->m_pSimulationLog, "Skip Group" << std::endl;)
            // update the number of bodies in this group and skip this group xml node
            *currGroupIdToNBodies += instances;
            return;
        }

        // Full id range for this group would be [m_starBodyId , endBodyId ]
        m_startIdGroup = RigidBodyId::makeId(startBodyNr, groupId);
        RigidBodyIdType endBodyId = RigidBodyId::makeId(startBodyNr+instances-1, groupId);
        LOG(m_parser->m_pSimulationLog,"---> Group range: [" << RigidBodyId::getBodyIdString(m_startIdGroup)
                             << "," << RigidBodyId::getBodyIdString(endBodyId) << "]" << std::endl;)

        typename BodyRangeType::iterator bodyIdIt;
        bool updateStartRange = false;

        if(m_parseSelectiveBodyIds && hasSelectiveFlag){
            // parse group selective , determine start iterator in bodyRang
            m_bodyIdRangePtr = &m_bodyIdRange;
            m_startRangeIdIt = std::lower_bound(m_startRangeIdIt,m_bodyIdRangePtr->end(),m_startIdGroup);
            bodyIdIt = m_startRangeIdIt;
            if( m_startRangeIdIt == m_bodyIdRangePtr->end()){ // no ids in the range
                *currGroupIdToNBodies += instances;
                LOG(m_parser->m_pSimulationLog,"---> No ids in range: skip" << std::endl;)
                return;
            }
            LOG(m_parser->m_pSimulationLog,"---> Selective range startId: " <<
                                  RigidBodyId::getBodyIdString(*m_startRangeIdIt) << std::endl;)
            updateStartRange = true;
        }else{
            // parse all bodies;
            //overwrite range containing all bodies, if we don't parse selective ids, parse all bodies!
            m_bodyIdRangePtr = &m_bodyIdRangeTmp;
            m_bodyIdRangeTmp = std::make_pair(m_startIdGroup,endBodyId+1);
            bodyIdIt = m_bodyIdRangePtr->begin();
            LOG(m_parser->m_pSimulationLog,"---> overwrite selective range... " << std::endl;)
        }

        // Adding bodies in the range =============================
        // iterator bodyRange till the id is > endBodyId or we are out of the bodyIdRange
        auto itEnd = m_bodyIdRangePtr->end();
        m_parsedInstancesGroup = 0;

        m_bodyListGroup.resize(m_bodyIdRangePtr->size());
        auto bodyIt = m_bodyListGroup.begin();
        for( /* nothing*/ ; (bodyIdIt != itEnd) && ( *bodyIdIt <= endBodyId); ++bodyIdIt )
        {
            //LOG(m_parser->m_pSimulationLog,"---> Added RigidBody Instance: "<<RigidBodyId::getBodyIdString(*bodyIdIt)<<std::endl);
            // Push new body
            if(m_allocateBodies){
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

        LOG(m_parser->m_pSimulationLog,"---> Added "<<m_parsedInstancesGroup<<" RigidBody Instances..."<<std::endl;);
        // =======================================================

        // Parse Geometry
        XMLNodeType  geometryNode = rigidbodies.child("Geometry").first_child();
        CHECK_XMLNODE(geometryNode,"Geometry (first node)")
        if(m_pGeomMod){
            m_pGeomMod->parseGeometry(geometryNode, &m_bodyListGroup, m_startIdGroup);
        }

         //Parse DynamicsProperties
        parseDynamicProperties(dynPropNode);

        //Copy the pointers!
        if(m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {
            LOG(m_parser->m_pSimulationLog,"---> Copy Simulated RigidBody References to DynamicSystem ..."<<std::endl;);
            bool added = addAllBodies(m_pSimBodies);
            if(!added) {THROWEXCEPTION("Could not add body to m_SimBodies!, some bodies exist already in map!");};
            m_nSimBodies += m_parsedInstancesGroup;
            m_nBodies += m_parsedInstancesGroup;
        } else if(m_eBodiesState == RigidBodyType::BodyState::STATIC) {
            LOG(m_parser->m_pSimulationLog,"---> Copy Static RigidBody References to DynamicSystem ..."<<std::endl;);
            bool added = addAllBodies(m_pBodies);
            if(!added) {THROWEXCEPTION("Could not add body to m_Bodies!, some bodies exist already in map!");};
            m_nBodies += m_parsedInstancesGroup;
        } else {
            THROWEXCEPTION("---> Adding only simulated and not simulated objects supported!");
        }

//        XMLNodeType  visualizationNode = rigidbodies.child("Visualization");
//        parseVisualization( visualizationNode);
        // ===============================================================================================================


    }

    virtual void parseDynamicProperties( XMLNodeType  dynProp) {
//        LOG(m_parser->m_pSimulationLog,"---> Parse DynamicProperties ..."<<std::endl;);
//
//        // DynamicState has already been parsed for the group!
//
//        // apply first to all bodies :-)
//        for(auto & b: m_bodyListGroup) {
//            b.m_body->m_eState = m_eBodiesState;
//        }
//
//        if(m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {
//            parseDynamicPropertiesSimulated(dynProp);
//        } else if(m_eBodiesState == RigidBodyType::BodyState::STATIC) {
//            parseDynamicPropertiesNotSimulated(dynProp);
//        }
    }
    virtual void parseDynamicState(XMLNodeType  dynProp){

//        ticpp::Element * element = dynProp.child("DynamicState")->ToElement();
//        std::string type = element.attribute("type").value();
//        if(type == "simulated") {
//            m_eBodiesState =  RigidBodyType::BodyState::SIMULATED;
//        } else if(type == "not simulated" || type == "static") {
//            m_eBodiesState =  RigidBodyType::BodyState::STATIC;
//        } else if(type == "animated") {
//            m_eBodiesState =  RigidBodyType::BodyState::ANIMATED;
//            THROWEXCEPTION("---> The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
//        } else {
//            THROWEXCEPTION("---> The attribute 'type' '" + type + std::string("' of 'DynamicState' has no implementation in the parser"));
//        }

    }
    virtual void parseDynamicPropertiesSimulated( XMLNodeType  dynProp) {
//        ticpp::Element *element = nullptr;
//        std::string distribute;
//
//        if(!m_parseOnlyVisualizationProperties) {
//            // First allocate a new SolverDate structure
//            for(auto & b : m_bodyListGroup)  {
//                b.m_body->m_pSolverData = new RigidBodySolverDataType();
//            }
//            // Mass ============================================================
//            element = dynProp.child("Mass")->ToElement();
//            distribute = element.attribute("distribute").value();
//            if(distribute == "uniform") {
//
//                double mass = element->GetAttribute<double>("value");
//
//                for(auto & b : m_bodyListGroup) {
//                    b.m_body->m_mass = mass;
//                }
//
//
//            } else {
//                THROWEXCEPTION("---> The attribute 'distribute' '" + distribute + std::string("' of 'Mass' has no implementation in the parser"));
//            }
//
//            // InertiaTensor ============================================================
//            element = dynProp.child("InertiaTensor")->ToElement();
//            std::string type = element.attribute("type").value();
//            if(type == "homogen") {
//                for(auto & b : m_bodyListGroup) {
//                    InertiaTensor::calculateInertiaTensor(b.m_body);
//                }
//            } else {
//                THROWEXCEPTION("---> The attribute 'type' '" + type + std::string("' of 'InertiaTensor' has no implementation in the parser"));
//            }
//
//            element = dynProp.child("Material")->ToElement();
//            distribute = element.attribute("distribute").value();
//            if(distribute == "uniform") {
//                typename RigidBodyType::BodyMaterialType eMaterial = 0;
//
//                if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(eMaterial, element.attribute("id").value())) {
//                    THROWEXCEPTION("---> String conversion in Material: id failed");
//                }
//
//                for(auto & b : m_bodyListGroup) {
//                    b.m_body->m_eMaterial = eMaterial;
//                }
//            } else {
//                THROWEXCEPTION("---> The attribute 'distribute' '" + distribute + std::string("' of 'Material' has no implementation in the parser"));
//            }
//        }
//
//        // InitialPosition ============================================================
//        XMLNodeType  node = dynProp.child("InitialCondition",true);
//        parseInitialCondition(node);
    }
    virtual void parseDynamicPropertiesNotSimulated( XMLNodeType  dynProp) {

        // InitialPosition ============================================================
//        XMLNodeType  node = dynProp.child("InitialCondition",true);
//        parseInitialCondition(node);
//
//
//        ticpp::Element *element = nullptr;
//        std::string distribute;
//
//        if(!m_parseOnlyVisualizationProperties) {
//            element = dynProp.child("Material")->ToElement();
//            distribute = element.attribute("distribute").value();
//            if(distribute == "uniform") {
//                typename RigidBodyType::BodyMaterialType eMaterial = 0;
//
//                if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(eMaterial, element.attribute("id").value())) {
//                    THROWEXCEPTION("---> String conversion in Material: id failed");
//                }
//
//                for(auto & b : m_bodyListGroup) {
//                    b.m_body->m_eMaterial = eMaterial;
//                }
//            } else {
//                THROWEXCEPTION("---> The attribute 'distribute' '" + distribute + std::string("' of 'Material' has no implementation in the parser"));
//            }
//        }

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
    bool m_allocateBodies;


    /// Other Modules
    GeometryModuleType * m_pGeomMod;
    InitStatesModuleType * m_pInitStatesMod;
    std::unique_ptr<VisModType> m_pVisMod;

    RigidBodySimContainerType * pSimBodies;

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
    using GeometryModuleType    = ParserModules::GeometryModule<SceneParser>;
    using ContactParamModuleType= ParserModules::ContactParamModule<SceneParser>;

    using InitStatesModuleType  = ParserModules::InitStatesModule<SceneParser> ;
    using BodyVisModuleType     = ParserModules::BodyVisModuleDummy<SceneParser>;
    using BodyModuleType        = ParserModules::BodyModule< SceneParser, BodyVisModuleType > ;
    using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<SceneParser>;

    using XMLNodeType = pugi::xml_node;
    using XMLAttributeType = pugi::xml_attribute;


    using RandomGenType = typename DynamicsSystemType::RandomGenType;
    template<typename T>
    using UniformDistType = std::uniform_real_distribution<T>;

protected:

    friend class ParserModules::SettingsModule<SceneParser>;
    friend class ParserModules::GeometryModule<SceneParser>;
    friend class ParserModules::ContactParamModule<SceneParser>;

    friend class ParserModules::InitStatesModule<SceneParser>;
    friend class ParserModules::BodyModule< SceneParser, BodyVisModuleType > ;
    friend class ParserModules::ExternalForcesModule<SceneParser>;

public:

    SceneParser( std::shared_ptr<DynamicsSystemType> pDynSys):m_pDynSys(pDynSys) {
        // Get all Modules from DynamicsSystem
        std::tie(m_pSettingsModule, m_pBodyModule, m_pInitStatesModule, m_pGeometryModule, m_pExternalForcesModule, m_pContactParamModule )
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

    void checkFileExists(boost::filesystem::path file) {
        if( !boost::filesystem::exists(file) ) {
            THROWEXCEPTION("---> The file ' " + file.string() + "' does not exist!");
        }
    }

protected:

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

            node = m_xmlRootNode.child("SceneObjects");
            parseSceneObjects(node);

            node = m_xmlRootNode.child("SceneSettings");
            parseSceneSettingsPost(node);

            //parseOtherOptions(m_xmlRootNode);


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

        if(m_pExternalForcesModule) {
            m_pExternalForcesModule->parse(sceneSettings);
        }

        if(m_pGeometryModule) {
            m_pGeometryModule->parseGlobalGeometries(sceneSettings);
        }
    }

    virtual void parseSceneSettingsPost( XMLNodeType sceneSettings ) {
        if(m_pInitStatesModule) {
            m_pInitStatesModule->parseGlobalInitialCondition(sceneSettings);
        }
    }

    virtual void parseSceneObjects( XMLNodeType sceneObjects) {
        if(m_pBodyModule){
            m_pBodyModule->parse(sceneObjects);
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
    std::unique_ptr< GeometryModuleType>       m_pGeometryModule;
    std::unique_ptr< ExternalForcesModuleType> m_pExternalForcesModule;
    std::unique_ptr< ContactParamModuleType>   m_pContactParamModule;

    std::unique_ptr< BodyModuleType>           m_pBodyModule;
    std::unique_ptr< InitStatesModuleType>     m_pInitStatesModule;

};




#endif

