#ifndef SceneParserModules_hpp
#define SceneParserModules_hpp

#include <vector>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include <memory>

#include <boost/filesystem.hpp>

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>       // Output data structure
#include <assimp/postprocess.h> // Post processing flags

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "AssertionDebug.hpp"

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
#include "XMLMacros.hpp"

#include "SceneParserBaseTraits.hpp"


namespace ParserModules {

class  DummyModule{
public:
    DummyModule(const char * type) {
       ERRORMSG("This is a standart dummy module: " << type << "which does nothing!" <<
                " This function should not be called!");
    };

    void cleanUp() {}

    template<typename... Args>
    void parse(Args&&... args){}
    template<typename... Args>
    void parseOtherOptions(Args&&... args) {}
    template<typename... Args>
    void parseSceneSettingsPost(Args&&... args) {}
    template<typename... Args>
    void parseSceneSettingsPre(Args&&... args) {}
};



/** Parses TimestepperSettings, InclusionSolverSettings, RecorderSettings */
template<typename TParserTraits>
class SettingsModule {
protected:

    DEFINE_PARSER_TYPE_TRAITS(TParserTraits )
    DEFINE_LAYOUT_CONFIG_TYPES

    using RecorderSettingsType        = typename DynamicsSystemType::RecorderSettingsType;
public:
    using TimeStepperSettingsType     = typename DynamicsSystemType::TimeStepperSettingsType;
protected:
    using InclusionSolverSettingsType = typename DynamicsSystemType::InclusionSolverSettingsType;

    RecorderSettingsType        * m_recorderSettings;
    TimeStepperSettingsType     * m_timestepperSettings;
    InclusionSolverSettingsType * m_inclusionSettings;

    LogType * m_pSimulationLog;
    ParserType * m_parser;
public:

    void cleanUp() {}

    SettingsModule(ParserType * p, RecorderSettingsType * r, TimeStepperSettingsType * t, InclusionSolverSettingsType * i)
        :m_parser(p),m_pSimulationLog(p->getSimLog()),m_recorderSettings(r),m_timestepperSettings(t), m_inclusionSettings(i) {};

    void parse(XMLNodeType sceneSettings) {

        LOGSCLEVEL1(m_pSimulationLog, "==== SettingsModule: parsing ======================================"<<std::endl;)

        XMLNodeType node;
        XMLAttributeType att;

        XMLNodeType timestepNode = sceneSettings.child("TimeStepperSettings");

        if( m_timestepperSettings ) {
            LOGSCLEVEL1(m_pSimulationLog,"---> TimeStepperSettings ..." << std::endl;)
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
            SKIPLOGSC(m_pSimulationLog, "---> SettingsModule: skipped TimeStepperSettings ..."<<std::endl;)
        }


        if(m_inclusionSettings) {
            LOGSCLEVEL1(m_pSimulationLog,"---> InclusionSolverSettings ..." << std::endl;)
            auto node = timestepNode.child("InclusionSolverSettings");
            CHECK_XMLNODE(node,"InclusionSolverSettings");

            if(!Utilities::stringToType(m_inclusionSettings->m_alphaJORProx, node.attribute("alphaJORProx").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: alphaJORProx failed");
            }
            if(!Utilities::stringToType(m_inclusionSettings->m_alphaSORProx, node.attribute("alphaSORProx").value())) {
                THROWEXCEPTION("---> String conversion in InclusionSolverSettings: alphaJORProx failed");
            }

            att = node.attribute("matrixRStrategy");
            if(att) {
                std::string method = att.value();
                if(method == "max") {
                    m_inclusionSettings->m_RStrategy = InclusionSolverSettingsType::RSTRATEGY_MAX;
                } else if (method == "sum") {
                    m_inclusionSettings->m_RStrategy = InclusionSolverSettingsType::RSTRATEGY_SUM;
                } else if (method == "sum2") {
                    m_inclusionSettings->m_RStrategy = InclusionSolverSettingsType::RSTRATEGY_SUM2;
                } else {
                    THROWEXCEPTION("---> String conversion in InclusionSolverSettings: matrixRStrategy failed: not a valid setting");
                }
            } else {
                m_inclusionSettings->m_RStrategy = InclusionSolverSettingsType::RSTRATEGY_MAX;
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
                m_inclusionSettings->m_eMethod = InclusionSolverSettingsType::SOR_CONTACT_AC;
            } else if (method == "SORContact") {
                m_inclusionSettings->m_eMethod = InclusionSolverSettingsType::SOR_CONTACT_AC;
            } else if (method == "SORContactAC") {
                m_inclusionSettings->m_eMethod = InclusionSolverSettingsType::SOR_CONTACT_AC;
            } else if (method == "SORContactDS") {
                m_inclusionSettings->m_eMethod = InclusionSolverSettingsType::SOR_CONTACT_DS;
            } else if (method == "SORFull") {
                m_inclusionSettings->m_eMethod = InclusionSolverSettingsType::SOR_FULL;
            } else if (method == "SORNormalTangential") {
                m_inclusionSettings->m_eMethod = InclusionSolverSettingsType::SOR_NORMAL_TANGENTIAL;
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
            SKIPLOGSC(m_pSimulationLog, "---> SettingsModule: skipped InclusionSolverSettings ..."<<std::endl;)
        }


        if(m_recorderSettings) {
            LOGSCLEVEL1(m_pSimulationLog,"---> RecorderSettings ..." << std::endl;)
            node = sceneSettings.child("RecorderSettings");
            CHECK_XMLNODE(node,"RecorderSettings");
            std::string method = node.attribute("mode").value();
            if(method == "everyTimeStep") {
                m_recorderSettings->setMode(RecorderSettings::RECORD_EVERY_STEP);
            } else if (method == "everyXTimeStep") {
                m_recorderSettings->setMode(RecorderSettings::RECORD_EVERY_X_STEP);
                PREC fps;
                if(!Utilities::stringToType(fps, node.attribute("statesPerSecond").value())) {
                    THROWEXCEPTION("---> String conversion in RecorderSettings: statesPerSecond failed");
                }
                m_recorderSettings->setEveryXTimestep(fps,m_timestepperSettings->m_deltaT);
            } else if (method == "noOutput") {
                m_recorderSettings->setMode(RecorderSettings::RECORD_NOTHING);
            } else {
                THROWEXCEPTION("---> String conversion in RecorderSettings: recorderMode failed: not a valid setting");
            }

        } else {
            SKIPLOGSC(m_pSimulationLog, "---> SettingsModule: skipped RecorderSettings ..."<<std::endl;)
        }

        this->parseOtherOptions(sceneSettings);

        LOGSCLEVEL1(m_pSimulationLog, "==================================================================="<<std::endl;)
    }

    virtual void parseOtherOptions(XMLNodeType sceneSettings) {
        // This function does nothing!
    }

    void setTimeStepperSettings(PREC startTime){
        m_timestepperSettings->m_startTime = startTime;
    }
};


template<typename TParserTraits>
class SettingsModuleDummy : public DummyModule{
public:

    using TimeStepperSettingsType = void;

    template<typename... Args>
    SettingsModuleDummy(Args&&... args) : DummyModule("SettingsModule") {}

    template<typename... Args>
    void setTimeStepperSettings(Args&&... args){}
};



template<bool loadMesh = true, bool allocateGeometry = true, bool cacheScale = false, bool cacheGeometry = false>
struct GeometryModuleStaticOptions {
    static const bool m_loadMesh = loadMesh; ///< loads the mesh into the MeshGeometry from the file
    static const bool m_allocateGeometry = allocateGeometry; ///< If false, absolutely no geometries are allocated!
    static const bool m_cacheScale = cacheScale;         /// cache scale of geometry in the parsing group in external list
    static const bool m_cacheGeometry = cacheGeometry;   /// cache geometry in external list
};

template<typename TParserTraits, typename TStaticOptions = GeometryModuleStaticOptions<> >
class GeometryModule {

private:
    DEFINE_PARSER_TYPE_TRAITS(TParserTraits )
    DEFINE_LAYOUT_CONFIG_TYPES

    using GlobalGeometryMapType = typename DynamicsSystemType::GlobalGeometryMapType;
    using RigidBodyType = typename DynamicsSystemType::RigidBodyType;

    LogType * m_pSimulationLog;
    ParserType * m_parser;

    using BodyListType = typename BodyModuleType::BodyListType;

    BodyListType * m_bodiesGroup = nullptr;
    bool m_addToGlobalGeoms = false;
    RigidBodyIdType m_startIdGroup;

    GlobalGeometryMapType * m_globalGeometries;

    std::unordered_map<unsigned int, Vector3> m_scalesGlobal;

    using Options = TStaticOptions;

public:
    using GeometryMap = std::unordered_map<RigidBodyIdType, typename GlobalGeometryMapType::mapped_type>;
    using ScalesList= std::vector<Vector3>;
private:
    GeometryMap * m_externalGeometryCache = nullptr; ///< external list to cache geometry
    ScalesList * m_externalScalesGroupCache = nullptr; ///< external list to cache the scales for the parsed group
public:


    void cleanUp() {
        m_scalesGlobal.clear();
    }

    GeometryModule(ParserType * p,
                   GlobalGeometryMapType * g,
                   ScalesList * scalesGroup = nullptr,
                   GeometryMap * geomMap = nullptr)
        : m_parser(p),m_pSimulationLog(p->getSimLog()),m_globalGeometries(g),
          m_externalScalesGroupCache(scalesGroup), m_externalGeometryCache(geomMap) {
        ASSERTMSG( m_globalGeometries , "GeometryModule needs a globalGeometry pointer!")
    };


    void parseGlobalGeometries(XMLNodeType sceneSettings) {
        LOGSCLEVEL1(m_pSimulationLog, "==== GeometryModule: parsing (GlobalGeometry) ====================="<<std::endl;)

        m_scalesGlobal.clear();

        if(m_globalGeometries) {
            XMLNodeType globalGeom = sceneSettings.child("GlobalGeometries");
            if(globalGeom) {
                for (XMLNodeType n : globalGeom.children()) {
                    parseGeometry_imp(n, nullptr);
                }
            }
        } else {
            SKIPLOGSC(m_pSimulationLog, "---> GeometryModule: skipped GlobalGeometries ..."<<std::endl;)
        }

        LOGSCLEVEL1(m_pSimulationLog, "==================================================================="<<std::endl;)
    }

    void parseGeometry( XMLNodeType geometryNode,  BodyListType * bodyList, RigidBodyIdType startId) {
        LOGSCLEVEL1(m_pSimulationLog, "---> GeometryModule: parsing (BodyGeometry)"<<std::endl;)
        XMLNodeType geom= geometryNode.first_child(); // get the first geometry (sphere, box , mesh ...)
        CHECK_XMLNODE(geom, "(Sphere/Box/Mesh/Halfspace)");
        parseGeometry_imp(geom, bodyList, startId);
    }


private:

    void parseGeometry_imp( XMLNodeType geometryNode,  BodyListType * bodyList=nullptr, RigidBodyIdType startId=0) {
        m_startIdGroup = startId;
        m_bodiesGroup = bodyList;
        m_addToGlobalGeoms = m_bodiesGroup? false : true;

        if(Options::m_cacheGeometry && !m_externalGeometryCache) {
            THROWEXCEPTION("Trying to cache geometries but externalGeometryCache pointer is null!")
        }
        if(Options::m_cacheScale && !m_externalScalesGroupCache) {
            THROWEXCEPTION("Trying to cache scales but externalScaleCache pointer is null!")
        }


        if(Options::m_cacheScale && bodyList) {
            m_externalScalesGroupCache->assign(bodyList->size(), Vector3(1,1,1));
        }

        if(std::strcmp(geometryNode.name() , "Sphere")==0) {
            parseSphereGeometry(geometryNode);

        } else if(std::strcmp(geometryNode.name() , "Halfspace")==0) {
            parseHalfspaceGeometry( geometryNode);

        } else if(std::strcmp(geometryNode.name() , "Mesh")==0) {
            parseMeshGeometry( geometryNode);

        } else if(std::strcmp(geometryNode.name() , "Box")==0) {
            parseBoxGeometry( geometryNode);

        } else if(std::strcmp(geometryNode.name() , "GlobalGeomId")==0 && !m_addToGlobalGeoms) {
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
        LOGSCLEVEL2(m_pSimulationLog,"\t---> Added GlobalGeomId: " << id;);
        if(2<=SCENEPARSER_LOGLEVEL) {
            PrintGeometryDetailsVisitor(m_pSimulationLog, ret.first->second, ", ");
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

            if(m_addToGlobalGeoms && Options::m_allocateGeometry) {

                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,sphere.attribute("id").value())) {
                    THROWEXCEPTION("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    THROWEXCEPTION("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                if(Options::m_cacheScale) {
                    m_scalesGlobal.emplace(id,Vector3(radius,radius,radius));
                }
                addToGlobalGeomList(id,std::shared_ptr<SphereGeometry >(new SphereGeometry(radius)));


            } else {

                unsigned int bodyIdx = 0;
                for(auto & b : *m_bodiesGroup) {
                    LOGSCLEVEL3(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_id) << ", GeometryType: Sphere" << std::endl);
                    LOGSCLEVEL3(m_pSimulationLog, "\t\t---> radius: " << radius << std::endl; );

                    if(Options::m_allocateGeometry) {
                        auto s = std::shared_ptr<SphereGeometry >(new SphereGeometry(radius));
                        if(b.m_body) {
                            b.m_body->m_geometry = s;
                        }
                        if(Options::m_cacheGeometry) {
                            m_externalGeometryCache->emplace(b.m_id,s);
                        }
                    }
                    if(Options::m_cacheScale) {
                        (*m_externalScalesGroupCache)[bodyIdx] = Vector3(radius,radius,radius);
                        ++bodyIdx;
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

            if(m_addToGlobalGeoms && Options::m_allocateGeometry) {

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
                    if(Options::m_cacheScale) {
                        m_scalesGlobal.emplace(i,Vector3(radius,radius,radius));
                    }
                    addToGlobalGeomList(i, std::shared_ptr<SphereGeometry >(new SphereGeometry(radius)));
                }
            } else {


                RigidBodyIdType diffId = m_startIdGroup; // id to generate to correct amount of random values!
                PREC radius = uni(gen); // generate first value

                unsigned int bodyIdx = 0;
                for(auto & b : *m_bodiesGroup) {

                    // Generate the intermediate random values if there are any
                    radius = Utilities::genRandomValues<PREC>(radius, gen,uni,b.m_id-diffId); // (id:16 - id:13 = 3 values, 13 is already generated)
                    diffId = b.m_id; // update current diffId;



                    LOGSCLEVEL3(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_id)<< ", GeometryType: Sphere" << std::endl);
                    LOGSCLEVEL3(m_pSimulationLog, "\t\t---> radius: " << radius << std::endl; );

                    if(Options::m_allocateGeometry) {
                        auto s = std::shared_ptr<SphereGeometry >(new SphereGeometry(radius));
                        if(b.m_body) {
                            b.m_body->m_geometry = s;
                        }
                        if(Options::m_cacheGeometry) {
                            m_externalGeometryCache->emplace(b.m_id,s);
                        }
                    }
                    if(Options::m_cacheScale) {
                        (*m_externalScalesGroupCache)[bodyIdx] = Vector3(radius,radius,radius);
                        ++bodyIdx;
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


            if(m_addToGlobalGeoms && Options::m_allocateGeometry) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,halfspace.attribute("id").value())) {
                    THROWEXCEPTION("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    THROWEXCEPTION("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                //no scale (here)
                addToGlobalGeomList(id, std::shared_ptr<HalfspaceGeometry >(new HalfspaceGeometry(n,p)));
            } else {

                if(Options::m_allocateGeometry) {
                    auto s = std::shared_ptr<HalfspaceGeometry >(new HalfspaceGeometry(n,p));
                    for(auto & b : *m_bodiesGroup) {
                        // no scale (here)
                        if(b.m_body) {
                            b.m_body->m_geometry = s;
                        }
                        if(Options::m_cacheGeometry) {
                            m_externalGeometryCache->emplace(b.m_id,s);
                        }
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

            if(m_addToGlobalGeoms && Options::m_allocateGeometry) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,box.attribute("id").value())) {
                    THROWEXCEPTION("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    THROWEXCEPTION("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                if(Options::m_cacheScale) {
                    m_scalesGlobal.emplace(id,Vector3(extent(0),extent(1),extent(2)));
                }
                addToGlobalGeomList(id, std::shared_ptr<BoxGeometry >(new BoxGeometry(center,extent)));
            } else {

                unsigned int bodyIdx = 0;
                for(auto & b  : *m_bodiesGroup) {

                    if(Options::m_allocateGeometry) {
                        auto s = std::shared_ptr<BoxGeometry >(new BoxGeometry(center,extent));
                        if(b.m_body) {
                            b.m_body->m_geometry = s;
                        }
                        if(Options::m_cacheGeometry) {
                            m_externalGeometryCache->emplace(b.m_id,s);
                        }
                    }

                    if(Options::m_cacheScale) {
                        (*m_externalScalesGroupCache)[bodyIdx] = Vector3(extent(0),extent(1),extent(2));
                        ++bodyIdx;
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

        std::string type = mesh.attribute("distribute").value();
        if(type == "uniform") {

            // import here an object model;
            // First make an meshinformatino structure

            boost::filesystem::path fileName =  mesh.attribute("file").value();
            m_parser->checkFileExists(fileName);

            MeshData * meshData =  nullptr;

            Vector3 scale_factor;
            if(!Utilities::stringToVector3(scale_factor, mesh.attribute("scale").value())) {
                THROWEXCEPTION("---> String conversion in parseMeshGeometry failed: scale");
            }
            if(scale_factor.norm()==0) {
                THROWEXCEPTION("---> Wrong scale factor (=0) specified in parseMeshGeometry!");
            }

            if(Options::m_loadMesh) {
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
                QuaternionHelpers::setQuaternion(quat,axis,angle);


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

                meshData = new MeshData();

                if(!meshData->setup(importer,scene, scale_factor,quat,trans)) {
                    THROWEXCEPTION("---> Imported Mesh (with Assimp) could not be setup internally");
                }

                if(mesh.attribute("writeToLog")) {
                    bool writeToLog;
                    if(!Utilities::stringToType(writeToLog, mesh.attribute("writeToLog").value())) {
                        THROWEXCEPTION("---> String conversion in parseMeshGeometry: angleDegree failed");
                    }
                    if(writeToLog) {
                        meshData->writeToLog(fileName.string(), m_pSimulationLog);
                    }
                }
            }

            // Assign Geometry
            if(m_addToGlobalGeoms && Options::m_allocateGeometry) {
                unsigned int id;
                if(!Utilities::stringToType<unsigned int>(id,mesh.attribute("id").value())) {
                    THROWEXCEPTION("---> String conversion in addToGlobalGeomList: id failed");
                }
                if(id == 0) {
                    THROWEXCEPTION("---> addToGlobalGeomList: a global geometry id: 0 is not allowed!");
                    // 0 wird verwendet als m_globalGeomId in RigidBody um zu spezifizieren, dass der Body seine eigene Geom hat
                }
                if(Options::m_cacheScale) {
                    m_scalesGlobal.emplace(id,scale_factor);
                }
                addToGlobalGeomList(id, std::shared_ptr<MeshGeometry >(new MeshGeometry(fileName,meshData)));
            } else {

                unsigned int bodyIdx = 0;
                for(auto & b  : *m_bodiesGroup) {

                    if(Options::m_allocateGeometry) {
                        auto s = std::shared_ptr<MeshGeometry >(new MeshGeometry(fileName,meshData));;
                        if(b.m_body) {
                            b.m_body->m_geometry = s;
                        }
                        if(Options::m_cacheGeometry) {
                            m_externalGeometryCache->emplace(b.m_id,s);
                        }
                    }

                    if(Options::m_cacheScale) {
                        (*m_externalScalesGroupCache)[bodyIdx] = scale_factor ;
                        ++bodyIdx;
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

            typename GlobalGeometryMapType::iterator it;
            if(Options::m_allocateGeometry) {
                it = m_globalGeometries->find(id);
                // it->second is the GeometryType in RigidBody
                if(it == m_globalGeometries->end()) {
                    THROWEXCEPTION("---> Geometry search in parseGlobalGeomId: failed for id: " << id << std::endl);
                }
            }

            bool addScale = true;
            auto globalScaleIt = m_scalesGlobal.find(id);
            if(globalScaleIt == m_scalesGlobal.end()) {
                addScale = false;
            }

            unsigned int bodyIdx = 0;
            for(auto & b : *m_bodiesGroup) {

                if(Options::m_allocateGeometry) {
                    if(b.m_body) {
                        b.m_body->m_geometry = it->second;
                        b.m_body->m_globalGeomId = id;
                        LOGSCLEVEL3(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_body) << ", GlobalGeomId: " << id <<  std::endl);
                    }
                    if(Options::m_cacheGeometry) {
                        m_externalGeometryCache->emplace(b.m_id,it->second);
                    }
                }

                if(addScale && Options::m_cacheScale) {
                    (*m_externalScalesGroupCache)[bodyIdx] = globalScaleIt->second ;
                    ++bodyIdx;
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

            unsigned int bodyIdx = 0;
            for(auto & b : *m_bodiesGroup) {
                unsigned int id = startId + (b.m_id - m_startIdGroup); //make linear offset from start of this group

                if(Options::m_allocateGeometry) {
                    auto it = m_globalGeometries->find(id);
                    // it->second is the GeometryType in RigidBody
                    if(it == m_globalGeometries->end()) {
                        THROWEXCEPTION("---> parseGlobalGeomId: Geometry search failed for id: " << id << std::endl);
                    }
                    if(b.m_body) {
                        b.m_body->m_geometry = it->second;
                        b.m_body->m_globalGeomId = id;
                        LOGSCLEVEL3(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_body) << ", GlobalGeomId: " << id <<  std::endl);
                    }
                    if(Options::m_cacheGeometry) {
                        m_externalGeometryCache->emplace(b.m_id,it->second);
                    }
                }

                if(Options::m_cacheScale) {
                    auto globalScaleIt = m_scalesGlobal.find(id); // some geometries might not have a scale , e.g. halfspace
                    if(globalScaleIt != m_scalesGlobal.end()) {
                        (*m_externalScalesGroupCache)[bodyIdx] = globalScaleIt->second;
                    }
                    ++bodyIdx;
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

            unsigned int bodyIdx = 0;
            for(auto & b: *m_bodiesGroup) {
                //std::cout << b.m_id - diffId << std::endl;
                id = Utilities::genRandomValues<unsigned int>(id,gen,uni,b.m_id - diffId);
                diffId = b.m_id;

                if(Options::m_allocateGeometry) {
                    auto it = m_globalGeometries->find(id);
                    // it->second is the GeometryType in RigidBody
                    if(it == m_globalGeometries->end()) {
                        THROWEXCEPTION("---> Geometry search in parseGlobalGeomId: failed for id: " << id << std::endl);
                    }

                    if(b.m_body) {
                        b.m_body->m_geometry = it->second;
                        b.m_body->m_globalGeomId = id;
                        LOGSCLEVEL3(m_pSimulationLog, "\t---> Body id:" << RigidBodyId::getBodyIdString(b.m_body) << ", GlobalGeomId: " << id <<  std::endl);
                    }
                    if(Options::m_cacheGeometry) {
                        m_externalGeometryCache->emplace(b.m_id,it->second);
                    }

                    if(Options::m_cacheScale) {
                        auto globalScaleIt = m_scalesGlobal.find(id); // some geometries might not have a scale , e.g. halfspace
                        if(globalScaleIt != m_scalesGlobal.end()) {
                            (*m_externalScalesGroupCache)[bodyIdx] = globalScaleIt->second;
                        }
                        ++bodyIdx;
                    }

                }
            }


        } else {
            THROWEXCEPTION("---> The attribute 'distribute' '" + distribute + std::string("' of 'GlobalGeomId' has no implementation in the parser"));
        }



    }
    ///  ================================================================================ Geometries


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


};


template<typename TParserTraits>
class GeometryModuleDummy : public DummyModule{
public:
    template<typename... Args>
    GeometryModuleDummy(Args&&... args) : DummyModule("GeometryModule") {}

    template<typename... Args>
    void parseGlobalGeometries(Args&&... args){}
    template<typename... Args>
    void parseGeometry(Args&&... args){}
};


template<typename TParserTraits>
class ContactParamModule {
private:

    DEFINE_PARSER_TYPE_TRAITS(TParserTraits )
    DEFINE_LAYOUT_CONFIG_TYPES

    using ContactParameterMapType = typename DynamicsSystemType::ContactParameterMapType;
    ContactParameterMapType * m_contactParams;

    using RigidBodyType = typename DynamicsSystemType::RigidBodyType;

    LogType * m_pSimulationLog;

public:
    void cleanUp() {}

    ContactParamModule(ParserType * p, ContactParameterMapType * c): m_pSimulationLog(p->getSimLog()), m_contactParams(c) {};

    void parse(XMLNodeType sceneSettings) {

        LOGSCLEVEL1(m_pSimulationLog, "---> ContactParamModule: parsing =================================="<<std::endl;)

        if( !m_contactParams ) {
            SKIPLOGSC(m_pSimulationLog, "---> ContactParamModule: skipping ..."<<std::endl;)
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

        LOGSCLEVEL1(m_pSimulationLog, "==================================================================="<<std::endl;)
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
            } else if(type == "UCF") {
                contactParameter = ContactParameter::createParams_UCF_ContactModel(epsilonN,epsilonT,mu);
            }

            if(stdMaterial) {
                LOGSCLEVEL2(m_pSimulationLog,"---> Add ContactParameter standart"<<std::endl;);
                m_contactParams->setStandardValues(contactParameter);
            } else {
                LOGSCLEVEL2(m_pSimulationLog,"---> Add ContactParameter standart of id="<<material1<<" to id="<<material2<<std::endl;);
                if(!m_contactParams->addContactParameter(material1,material2,contactParameter)) {
                    THROWEXCEPTION("---> Add ContactParameter failed");
                }
            }


        } else {
            THROWEXCEPTION("---> String conversion in ContactParameter: type failed");
        }
    }

};

template<typename TParserTraits>
class ContactParamModuleDummy : public DummyModule{
public:
    template<typename... Args>
    ContactParamModuleDummy(Args&&... args) : DummyModule("ContactParamModule") {}
};


template<typename TParserTraits>
class ExternalForcesModule {
private:

    DEFINE_PARSER_TYPE_TRAITS(TParserTraits )
    DEFINE_LAYOUT_CONFIG_TYPES

    using ExternalForceListType = typename DynamicsSystemType::ExternalForceListType;
    ExternalForceListType * m_forceList;

    using RigidBodyType = typename DynamicsSystemType::RigidBodyType;

    LogType * m_pSimulationLog;


public:
    void cleanUp() {}

    ExternalForcesModule(ParserType * p, ExternalForceListType * f): m_pSimulationLog(p->getSimLog()), m_forceList(f) {};

    void parse(XMLNodeType sceneSettings) {
        LOGSCLEVEL1(m_pSimulationLog, "==== ExternalForcesModule: parsing ================================"<<std::endl;)

        XMLNodeType externalForces = sceneSettings.child("ExternalForces");
        if(externalForces) {
            for (XMLNodeType n : externalForces.children()) {
                if( std::string(n.name()) == "ForceField") {
                    parseForceField( n );
                }
            }
        } else {
            SKIPLOGSC(m_pSimulationLog, "---> ExternalForcesModule: skipping ..."<<std::endl;)
        }
        LOGSCLEVEL1(m_pSimulationLog, "==================================================================="<<std::endl;)
    }

private:

    void parseForceField( XMLNodeType forceField) {

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
            LOGSCLEVEL2(m_pSimulationLog,"---> added SpatialSphericalTimeRandomForceField ..."<<std::endl;);

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

            LOGSCLEVEL2(m_pSimulationLog,"---> added GravityForceField ..."<<std::endl;);
        } else {
            THROWEXCEPTION("---> String conversion in parseForceField: type failed");
        }
    }
};

template<typename TParserTraits>
class ExternalForcesModuleDummy : public DummyModule{
public:
    template<typename... Args>
    ExternalForcesModuleDummy(Args&&... args) : DummyModule("ExternalForcesModule") {}
};


template<typename TParserTraits>
class VisModuleDummy : public DummyModule {
public:
    template<typename... Args>
    VisModuleDummy(Args&&... args) : DummyModule("VisModule") {};

    template<typename... Args>
    void parseSceneSettingsPost(Args&&... args) {}
};



template<typename TParserTraits>
class InitStatesModule {
private:
    DEFINE_PARSER_TYPE_TRAITS(TParserTraits )
    DEFINE_LAYOUT_CONFIG_TYPES

    using RigidBodyType = typename DynamicsSystemType::RigidBodyType;
    using RigidBodyStatesContainerType = typename DynamicsSystemType::RigidBodyStatesContainerType;
    RigidBodyStatesContainerType * m_initStates;

    SettingsModuleType * m_settings;
    LogType * m_pSimulationLog;

    using BodyListType = typename BodyModuleType::BodyListType;
    BodyListType * m_bodiesGroup;
    RigidBodyIdType m_startIdGroup;

public:
    using StatesGroupType = std::vector<RigidBodyState> ;
private:
    StatesGroupType m_statesGroup; ///< the internal states for the group!

public:

    void cleanUp() {}

    InitStatesModule(ParserType * p, RigidBodyStatesContainerType * c, SettingsModuleType * s): m_pSimulationLog(p->getSimLog()),m_initStates(c), m_settings(s) {
        ASSERTMSG(m_initStates, "should not be null");
    };


    void parseGlobalInitialCondition( XMLNodeType sceneSettings) {
        LOGSCLEVEL1(m_pSimulationLog, "---> InitStatesModule: parsing (GlobalInitCondition) =============="<<std::endl;)
        XMLNodeType initCond = sceneSettings.child("GlobalInitialCondition");
        if(initCond) {
            double time = -1;
            short which = 2;
            std::string str = initCond.attribute("whichState").value();
            if( str == "end") {
                which = 2;
            } else if(str == "beg") {
                which = 0;
            } else if(str == "time") {
                which = 1;
                if(!Utilities::stringToType(time, initCond.attribute("time").value())) {
                    THROWEXCEPTION("---> String conversion in GlobalInitialCondition: time failed");
                }
            } else {
                THROWEXCEPTION("---> String conversion in GlobalInitialCondition: whichState failed");
            }

            boost::filesystem::path relpath = initCond.attribute("file").value();


            setupInitialConditionBodiesFromFile_imp(relpath, time, which);


            bool useTime = false;
            if(!Utilities::stringToType(useTime, initCond.attribute("useTimeToContinue").value())) {
                THROWEXCEPTION("---> String conversion in GlobalInitialCondition: useTimeToContinue failed");
            }

            // Set the time in the dynamics system timestepper settings
            if(useTime && m_settings) {
                m_settings->setTimeStepperSettings(time);
            }
        }
        LOGSCLEVEL1(m_pSimulationLog, "==================================================================="<<std::endl;)
    }

    void parseInitialCondition(XMLNodeType initCondNode, BodyListType * bodyList,
                               RigidBodyIdType startId, bool parseVelocity = true , bool addToInitList = true) {

        LOGSCLEVEL1(m_pSimulationLog, "---> InitStatesModule: parsing (BodyInitState)"<<std::endl;)
        ASSERTMSG(bodyList, "Should not be null!")

        m_bodiesGroup = bodyList;
        m_startIdGroup = startId;

        // make state list as big as body list and reset to zero
        m_statesGroup.clear();
        m_statesGroup.resize(m_bodiesGroup->size());
        // set all ids for the states:
        for(unsigned int i = 0 ; i < m_bodiesGroup->size(); i++){
            m_statesGroup[i].m_id = (*m_bodiesGroup)[i].m_id;
        }

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
        auto itState = m_statesGroup.begin();
        for(auto & b: *m_bodiesGroup) {
            LOGSCLEVEL3(m_pSimulationLog, "\t---> InitState id: " << itState->m_id <<": "<< itState->m_q.transpose() << " , " <<  itState->m_u.transpose()  << std::endl;)
            if(b.m_body) {
                LOGSCLEVEL3(m_pSimulationLog, "\t---> apply to body" << std::endl;)
                b.m_body->template applyBodyState<true>( *itState);
            }
            if(addToInitList) {
                added &= m_initStates->emplace(b.m_id, *itState).second;
            }
            ++itState;
        }
        if(!added && addToInitList) {
            THROWEXCEPTION("Could not add init state to m_initStates!, some bodies exist already in map!");
        };

        //LOGSCLEVEL1(m_pSimulationLog, "==================================================================="<<std::endl;)
    }


    StatesGroupType * getStatesGroup() {
        return &m_statesGroup;
    };

private:
    void setupInitialConditionBodiesFromFile_imp(boost::filesystem::path relpath, double &time , short which ) {

        InitialConditionBodies::setupInitialConditionBodiesFromFile(relpath,*m_initStates,time,true,true,which);
        LOGSCLEVEL2(m_pSimulationLog,"---> Found time: "<< time << " in " << relpath << std::endl;);

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

        InitialConditionBodies::setupPositionBodiesLinear(*m_bodiesGroup, m_statesGroup,m_startIdGroup,pos,dir,dist,jitter,delta,seed);

    }
    void parseInitialPositionGrid(XMLNodeType initCond) {

        Vector3 trans;
        Vector3 dirZ(0,0,1);
        Vector3 dirX(1,0,0);
        if(!Utilities::stringToVector3(trans, initCond.attribute("translation").value())) {
            THROWEXCEPTION("---> String conversion in InitialPositionGrid: translation failed");
        }

        if(initCond.attribute("dirZ")) {
            if(!Utilities::stringToVector3(dirZ, initCond.attribute("dirZ").value())) {
                THROWEXCEPTION("---> String conversion in InitialPositionGrid: jitter seed failed");
            }
        }
        if(initCond.attribute("dirX")) {
            if(!Utilities::stringToVector3(dirX, initCond.attribute("dirX").value())) {
                THROWEXCEPTION("---> String conversion in InitialPositionGrid: jitter seed failed");
            }
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

        InitialConditionBodies::setupPositionBodiesGrid(*m_bodiesGroup, m_statesGroup, m_startIdGroup,gridX,gridY,dist,trans,jitter,delta, seed, dirZ,dirX);
    }
    void parseInitialPositionFile(DynamicsState & state, XMLNodeType initCond) {
//        m_SimBodyInitStates.push_back(DynamicsState((unsigned int)m_bodiesGroup->size()));
//
//        boost::filesystem::path name =  initCond->GetAttribute<std::string>("relpath");
//
//        boost::filesystem::path filePath = m_currentParseFileDir / name;
//        InitialConditionBodies::setupPositionBodiesFromFile(state,filePath);
        THROWEXCEPTION("Not implemented")
    }
    void parseInitialPositionTransforms(XMLNodeType initCond) {

        unsigned int nodeCounter = 0;

        auto bodyIt = m_bodiesGroup->begin();
        auto itEnd = m_bodiesGroup->end();
        ASSERTMSG(bodyIt != itEnd, "no bodies in list");

        Quaternion q_KI, q_BK;
        Vector3 I_r_IK;

        auto nodes = initCond.children("Pos");
        auto itNodeEnd = nodes.end();
        auto stateIt = m_statesGroup.begin();
        for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {

            if(bodyIt==itEnd) {
                LOGSCLEVEL2(m_pSimulationLog,"---> InitialPositionTransforms: You specified to many transforms, -> neglecting ..."<<std::endl;);
                break;
            } else if(nodeCounter != bodyIt->m_id - m_startIdGroup) {
                // this value does not correspond to the linear offset from the startId, skip this node
                nodeCounter++; // next node
                continue;
            }

            QuaternionHelpers::setQuaternionZero(q_KI);
            I_r_IK.setZero();

            // Iterate over all transforms an successfully applying the total trasnformation!
            Vector3 trans;
            Vector3 axis;
            PREC angle;
            for ( XMLNodeType & transf : itNode->children("Trafo")) {


                if(!Utilities::stringToVector3(trans, transf.attribute("trans").value())) {
                    THROWEXCEPTION("---> String conversion in InitialPositionTransforms: translation failed");
                }

                if(!Utilities::stringToVector3(axis, transf.attribute("axis").value())) {
                    THROWEXCEPTION("---> String conversion in InitialPositionTransforms: rotationAxis failed");
                }

                if( axis.norm() == 0) {
                    THROWEXCEPTION("---> Specified wrong axis in InitialPositionTransforms");
                }

                auto att = transf.attribute("deg");
                if(att) {
                    if(!Utilities::stringToType(angle, att.value())) {
                        THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: rad failed");
                    }
                    angle = angle / 180 * M_PI;
                } else {
                    att = transf.attribute("rad");
                    if(att) {
                        if(!Utilities::stringToType(angle, att.value())) {
                            THROWEXCEPTION("---> String conversion in InitialPositionPosAxisAngle: deg failed");
                        }
                    } else {
                        THROWEXCEPTION("---> No angle found in InitialPositionPosAxisAngle");
                    }
                }

                QuaternionHelpers::setQuaternion(q_BK,axis,angle);
                QuaternionHelpers::rotateVector(q_KI, trans ); //K_r_KB = trans;
                I_r_IK +=  trans;  // + Rot_KI * K_r_KB; // Transforms like A_IK * K_r_KB;

                q_KI = QuaternionHelpers::quatMult(q_KI,q_BK);
                // Sequential (aktiv) rotation ( A_AB * B_R_2 * A_BA * A_R_1 ) *A_x
                // is the same like: A_R_1 * B_R_2 (see documentation page)

            }

            // Apply overall transformation!
            stateIt->m_q.template head<3>() = I_r_IK;
            stateIt->m_q.template tail<4>() = q_KI;

            ++bodyIt;
            ++stateIt; // next body, next state
            ++nodeCounter; // next node
        }

        // apply over remaining body, but only if we parsed some states for bodies)
        if(bodyIt != itEnd && bodyIt != m_bodiesGroup->begin()) {
            LOGSCLEVEL2(m_pSimulationLog,"---> InitialPositionTransforms: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
            for(; bodyIt != itEnd; ++bodyIt) {
                stateIt->m_q.template head<3>() = I_r_IK;
                stateIt->m_q.template tail<4>() = q_KI;
                ++stateIt;
            }
        }

    }
    void parseInitialVelocityTransRot(XMLNodeType initCond) {
        unsigned int nodeCounter = 0;
        Vector3 transDir,rotDir;
        PREC rot,vel;

        auto bodyIt = m_bodiesGroup->begin();
        auto itEnd = m_bodiesGroup->end();
        ASSERTMSG(bodyIt != itEnd, "no bodies in list");

        // Iterate over all values in the list
        auto nodes = initCond.children("Vel");
        auto itNodeEnd = nodes.end();
        auto stateIt = m_statesGroup.begin();
        for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {

            if(bodyIt==itEnd) {
                LOGSCLEVEL2(m_pSimulationLog,"---> InitialVelocityTransRot: You specified to many velocities (size: " <<nodeCounter <<"), -> neglecting ..."<<std::endl;);
                break;
            } else if(nodeCounter != bodyIt->m_id - m_startIdGroup) {
                // this value does not correspond to the linear offset from the startId, skip this node
                nodeCounter++; // next node
                continue;
            }

            if(!Utilities::stringToVector3(transDir, itNode->attribute("transDir").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: trans failed");
            }
            transDir.normalize();

            if(!Utilities::stringToType(vel, itNode->attribute("absTransVel").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: absTransVel failed");
            }

            if(!Utilities::stringToVector3(rotDir, itNode->attribute("rotDir").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: transDir failed");
            }
            rotDir.normalize();

            if(!Utilities::stringToType(rot, itNode->attribute("absRotVel").value())) {
                THROWEXCEPTION("---> String conversion in InitialVelocityTransRot: absTransVel failed");
            }


            stateIt->m_u.template head<3>() = transDir*vel;
            stateIt->m_u.template tail<3>() = rotDir*rot;

            ++bodyIt;
            ++stateIt; // next body, next state
            ++nodeCounter; // next node

        }

        // apply over remaining body, but only if we parsed some states for bodies)
        if(bodyIt != itEnd && bodyIt != m_bodiesGroup->begin()) {
            LOGSCLEVEL2(m_pSimulationLog,"---> InitialPositionTransforms: You specified to little transforms, -> applying last to all remainig bodies ..."<<std::endl;);
            for(; bodyIt != itEnd; ++bodyIt) {
                stateIt->m_u.template head<3>() = transDir*vel;
                stateIt->m_u.template tail<3>() = rotDir*rot;
                ++stateIt;
            }
        }

    }

};

template<typename TParserTraits>
class InitStatesModuleDummy : public DummyModule{
public:
    template<typename... Args>
    InitStatesModuleDummy(Args&&... args) : DummyModule("InitStatesModule") {}

    template<typename... Args>
    void parseGlobalInitialCondition(Args&&... args){}
    template<typename... Args>
    void parseInitialCondition(Args&&... args){}
};



struct BodyModuleDynamicOptions {
    BodyModuleDynamicOptions() = default;
    BodyModuleDynamicOptions(const BodyModuleDynamicOptions& o) = default;
    BodyModuleDynamicOptions(BodyModuleDynamicOptions&& o) = default;
    BodyModuleDynamicOptions& operator=(const BodyModuleDynamicOptions& o) = default;
    BodyModuleDynamicOptions& operator=(BodyModuleDynamicOptions&& o) = default;

    using BodyRangeType = Range<RigidBodyIdType>;
    BodyRangeType m_bodyIdRange;          ///< Range of body ids, original list which is handed to parseScene, this range is only applied to enableSelectiveIds="true"

};
template<  bool parseAllIfRangeEmpty = true,
           bool parseAllBodiesNonSelGroup = true ,
           bool parseSimBodies = true,
           bool parseStaticBodies = true ,
           bool allocateSimBodies = true,
           bool allocateStaticBodies = true,
           bool parseInitialCondition = true>
struct BodyModuleStaticOptions {
    static const bool m_parseAllIfRangeEmpty = parseAllIfRangeEmpty;

    static const bool m_parseAllBodiesNonSelGroup = parseAllBodiesNonSelGroup;  ///< Parse all bodies in groups where m_bodyIdRange is not applied (enableSelectiveIds="false") (default= true)

    static const bool m_parseSimBodies = parseSimBodies;                        ///< Parses only simulated bodies (default= false)
    static const bool m_parseStaticBodies = parseStaticBodies;

    static const bool m_allocateSimBodies = allocateSimBodies;                  ///< if false, does only parse the nodes which do not need the body -> initial condition
    static const bool m_allocateStaticBodies = allocateStaticBodies;            ///< if false, does only parse the nodes which do not need the body -> initial condition

    static const bool m_parseInitialCondition = parseInitialCondition;          ///< if false, the initial conditions are not parsed!
};


template<typename TParserTraits, typename TStaticOptions = BodyModuleStaticOptions<> >
class BodyModule {
private:

    DEFINE_PARSER_TYPE_TRAITS(TParserTraits )
    DEFINE_LAYOUT_CONFIG_TYPES

    using RigidBodyType = typename DynamicsSystemType::RigidBodyType;
    using RigidBodySimContainerType = typename DynamicsSystemType::RigidBodySimContainerType ;
    using RigidBodyStaticContainerType = typename DynamicsSystemType::RigidBodyStaticContainerType ;

    RigidBodySimContainerType    * m_pSimBodies = nullptr;
    RigidBodyStaticContainerType * m_pBodies    = nullptr;

    LogType * m_pSimulationLog;

    using Options = TStaticOptions;

public:
    using DynamicOptionsType = BodyModuleDynamicOptions;
private:

    DynamicOptionsType m_opts;
    using BodyRangeType = typename DynamicOptionsType::BodyRangeType;
    bool m_parseSelectiveBodyIds;      ///< Use the m_bodyIdRange to only load the selective ids in the group which use enableSelectiveIds="true"

    /// Parsing helpers
    BodyRangeType m_bodyIdRangeTmp;                     ///< Range of bodies, temporary for load of all bodies
    typename BodyRangeType::iterator m_startRangeIdIt;  ///< Current iterator which marks the start for the current group in m_bodyIdRange
    BodyRangeType * m_bodyIdRangePtr;                   ///< Switches between m_bodyIdRangeTmp/m_bodyIdRange, depending on parsing state

    unsigned int m_parsedInstancesGroup;            ///< Number of instances generated in the current group
    RigidBodyIdType m_startIdGroup;                  ///< Start id of the current group smaller or equal to *m_startRangeIdIt


    // Parsed counts of bodies
    unsigned int m_nSimBodies;
    unsigned int m_nBodies;
    unsigned int m_nStaticBodies;
    // Spcified amount of bodies in the Scene File
    unsigned int m_nSpecifiedSimBodies;

    using GroupToNBodyType = std::unordered_map<unsigned int,unsigned int>;
    GroupToNBodyType m_groupIdToNBodies;
    unsigned int m_globalMaxGroupId; // Group Id used to build a unique id!

    /// Temprary structures for each sublist (groupid=?) of rigid bodies
    typename RigidBodyType::BodyMode m_eBodiesState;     ///< Used to parse a RigidBody Node

    struct BodyData {
        BodyData(): m_body(nullptr), m_id(0) {}
        BodyData( RigidBodyType * p, const RigidBodyIdType & id) : m_body(p),m_id(id) {}
        RigidBodyIdType m_id;
        RigidBodyType* m_body; // might be also zero (if we dont need the whole body for visualization only)
    };

public:
    using BodyListType   = std::vector<BodyData>;
private:

    BodyListType   m_bodiesGroup; ///< Used to parse a RigidBody Node

    /// Other Modules
    GeometryModuleType * m_pGeomMod;
    InitStatesModuleType * m_pInitStatesMod;
    VisModuleType * m_pVisMod;

    RigidBodySimContainerType * pSimBodies;

public:

    void cleanUp() {
        m_groupIdToNBodies.clear();
        m_bodiesGroup.clear();
    }

    BodyModule(ParserType * p, GeometryModuleType * g,  InitStatesModuleType * is, VisModuleType * i,
               RigidBodySimContainerType * simBodies, RigidBodyStaticContainerType * bodies )
        : m_pSimulationLog(p->getSimLog()), m_pGeomMod(g), m_pVisMod(i), m_pInitStatesMod(is), m_pSimBodies(simBodies), m_pBodies(bodies) {
    };

    void parseModuleOptions(XMLNodeType & sceneObjects) {
        LOGSCLEVEL1(m_pSimulationLog, "==== BodyModule: parsing (ModuleOptions) ==========================="<<std::endl;)

        if( !m_opts.m_bodyIdRange.empty()) {
            LOGSCLEVEL1(m_pSimulationLog, "---> skipping because bodyIdRange is already set!" <<std::endl;)
            return;
        }

        XMLNodeType selectIds = sceneObjects.child("GlobalSelectiveIds");
        if(selectIds) {
            XMLNodeType n = selectIds.child("Set");
            if(n) {
                using SetType = std::set<RigidBodyIdType>;
                SetType s;
                using CSPBS = Utilities::CommaSeperatedPairBinShift<RigidBodyIdType,RigidBodyIdHalfType>;

                if( !Utilities::stringToType<SetType,CSPBS>(s, n.attribute("value").value())  ) {
                    THROWEXCEPTION("---> String conversion in parseModuleOptions: Set: value failed");
                }
                // Overwrite
                m_opts.m_bodyIdRange = s;


                LOGSCLEVEL2(m_pSimulationLog, "---> Overwriten SelectiveIdRange with Set: [")
                for(auto & id : s) {
                    LOGSCLEVEL2(m_pSimulationLog, RigidBodyId::getBodyIdString(id) << ",")
                }
                LOGSCLEVEL2(m_pSimulationLog, " linear: " << m_opts.m_bodyIdRange.isLinear() <<" ]")
            } else {
                n = selectIds.child("Range");
                using SetType = std::pair<RigidBodyIdType,RigidBodyIdType>;
                SetType r;
                using CSPBS = Utilities::CommaSeperatedPairBinShift<RigidBodyIdType,RigidBodyIdHalfType>;
                if( !Utilities::stringToType<SetType,CSPBS>(r, n.attribute("value").value())  ) {
                    THROWEXCEPTION("---> String conversion in parseModuleOptions: Set: value failed");
                }
                // Overwrite
                m_opts.m_bodyIdRange = r;
                LOGSCLEVEL2(m_pSimulationLog, "---> Overwrite SelectiveIdRange with Range: [" << RigidBodyId::getBodyIdString(r.first)
                            <<", " << RigidBodyId::getBodyIdString(r.second) <<", linear: " << m_opts.m_bodyIdRange.isLinear() <<"]"<<std::endl;)

            }
        }

        if(m_opts.m_bodyIdRange.size() ) {
            m_parseSelectiveBodyIds = true;
        }


        LOGSCLEVEL1(m_pSimulationLog, "==================================================================="<<std::endl;)
    }



    void parse(XMLNodeType & sceneObjects) {
        LOGSCLEVEL1(m_pSimulationLog, "==== BodyModule: parsing (SceneObjects) ============================"<<std::endl;)


        LOGSCLEVEL1( m_pSimulationLog, "---> BodyModule Options: " <<std::endl
                     <<"\t parse simulated bodies:"<<(bool)Options::m_parseSimBodies << std::endl
                     <<"\t parse static bodies:"<<(bool)Options::m_parseStaticBodies << std::endl
                     <<"\t parse all bodies in group with disabled selective ids:"<<(bool)Options::m_parseAllBodiesNonSelGroup << std::endl
                     <<"\t parse selective ids: "<< m_parseSelectiveBodyIds << std::endl
                     <<"\t allocate simulated bodies: "<< (bool)Options::m_allocateSimBodies << std::endl
                     <<"\t allocate static bodies: "<< (bool)Options::m_allocateStaticBodies << std::endl;)
        // Init startRangeIterator
        m_startRangeIdIt = m_opts.m_bodyIdRange.begin();


        m_groupIdToNBodies.clear();
        m_bodiesGroup.clear();
        m_nSimBodies = 0;
        m_nStaticBodies = 0;
        m_nBodies = 0;
        m_nSpecifiedSimBodies = 0;


        for ( XMLNodeType & node  : sceneObjects.children("RigidBodies")) {

            // Parse Body Group
            parseRigidBodies(node);
        }

        LOGSCLEVEL1(m_pSimulationLog, "==================================================================="<<std::endl;)
    }


    void setParsingOptions(DynamicOptionsType o) {
        m_opts = std::move(o);

        if(m_opts.m_bodyIdRange.empty() && Options::m_parseAllIfRangeEmpty) {
            m_parseSelectiveBodyIds = false;
        } else {
            m_parseSelectiveBodyIds = true;
        }

    }

    unsigned int getSpecifiedSimBodies() {
        return m_nSpecifiedSimBodies;
    }

private:

    void parseRigidBodies( XMLNodeType  rigidbodies ) {

        LOGSCLEVEL1(m_pSimulationLog,"==================================" << std::endl <<
                    "---> Parse RigidBodies, group name: "<< rigidbodies.attribute("name").value() << std::endl;);


        //Clear current body list;
        m_bodiesGroup.clear();

        // Check if this group has selecitveIdsOnly turned on to load only selective bodies
        bool hasSelectiveFlag = false;
        auto att = rigidbodies.attribute("enableSelectiveIds");
        if(att) {
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
        } else {
            currGroupIdToNBodies = &(it->second);
            startBodyNr = *currGroupIdToNBodies;
            LOGSCLEVEL2(m_pSimulationLog, "---> Found Group Nr: " <<groupId<< std::endl;)
        }

        // Skip group if we can:
        if( (!Options::m_parseAllBodiesNonSelGroup && !hasSelectiveFlag)
                || ( m_eBodiesState == RigidBodyType::BodyMode::SIMULATED  && !Options::m_parseSimBodies)
                || ( m_eBodiesState == RigidBodyType::BodyMode::STATIC  && !Options::m_parseStaticBodies)
                || (m_parseSelectiveBodyIds && hasSelectiveFlag && m_startRangeIdIt== m_opts.m_bodyIdRange.end()) // out of m_bodyIdRange, no more id's which could be parsed in
          ) {
            LOGSCLEVEL1(m_pSimulationLog, "---> Skip Group" << std::endl;)
            // update the number of bodies in this group and skip this group xml node

            *currGroupIdToNBodies += instances;
            if(m_eBodiesState == RigidBodyType::BodyMode::SIMULATED ) {
                m_nSpecifiedSimBodies += instances;
            }

            return;
        }

        // Full id range for this group would be [m_starBodyId , endIdGroup ]
        m_startIdGroup = RigidBodyId::makeId(groupId,startBodyNr);
        RigidBodyIdType endIdGroup = RigidBodyId::makeId(groupId, startBodyNr+instances-1);
        LOGSCLEVEL2(m_pSimulationLog,"---> Group range: [" << RigidBodyId::getBodyIdString(m_startIdGroup)
                    << "," << RigidBodyId::getBodyIdString(endIdGroup) << "]" << std::endl;)

        typename BodyRangeType::iterator bodyIdIt;
        bool updateStartRange = false;

        if(m_parseSelectiveBodyIds && hasSelectiveFlag) {
            // parse group selective , determine start iterator in bodyRange
            m_bodyIdRangePtr = &m_opts.m_bodyIdRange;
            m_startRangeIdIt = std::lower_bound(m_startRangeIdIt,m_bodyIdRangePtr->end(),m_startIdGroup);

            if( m_startRangeIdIt == m_bodyIdRangePtr->end() || *m_startRangeIdIt > endIdGroup) { // no ids in the range
                *currGroupIdToNBodies += instances;
                LOGSCLEVEL2(m_pSimulationLog,"---> No ids in range: skip" << std::endl;)
                return;
            }
            LOGSCLEVEL2(m_pSimulationLog,"---> Selective range startId: " <<
                        RigidBodyId::getBodyIdString(*m_startRangeIdIt) << std::endl;)

            bodyIdIt = m_startRangeIdIt;
            updateStartRange = true;
        } else {
            // no selective parsing
            // parse all bodies
            //overwrite range containing all bodies, if we don't parse selective ids, parse all bodies!
            m_bodyIdRangePtr = &m_bodyIdRangeTmp;
            m_bodyIdRangeTmp = std::make_pair(m_startIdGroup,endIdGroup+1);
            bodyIdIt = m_bodyIdRangePtr->begin();
            LOGSCLEVEL2(m_pSimulationLog,"---> overwrite selective range... " << std::endl;)
        }

        //update groupIds and specified sim body counter
        *currGroupIdToNBodies += instances;
        if(m_eBodiesState == RigidBodyType::BodyMode::SIMULATED ) {
            m_nSpecifiedSimBodies += instances;
        }

        // Adding bodies in the range =============================
        // iterator bodyRange till the id is > endIdGroup or we are out of the bodyIdRange
        auto itEnd = m_bodyIdRangePtr->end();
        m_parsedInstancesGroup = 0;

        // Reserve as many bodyDatas as big the (possible) range is
        m_bodiesGroup.reserve(m_bodyIdRangePtr->size());
        m_bodiesGroup.clear();

        for( /* nothing*/ ; (bodyIdIt != itEnd) && ( *bodyIdIt <= endIdGroup); ++bodyIdIt ) {
            // Push new body
            if(   (Options::m_allocateSimBodies && m_eBodiesState == RigidBodyType::BodyMode::SIMULATED)
                    || (Options::m_allocateStaticBodies && m_eBodiesState == RigidBodyType::BodyMode::STATIC) ) {
                m_bodiesGroup.emplace_back(new RigidBodyType(*bodyIdIt), *bodyIdIt);
                LOGSCLEVEL3(m_pSimulationLog,"---> Added RigidBody Instance: "<<RigidBodyId::getBodyIdString(*bodyIdIt)<<std::endl);
            } else {
                // add no bodies for visualization stuff, we dont need it!
                m_bodiesGroup.emplace_back( nullptr, *bodyIdIt);
            }
            ++m_parsedInstancesGroup;;
        }

        // Only update start range for selective parsing;
        if(updateStartRange) {
            m_startRangeIdIt = bodyIdIt;
        }

        LOGSCLEVEL2(m_pSimulationLog,"---> Added "<<m_parsedInstancesGroup<<" RigidBody Ids ..."<<std::endl;);
        // =======================================================

        // Parse Geometry
        if(m_pGeomMod) {
            XMLNodeType geometryNode;
            GET_XMLCHILDNODE_CHECK(geometryNode, "Geometry",rigidbodies);
            m_pGeomMod->parseGeometry(geometryNode, &m_bodiesGroup, m_startIdGroup);
        }

        //Parse DynamicsProperties
        parseDynamicProperties(dynPropNode);

        //Copy the pointers!
        if(m_eBodiesState == RigidBodyType::BodyMode::SIMULATED) {
            //LOGSCLEVEL1(m_pSimulationLog,"---> Copy Simulated RigidBody References to DynamicSystem ..."<<std::endl;);
            bool added = addAllBodies(m_pSimBodies);
            if(!added) {
                THROWEXCEPTION("Could not add body to m_simBodies!, some bodies exist already in map!");
            };
            m_nSimBodies += m_parsedInstancesGroup;
            m_nBodies += m_parsedInstancesGroup;
        } else if(m_eBodiesState == RigidBodyType::BodyMode::STATIC) {
            //LOGSCLEVEL1(m_pSimulationLog,"---> Copy Static RigidBody References to DynamicSystem ..."<<std::endl;);
            bool added = addAllBodies(m_pBodies);
            if(!added) {
                THROWEXCEPTION("Could not add body to m_staticBodies!, some bodies exist already in map!");
            };
            m_nStaticBodies += m_parsedInstancesGroup;
            m_nBodies += m_parsedInstancesGroup;
        } else {
            THROWEXCEPTION("---> Adding only simulated and not simulated objects supported!");
        }
        // ===============================================================================================================

        // Parse Visualization (if not empty)
        XMLNodeType  visualizationNode = rigidbodies.child("Visualization");
        if(m_pVisMod && visualizationNode) {
            if(m_pInitStatesMod){
                m_pVisMod->parse(visualizationNode, &m_bodiesGroup, m_pInitStatesMod->getStatesGroup(), m_startIdGroup, m_eBodiesState );
            }else{
                m_pVisMod->parse(visualizationNode, &m_bodiesGroup, nullptr , m_startIdGroup, m_eBodiesState );
            }
        }

        LOGSCLEVEL1(m_pSimulationLog, "==================================" << std::endl;);
    }

    void parseDynamicProperties( XMLNodeType  dynProp) {
        LOGSCLEVEL2(m_pSimulationLog,"---> Parse DynamicProperties ..."<<std::endl;);

        // DynamicState has already been parsed for the group!
        if(m_eBodiesState == RigidBodyType::BodyMode::SIMULATED) {
            parseDynamicPropertiesSimulated(dynProp);
        } else if(m_eBodiesState == RigidBodyType::BodyMode::STATIC) {
            parseDynamicPropertiesStatic(dynProp);
        }
    }
    void parseDynamicState(XMLNodeType  dynProp) {

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
        if(Options::m_allocateSimBodies) {
            // First allocate a new SolverDate structure
            for(auto & b : m_bodiesGroup)  {
                b.m_body->m_pSolverData = new typename RigidBodyType::RigidBodySolverDataType();
                // apply first to all bodies :-)
                b.m_body->m_eMode = m_eBodiesState;
            }

            // Mass ============================================================
            n = dynProp.child("Mass");
            s = n.attribute("distribute").value();
            if(s == "uniform") {
                PREC mass;
                if(!Utilities::stringToType(mass, n.attribute("value").value())) {
                    THROWEXCEPTION("---> String conversion in Mass: value failed");
                }
                for(auto & b : m_bodiesGroup) {
                    b.m_body->m_mass = mass;
                }
            } else if(s == "homogen") {
                PREC density;
                if(!Utilities::stringToType(density, n.attribute("density").value())) {
                    THROWEXCEPTION("---> String conversion in Mass: density failed");
                }
                for(auto & b : m_bodiesGroup) {
                    MassComputations::calculateMass(b.m_body,density);
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
                for(auto & b : m_bodiesGroup) {
                    InertiaTensorComputations::calculateInertiaTensor(b.m_body);
                }
            } else {
                THROWEXCEPTION("---> The attribute 'type' '" << s <<"' of 'InertiaTensor' has no implementation in the parser");
            }
        }

        // InitialPosition ============================================================
        GET_XMLCHILDNODE_CHECK(n,"InitialCondition",dynProp)
        if(m_pInitStatesMod && Options::m_parseInitialCondition) {
            m_pInitStatesMod->parseInitialCondition(n,&m_bodiesGroup,m_startIdGroup,true);
        }

    }
    void parseDynamicPropertiesStatic( XMLNodeType  dynProp) {
        XMLNodeType n ;
        if(Options::m_allocateStaticBodies) {
            n = dynProp.child("Material");
            parseMaterial(n);
        }

        // InitialPosition ============================================================
        GET_XMLCHILDNODE_CHECK(n,"InitialCondition",dynProp)
        bool parseVel = false;
        if(m_pInitStatesMod && Options::m_parseInitialCondition) {
            m_pInitStatesMod->parseInitialCondition(n,&m_bodiesGroup,m_startIdGroup,parseVel,false);
        }


    }

    void parseMaterial(XMLNodeType n) {
        std::string s = n.attribute("distribute").value();
        if(s == "uniform") {
            typename RigidBodyType::BodyMaterialType eMaterial = 0;

            if(!Utilities::stringToType<typename RigidBodyType::BodyMaterialType>(eMaterial, n.attribute("id").value())) {
                THROWEXCEPTION("---> String conversion in Material: id failed");
            }

            for(auto & b : m_bodiesGroup) {
                b.m_body->m_eMaterial = eMaterial;
            }
        } else {
            THROWEXCEPTION("---> The attribute 'distribute' '" << s  <<"' of 'Material' has no implementation in the parser");
        }
    }



    /** General helper template to add all result to the lists */
    template<typename BodyContainer>
    inline bool addAllBodies(BodyContainer * bodies) {
        bool added = true;
        if(bodies) {
            for( auto & b : m_bodiesGroup) {
                added &= bodies->addBody(b.m_body);
            }
        }
        return added;
    }
};

template<typename TParserTraits>
class BodyModuleDummy : public DummyModule{
public:
    template<typename... Args>
    BodyModuleDummy(Args&&... args) : DummyModule("BodyModule") {}

    template<typename... Args>
    void parseModuleOptions(Args&&... args){}
    template<typename... Args>
    void setParsingOptions(Args&&... args){}
    unsigned int getSpecifiedSimBodies(){return 0;}
};


template<typename TParserTraits>
class MPIModuleDummy : public DummyModule {
public:
    template<typename... Args>
    MPIModuleDummy(Args&&... args) : DummyModule("BodyModule") {}
};



};

#endif
