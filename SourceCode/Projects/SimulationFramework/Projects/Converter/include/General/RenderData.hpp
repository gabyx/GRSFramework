#ifndef RenderData_hpp
#define RenderData_hpp


#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include <map>
#include <unordered_map>
#include <vector>

#include DynamicsSystem_INCLUDE_FILE

#include "RenderMaterial.hpp"
#include "RenderScriptGenerator.hpp"

namespace ParserModules {
template<typename TParserTraits>
class VisModuleConverter {
private:
    DEFINE_PARSER_TYPE_TRAITS(TParserTraits)
    DEFINE_MATRIX_TYPES

    LogType * m_pSimulationLog;

    using ScalesMap = typename DynamicsSystemType::ScalesMap;
    ScalesMap * m_scalesMap;

    using VisMeshMap = typename DynamicsSystemType::VisMeshMap;
    VisMeshMap * m_visMeshMap;

    using StatesGroupType = typename InitStatesModuleType::StatesGroupType;
    using BodyMode = typename DynamicsSystemType::RigidBodyType::BodyMode;
    using BodyListType = typename BodyModuleType::BodyListType;

    BodyListType * m_bodyListGroup;

public:

    VisModuleConverter(ParserType * p, ScalesMap * scales , VisMeshMap * meshMap)
        : m_pSimulationLog(p->getSimLog()), m_scalesMap(scales), m_visMeshMap(meshMap) {
        ASSERTMSG(m_scalesMap && m_visMeshMap, "these should not be zero!")
    };

    void parse(XMLNodeType vis, BodyListType * bodyList, StatesGroupType * states, RigidBodyIdType startId, BodyMode mode) {

        m_bodyListGroup = bodyList;

        LOGSCLEVEL1(m_pSimulationLog, "---> VisModuleConverter: parsing (BodyVisualization)"<<std::endl;)
        XMLNodeType node = vis.child("Mesh");

        if(node) {
            parseScale(node);
            parseMesh(node);
        } else {
            node = vis.child("Plane");
            parseScale(node);
        }
    }

    template<typename... Args>
    void parseSceneSettingsPost(Args&&... args) {
        //Do nothing
    }

    void cleanUp(){};

private:
    void parseScale(XMLNodeType node){
        XMLAttributeType att;
        bool scaleLikeGeometry = false;
        Vector3 scale;
        att = node.attribute("scaleLikeGeometry");
        if(att) {
            if(!Utilities::stringToType(scaleLikeGeometry, att.value())) {
                THROWEXCEPTION("---> String conversion in parseMesh: scaleWithGeometry failed");
            }
        }
        if(!scaleLikeGeometry) {
            if(!Utilities::stringToVector3(scale, node.attribute("scale").value() )) {
                THROWEXCEPTION("---> String conversion in parseMesh: scale failed");
            }
            for(auto & b : *m_bodyListGroup) {
                m_scalesMap->emplace( b.m_id , scale);
            }

        }
    }
    // Virtual function in SceneParser!, this function adds all objects to Ogre related objects!
    void parseMesh(XMLNodeType  meshNode ) {
        XMLAttributeType att;

        GET_XMLATTRIBUTE_CHECK(att,"file",meshNode);
        boost::filesystem::path meshName = att.value();

        for(auto & b : *m_bodyListGroup) {
            m_visMeshMap->emplace( b.m_id , meshName);
        }

    }
};

};



#define  DEFINE_RENDERCONVERTERDATA_TYPES  \
    DEFINE_DYNAMICSYSTEM_BASE_TYPES \
    using GeometryMapType = std::unordered_map< RigidBodyIdType , typename RigidBodyType::GeometryType>; \
    using ScalesMap = std::unordered_map< RigidBodyIdType ,Vector3 >; \
    using VisMeshMap = std::unordered_map< RigidBodyIdType , boost::filesystem::path  >; \
    using MaterialMapType = std::unordered_map<unsigned int, RenderMaterial * >; \
    using RenderScriptGen = RenderScriptGenerator;

class RenderData {
public:

    DEFINE_RENDERCONVERTERDATA_TYPES

    ~RenderData(){
        // Delete all materials!
        for( auto & m : m_materials){
            delete m.second;
        }
    }

    GlobalGeometryMapType m_globalGeometries;

    GeometryMapType m_geometryMap;

    ScalesMap m_scales;

    VisMeshMap m_visMeshs;


    MaterialMapType m_materials;
    RenderScriptGen m_renderScriptGen;


};


#endif
