#ifndef GRSF_General_RenderData_hpp
#define GRSF_General_RenderData_hpp


#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include <map>
#include <unordered_map>
#include <vector>

#include DynamicsSystem_INCLUDE_FILE

#include "GRSF/Converters/Renderer/RenderMaterial.hpp"


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
        XMLNodeType node = vis.first_child();
        std::string n = node.name();
        if( n == "Mesh"){
            parseMesh(node);
        }

        auto p = VisSubModuleScale::parseScale(node,n);

        for(auto & b : *m_bodyListGroup) {
            m_scalesMap->emplace( b.m_id , p.second );
        }
    }

    template<typename... Args>
    void parseSceneSettingsPost(Args&&... args) {
        //Do nothing
    }

    void cleanUp(){};

private:

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

};


#endif
