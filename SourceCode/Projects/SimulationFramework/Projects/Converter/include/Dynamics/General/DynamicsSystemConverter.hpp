#ifndef DynamicsSystemConverter_hpp
#define DynamicsSystemConverter_hpp


#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include <map>
#include <unordered_map>
#include <vector>

#include "DynamicsSystem.hpp"
#include "SceneParser.hpp"
#include "MaterialsCollectionParser.hpp"

#include RigidBody_INCLUDE_FILE


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

//class MaterialGenerator{
//public:
//    generateMaterial(){}
//};
//
//class MaterialMapperBase{
//public:
//    MaterialMapperBase(){};
//    virtual getMaterial
//};

class DynamicsSystemConverter {
public:
    DEFINE_DYNAMICSYSTEM_BASE_TYPES
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GlobalGeometryMapType m_globalGeometries;

    using GeometryMapType        = std::unordered_map< RigidBodyIdType , typename RigidBodyType::GeometryType>;
    GeometryMapType m_geometryMap;

    using ScalesMap = std::unordered_map< RigidBodyIdType ,Vector3 >;
    ScalesMap m_scales;

    using VisMeshMap = std::unordered_map< RigidBodyIdType , boost::filesystem::path  >;
    VisMeshMap m_visMeshs;

    struct ParserModulesCreator{
        ParserModulesCreator( DynamicsSystemConverter * p): m_p(p){}
        DynamicsSystemConverter * m_p;

        template<typename TSceneParser, typename TDynamicsSystem>
        struct SceneParserTraits : SceneParserBaseTraits<TSceneParser,TDynamicsSystem> {
            // Module typedefs
            using SettingsModuleType         = ParserModules::SettingsModule<SceneParserTraits>;
            using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<SceneParserTraits>;
            using ContactParamModuleType     = ParserModules::ContactParamModule<SceneParserTraits>;
            using InitStatesModuleType       = ParserModules::InitStatesModule<SceneParserTraits> ;

            using BodyMStaticOptions         = ParserModules::BodyModuleStaticOptions<true,true,true,true,false,false,false>;
            using BodyModuleType             = ParserModules::BodyModule< SceneParserTraits, BodyMStaticOptions > ;

            using GeomMStaticOptions         = ParserModules::GeometryModuleStaticOptions<false,true,false,true>;
            using GeometryModuleType         = ParserModules::GeometryModule<SceneParserTraits,GeomMStaticOptions>;

            using VisModuleType              = ParserModules::VisModuleConverter<SceneParserTraits>;
            using MPIModuleType              = ParserModules::MPIModuleDummy<SceneParserTraits>;
        };


        template<typename TParser>
        std::tuple< std::unique_ptr<typename TParser::SettingsModuleType >,
            std::unique_ptr<typename TParser::ExternalForcesModuleType >,
            std::unique_ptr<typename TParser::ContactParamModuleType>,
            std::unique_ptr<typename TParser::InitStatesModuleType >,
            std::unique_ptr<typename TParser::BodyModuleType >,
            std::unique_ptr<typename TParser::GeometryModuleType >,
            std::unique_ptr<typename TParser::VisModuleType>,
            std::unique_ptr<typename TParser::MPIModuleType>
            >
        createParserModules(TParser * p) {

            using SettingsModuleType       = typename TParser::SettingsModuleType ;
            using ContactParamModuleType   = typename TParser::ContactParamModuleType;
            using GeometryModuleType       = typename TParser::GeometryModuleType ;
            using InitStatesModuleType     = typename TParser::InitStatesModuleType ;
            using ExternalForcesModuleType = typename TParser::ExternalForcesModuleType ;
            using BodyModuleType           = typename TParser::BodyModuleType ;
            using VisModuleType            = typename TParser::VisModuleType ;
            using MPIModuleType            = typename TParser::MPIModuleType ;

            auto sett = std::unique_ptr<SettingsModuleType >(nullptr);
            auto geom = std::unique_ptr<GeometryModuleType >(new GeometryModuleType(p, &m_p->m_globalGeometries, nullptr, &m_p->m_geometryMap) );
            auto is  = std::unique_ptr<InitStatesModuleType >(nullptr);
            auto vis = std::unique_ptr<VisModuleType>(new VisModuleType(p, &m_p->m_scales, &m_p->m_visMeshs)); // no visualization needed
            auto bm  = std::unique_ptr<BodyModuleType>(new BodyModuleType(p, geom.get(), nullptr, vis.get() , nullptr, nullptr )) ;
            auto es  = std::unique_ptr<ExternalForcesModuleType >(nullptr);
            auto con = std::unique_ptr<ContactParamModuleType>(nullptr);

            auto mpi = std::unique_ptr<MPIModuleType>(nullptr);

            return std::make_tuple(std::move(sett),std::move(es),std::move(con),std::move(is),std::move(bm),std::move(geom),std::move(vis),std::move(mpi));
        };

    };




    using MaterialsMapType = std::unordered_map<unsigned int, std::string>;
    MaterialsMapType m_materials;

    struct MatCollParserModulesCreator{
        MatCollParserModulesCreator( DynamicsSystemConverter * p): m_p(p){}
        DynamicsSystemConverter * m_p;

        template<typename TSceneParser, typename TDynamicSystem>
        using MatCollParserTraits = MatCollParserTraits<TSceneParser,TDynamicSystem>;

        template<typename TParser>
        std::tuple< std::unique_ptr<typename TParser::MaterialsModuleType>>
        createParserModules(TParser * p) {

            using MaterialsModuleType = typename TParser::MaterialsModuleType;
            auto mat = std::unique_ptr<MaterialsModuleType >(new MaterialsModuleType(p, &m_p->m_materials));

            return std::make_tuple(std::move(mat));
        };

    };




};


#endif
