#ifndef GRSF_General_RenderLogicParserModules_hpp
#define GRSF_General_RenderLogicParserModules_hpp


#include <vector>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/Common/XMLMacros.hpp"

#include "GRSF/Dynamics/General/ParserFunctions.hpp"
#include "GRSF/General/GridderLogicParserTraitsMacro.hpp"

namespace GridderLogicParserModules {

//    template<typename TParserTraits>
//    class MaterialsModule {
//    public:
//        DEFINE_RENDERLOGICPARSER_TYPE_TRAITS(TParserTraits)
//
//        using MaterialMapType = typename DataStorageType::MaterialMapType;
//
//        MaterialsModule(ParserType * p, MaterialMapType * m):m_parser(p),m_materials(m), m_pLog(p->getLog()) {}
//
//        void parse(XMLNodeType & parent) {
//
//            XMLNodeType materialNode = parent.child("Materials");
//            if(!materialNode) {
//                return;
//            }
//
//            auto nodes = materialNode.children("Material");
//            auto itNodeEnd = nodes.end();
//            for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {
//                unsigned int id;
//                if(!Utilities::stringToType(id, itNode->attribute("id").value())) {
//                    ERRORMSG("---> String conversion in Material: id failed");
//                }
//
//                ASSERTMSG(itNode->child_value()!=""," String in material id: " << id << "is empty!")
//                m_materials->emplace(id, new RenderMaterial(id,itNode->child_value()) );
//
//                LOGLPLEVEL3(m_pLog,"---> Parsed Material with id: " << id << std::endl;)
//            }
//
//        }
//
//        void cleanUp() {
//        }
//
//        MaterialMapType * getMaterialMap() {
//            return m_materials;
//        }
//    private:
//        ParserType * m_parser;
//        LogType * m_pLog;
//        MaterialMapType * m_materials;
//    };

};



namespace GridderLogicParserModules {

template<typename TParserTraits>
class LogicModule {
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    DEFINE_GRIDDERLOGICPARSER_TYPE_TRAITS(TParserTraits)

    using GridExtSettingsListType = typename DataStorageType::GridExtSettingsListType;


    LogicModule(ParserType * p, GridExtSettingsListType * gridSettingsList)
        : m_pLog(p->getLog()), m_gridSettingsList(gridSettingsList) {}

    void parse(XMLNodeType & parent) {

        XMLNodeType logicNode = parent.child("Logic");
        if(!logicNode) {
            return;
        }
        // Add all tools into the execution list!
        auto nodes = logicNode.children("GridExtraction");
        auto itNodeEnd = nodes.end();
        for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {
           parseGridExtraction(*itNode);
        }

    }
    void parseGridExtraction(XMLNodeType & gridExt){

        m_gridSettingsList->emplace_back();
        auto & settings =  m_gridSettingsList->back();

        settings.m_fileName = gridExt.attribute("fileName").value();
        if(settings.m_fileName.empty()){
            ERRORMSG("---> String conversion 'fileName' failed");
        }

        XMLNodeType grid;
        GET_XMLCHILDNODE_CHECK( grid , "Grid",  gridExt);

        parseGrid(grid,settings);

    }

    template<typename TSettings>
    void parseGrid(XMLNodeType & grid, TSettings & settings){


            Vector3 minPoint;
            if(!Utilities::stringToVector3(minPoint,  grid.attribute("minPoint").value())) {
                ERRORMSG("---> String conversion 'minPoint' failed");
            }

            Vector3 maxPoint;
            if(!Utilities::stringToVector3(maxPoint,  grid.attribute("maxPoint").value())) {
                ERRORMSG("---> String conversion 'maxPoint' failed");
            }

            if(!Utilities::stringToVector3(settings.m_dimension,  grid.attribute("dimension").value())) {
                ERRORMSG("---> String conversion 'dimensions' failed");
            }

            Quaternion q_KI;
            Vector3 I_r_IK;
            ParserFunctions::parseTransformSequence(grid,q_KI,I_r_IK);

            settings.m_R_KI = q_KI.toRotationMatrix();

            I_r_IK = settings.m_R_KI.transpose()*I_r_IK; // make K_r_IK  = A_KI * I_r_IK
            settings.m_aabb = AABB3d( I_r_IK + minPoint, I_r_IK + maxPoint);



            LOGGPLEVEL1(m_pLog,"---> Parsed GridSettings for file: " << settings.m_fileName <<std::endl;)

    }

    void cleanUp() {}

private:
    LogType * m_pLog;
    GridExtSettingsListType * m_gridSettingsList;

};

};

#undef DEFINE_MAKEColorList
#undef DEFINE_ColorList2
#undef DEFINE_ColorList

#endif

