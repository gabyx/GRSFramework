#ifndef RenderScriptParserModules_hpp
#define RenderScriptParserModules_hpp


#include <vector>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "AssertionDebug.hpp"

#include "RenderScriptParserBaseTraits.hpp"

#include "RenderScriptGeneratorLogic.hpp"
#include "RenderOutputLogic.hpp"

namespace RenderMatParserModules {

    template<typename TParserTraits>
    class MaterialsModule {
    public:
        DEFINE_MATCOLPARSER_TYPE_TRAITS(TParserTraits)

        using MaterialMapType = typename CollectionType::MaterialMapType;

        MaterialsModule(ParserType * p, MaterialMapType * m):m_parser(p),m_materials(m), m_pLog(p->getLog()) {}

        void parse(XMLNodeType & materialNode) {
            auto nodes = materialNode.children("Material");
            auto itNodeEnd = nodes.end();
            for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {
                 unsigned int id;
                 if(!Utilities::stringToType(id, itNode->attribute("id").value())) {
                      THROWEXCEPTION("---> String conversion in Material: id failed");
                 }

                 ASSERTMSG(itNode->child_value()!=""," String in material id: " << id << "is empty!")
                 m_materials->emplace(id, new RenderMaterial(id,itNode->child_value()) );

                 LOGMCLEVEL3(m_pLog,"---> Parsed Material with id: " << id << std::endl;)
            }
        }

        void cleanUp() {
        }

        MaterialMapType * getMaterialMap(){return m_materials;}
    private:
        ParserType * m_parser;
        LogType * m_pLog;
        MaterialMapType * m_materials;
    };


    template<typename TParserTraits>
    class MaterialGenerator {
    public:
        DEFINE_MATCOLPARSER_TYPE_TRAITS(TParserTraits)

        using GeometryMapType = typename CollectionType::GeometryMapType;
        using MaterialMapType = typename CollectionType::MaterialMapType;
        using RenderScriptGen = typename CollectionType::RenderScriptGen;

        MaterialGenerator(ParserType * p, RenderScriptGen * g, GeometryMapType * geomMap)
        :m_parser(p),m_matGen(g), m_pLog(p->getLog()), m_geomMap(geomMap) {}

        void parse(XMLNodeType & matGenNode, MaterialMapType * materials) {

            m_materials = materials;

            // Add all tools into the execution list!
            auto nodes = matGenNode.children("Tool");
            auto itNodeEnd = nodes.end();
            for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {

                 unsigned int id;
                 if(!Utilities::stringToType(id, itNode->attribute("id").value())) {
                      THROWEXCEPTION("---> String conversion in Tool: id failed");
                 }
                 LOGMCLEVEL3(m_pLog,"---> Parsing Tool with id: " << id << std::endl;);
                 std::string type = itNode->attribute("type").value();
                 if(type == "BodyDataInput"){
                        createToolBodyData(*itNode,id);
                 }else if(type == "MaterialLookUp"){
                        createToolMaterialLookUp(*itNode,id);
                 }else if(type == "DisplacementToPosQuat"){
                        createToolDisplacementToPosQuat(*itNode,id);
                 }else if(type == "RendermanOutput"){
                        createToolRendermanOutput(*itNode,id);
                 }else{
                        THROWEXCEPTION("---> String conversion in Tool: type not found!");
                 }


            }


            // Add all tools into the execution list!
            nodes = matGenNode.children("Link");
            itNodeEnd = nodes.end();
            for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {

                 unsigned int outNode;
                 if(!Utilities::stringToType(outNode, itNode->attribute("outNode").value())) {
                      THROWEXCEPTION("---> String conversion in Tool: id failed");
                 }
                 unsigned int outSocket;
                 if(!Utilities::stringToType(outSocket, itNode->attribute("outSocket").value())) {
                      THROWEXCEPTION("---> String conversion in Tool: id failed");
                 }

                 unsigned int inNode;
                 if(!Utilities::stringToType(inNode, itNode->attribute("inNode").value())) {
                      THROWEXCEPTION("---> String conversion in Tool: id failed");
                 }
                 unsigned int inSocket;
                 if(!Utilities::stringToType(inSocket, itNode->attribute("inSocket").value())) {
                      THROWEXCEPTION("---> String conversion in Tool: id failed");
                 }

                 LOGMCLEVEL3(m_pLog,"---> Linking Tool:  " << outNode << " out: " << outSocket << " ------> "
                             << inSocket << ":in Tool: " << inNode << std::endl;);
                 // Link the nodes
                 m_matGen->link(outNode,outSocket,inNode,inSocket);


            }

        }

        void cleanUp() {
        }

    private:

        void createToolBodyData(XMLNodeType & matGenNode, unsigned int id){

            auto * node = new LogicNodes::BodyData(id);
            m_matGen->addNode(node,true,false);

        }

        void createToolDisplacementToPosQuat(XMLNodeType & matGenNode, unsigned int id){

            auto * node = new LogicNodes::DisplacementToPosQuat(id);
            m_matGen->addNode(node,false,false);

        }

        void createToolMaterialLookUp(XMLNodeType & matGenNode, unsigned int id){

            // Get default material
            auto it = m_materials->begin();


            if(it == m_materials->end()){
                ERRORMSG("No default material found for MaterialLookUp tool!")
            }
            LOGMCLEVEL3(m_pLog, "Default Material set to: " << std::endl << it->second->getMaterialString() << std::endl)

            std::string type = matGenNode.attribute("inputType").value();
            if( type == "unsigned int"){
                auto * node = new LogicNodes::LookUpTable<unsigned int,
                                            RenderMaterial *,
                                            MaterialMapType >(id,m_materials,it->second);
                m_matGen->addNode(node,false,false);
            }else if(type =="unsigned long int"){
                auto * node = new LogicNodes::LookUpTable<unsigned long int,
                                            RenderMaterial *,
                                            MaterialMapType >(id,m_materials,it->second);
                m_matGen->addNode(node,false,false);
            }else if(type =="unsigned long long int"){
                auto * node = new LogicNodes::LookUpTable<unsigned long long int,
                                            RenderMaterial *,
                                            MaterialMapType >(id,m_materials,it->second);
                m_matGen->addNode(node,false,false);
            }else{
                THROWEXCEPTION("---> String conversion in MaterialLookUp tool: inputType: '" << type << "' not found!");
            }


        }


        void createToolRendermanOutput(XMLNodeType & matGenNode, unsigned int id){

            auto * node = new LogicNodes::RendermanOutput(id, m_geomMap);
            m_matGen->addNode(node,false,true);

        }

        ParserType * m_parser;
        LogType * m_pLog;
        RenderScriptGen * m_matGen;
        MaterialMapType * m_materials;
        GeometryMapType * m_geomMap;
    };

};

#endif
