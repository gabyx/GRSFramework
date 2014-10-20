#ifndef RenderMaterialParserModules_hpp
#define RenderMaterialParserModules_hpp


#include <vector>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "AssertionDebug.hpp"

#include "RenderMaterialParserBaseTraits.hpp"

#include "RenderMaterialGenLogic.hpp"


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
                 auto s = std::shared_ptr<RenderMaterial>( new RenderMaterial(id,itNode->child_value()) );
                 m_materials->emplace(id, s);

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

        using MaterialMapType = typename CollectionType::MaterialMapType;
        using MaterialGenType = typename CollectionType::MaterialGenType;

        MaterialGenerator(ParserType * p, MaterialGenType * g):m_parser(p),m_matGen(g), m_pLog(p->getLog()) {}

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

                 bool isInput = false;
                 auto att = itNode->attribute("isInput");
                 if(att){
                    if(!Utilities::stringToType(isInput, att.value())) {
                      THROWEXCEPTION("---> String conversion in Tool: isInput failed");
                    }
                 }

                 bool isOutput = false;
                 att = itNode->attribute("isOutput");
                 if(att){
                    if(!Utilities::stringToType(isOutput, att.value())) {
                      THROWEXCEPTION("---> String conversion in Tool: isOutput failed");
                    }
                 }


                 std::string type = itNode->attribute("type").value();
                 if(type == "BodyData"){
                        createToolBodyData(*itNode,id,isInput,isOutput);
                 }else if(type == "MaterialLookUp"){
                        createToolMaterialLookUp(*itNode,id,isInput,isOutput);

                 }else{
                        THROWEXCEPTION("---> String conversion in Tool: type not found!");
                 }

                 LOGMCLEVEL3(m_pLog,"---> Parsed Tool with id: " << id << std::endl;);
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

                 // Link the nodes
                 m_matGen->link(outNode,outSocket,inNode,inSocket);

                 LOGMCLEVEL3(m_pLog,"---> Linked Tool:  " << outNode << " out: " << outSocket << " ------> "
                             << inSocket << ":in Tool: " << inNode << std::endl;);
            }

        }

        void cleanUp() {
        }

    private:

        void createToolBodyData(XMLNodeType & matGenNode, unsigned int id, bool isInput, bool isOutput){

            auto * node = new LogicNodes::BodyData(id);
            m_matGen->addNode(node,isInput,isOutput);

        }

        void createToolMaterialLookUp(XMLNodeType & matGenNode, unsigned int id, bool isInput, bool isOutput){

            // Get default material
            auto it = m_materials->begin();


            if(it == m_materials->end()){
                ERRORMSG("No default material found for MaterialLookUp tool!")
            }
            LOGMCLEVEL3(m_pLog, "Default Material set to: " << std::endl << it->second->getMaterialString() << std::endl)

            std::string type = matGenNode.attribute("inputType").value();
            if( type == "unsigned int"){
                auto * node = new LogicNodes::LookUpTable<unsigned int,
                                            std::shared_ptr<RenderMaterial>,
                                            MaterialMapType >(id,m_materials,it->second);
                m_matGen->addNode(node,isInput,isOutput);
            }else if(type =="unsigned long int"){
                auto * node = new LogicNodes::LookUpTable<unsigned long int,
                                            std::shared_ptr<RenderMaterial>,
                                            MaterialMapType >(id,m_materials,it->second);
                m_matGen->addNode(node,isInput,isOutput);
            }else if(type =="unsigned long long int"){
                auto * node = new LogicNodes::LookUpTable<unsigned long long int,
                                            std::shared_ptr<RenderMaterial>,
                                            MaterialMapType >(id,m_materials,it->second);
                m_matGen->addNode(node,isInput,isOutput);
            }else{
                THROWEXCEPTION("---> String conversion in MaterialLookUp tool: inputType: '" << type << "' not found!");
            }


        }


        ParserType * m_parser;
        LogType * m_pLog;
        MaterialGenType * m_matGen;
        MaterialMapType * m_materials;
    };

};

#endif
