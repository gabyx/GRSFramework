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


namespace RenderScriptParserModules {

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
                ERRORMSG("---> String conversion in Material: id failed");
            }

            ASSERTMSG(itNode->child_value()!=""," String in material id: " << id << "is empty!")
            m_materials->emplace(id, new RenderMaterial(id,itNode->child_value()) );

            LOGMCLEVEL3(m_pLog,"---> Parsed Material with id: " << id << std::endl;)
        }
    }

    void cleanUp() {
    }

    MaterialMapType * getMaterialMap() {
        return m_materials;
    }
private:
    ParserType * m_parser;
    LogType * m_pLog;
    MaterialMapType * m_materials;
};


template<typename TParserTraits>
class ScriptGeneratorModule {
public:
    DEFINE_MATCOLPARSER_TYPE_TRAITS(TParserTraits)

    using GeometryMapType = typename CollectionType::GeometryMapType;
    using MaterialMapType = typename CollectionType::MaterialMapType;
    using RenderScriptGen = typename CollectionType::RenderScriptGen;

    using ExecGroups = typename RenderScriptGen::ExecGroups;

    ScriptGeneratorModule(ParserType * p, RenderScriptGen * g, GeometryMapType * geomMap)
        :m_parser(p),m_renderScriptGen(g), m_pLog(p->getLog()), m_geomMap(geomMap) {}

    void parse(XMLNodeType & matGenNode, MaterialMapType * materials) {

        m_materials = materials;

        // Add all tools into the execution list!
        auto nodes = matGenNode.children("Tool");
        auto itNodeEnd = nodes.end();
        for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {

            unsigned int id;
            if(!Utilities::stringToType(id, itNode->attribute("id").value())) {
                ERRORMSG("---> String conversion in Tool: id failed");
            }
            LOGMCLEVEL3(m_pLog,"---> Parsing Tool with id: " << id << std::endl;);
            std::string type = itNode->attribute("type").value();
            if(type == "BodyDataInput") {
                createToolBodyData(*itNode,id);
            }else if(type == "FrameDataInput") {
                createToolFrameData(*itNode,id);
            } else if(type == "MaterialLookUp") {
                createToolMaterialLookUp(*itNode,id);
            } else if(type == "DisplacementToPosQuat") {
                createToolDisplacementToPosQuat(*itNode,id);
            }else if(type == "Constant"){
                createToolConstant(*itNode,id);
            } else if(type == "RendermanWriter") {
                createToolRendermanOutput(*itNode,id);
            } else {
                ERRORMSG("---> String conversion in Tool: type not found!");
            }


        }


        // Add all links
        nodes = matGenNode.children("Link");
        itNodeEnd = nodes.end();
        for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {

            unsigned int outNode;
            if(!Utilities::stringToType(outNode, itNode->attribute("outNode").value())) {
                ERRORMSG("---> String conversion in Tool: id failed");
            }
            unsigned int outSocket;
            if(!Utilities::stringToType(outSocket, itNode->attribute("outSocket").value())) {
                ERRORMSG("---> String conversion in Tool: id failed");
            }

            unsigned int inNode;
            if(!Utilities::stringToType(inNode, itNode->attribute("inNode").value())) {
                ERRORMSG("---> String conversion in Tool: id failed");
            }
            unsigned int inSocket;
            if(!Utilities::stringToType(inSocket, itNode->attribute("inSocket").value())) {
                ERRORMSG("---> String conversion in Tool: id failed");
            }

            LOGMCLEVEL3(m_pLog,"---> Linking Tool:  " << outNode << " out: " << outSocket << " ------> "
                        << inSocket << ":in Tool: " << inNode << std::endl;);
            // Link the nodes
            m_renderScriptGen->link(outNode,outSocket,inNode,inSocket);


        }

    }

    void cleanUp() {
    }

private:


    #define DEFINE_CONSTANT2(type, typeName) \
        ( t == #typeName ){ using T = type; \
        T tt; \
        if(!Utilities::stringToType(tt, matGenNode.attribute("value").value())) { \
            ERRORMSG("---> String conversion in Constant tool: value failed"); \
        } \
        n = new LogicNodes::ConstantNode<T>(id,tt); \
        } \

    #define DEFINE_CONSTANT(type) DEFINE_CONSTANT2(type,type)

    void createToolConstant(XMLNodeType & matGenNode, unsigned int id){

            std::string t = matGenNode.attribute("outputType").value();
            LogicNode * n;
            if DEFINE_CONSTANT(float)
            else if DEFINE_CONSTANT(double)
            else if DEFINE_CONSTANT(char)
            else if DEFINE_CONSTANT(short)
            else if DEFINE_CONSTANT(int)
            else if DEFINE_CONSTANT(long int)
            else if DEFINE_CONSTANT(long long int)
            else if DEFINE_CONSTANT(unsigned char)
            else if DEFINE_CONSTANT(unsigned short)
            else if DEFINE_CONSTANT(unsigned int)
            else if DEFINE_CONSTANT(unsigned long int)
            else if DEFINE_CONSTANT(unsigned long long int)
            else if ( t == "string" ){
                using T = std::string;
                T tt = matGenNode.attribute("value").value();
                if(tt.empty()){
                    ERRORMSG("---> String conversion in Constant tool: value failed"); \
                }
                n = new LogicNodes::ConstantNode<T>(id,tt);
            }
            else if ( t == "path" ){
                using T = boost::filesystem::path;
                std::string tt = matGenNode.attribute("value").value();
                if(tt.empty()){
                    ERRORMSG("---> String conversion in Constant tool: value failed"); \
                }
                n = new LogicNodes::ConstantNode<T>(id,tt);
            }
            else{
                ERRORMSG("---> String conversion in Constant tool: outputType: '" << t << "' not found!");
            }

            m_renderScriptGen->addNode(n,false,false);

            XMLAttributeType att;
            att = matGenNode.attribute("groupId");
            if(att) {
                std::string gid = att.value();
                if(gid == "Body"){
                    m_renderScriptGen->addNodeToGroup(id, ExecGroups::BODY);
                }else if(gid == "Frame"){
                    m_renderScriptGen->addNodeToGroup(id, ExecGroups::FRAME);
                }else{
                    ERRORMSG("---> String conversion in Constant tool: groupId: '" << gid << "' not found!");
                }
            }
    }


    void createToolFrameData(XMLNodeType & matGenNode, unsigned int id) {
        auto * node = new LogicNodes::FrameData(id);
        m_renderScriptGen->addNode(node,true,false);
        m_renderScriptGen->addNodeToGroup(id,ExecGroups::FRAME);
        m_renderScriptGen->setFrameData(node);
    }

    void createToolBodyData(XMLNodeType & matGenNode, unsigned int id) {
        auto * node = new LogicNodes::BodyData(id);
        m_renderScriptGen->addNode(node,true,false);
        m_renderScriptGen->addNodeToGroup(id,ExecGroups::BODY);
        m_renderScriptGen->setBodyData(node);
    }

    void createToolDisplacementToPosQuat(XMLNodeType & matGenNode, unsigned int id) {

        auto * node = new LogicNodes::DisplacementToPosQuat(id);
        m_renderScriptGen->addNode(node,false,false);
        m_renderScriptGen->addNodeToGroup(id,ExecGroups::BODY);

    }

    void createToolMaterialLookUp(XMLNodeType & matGenNode, unsigned int id) {


        unsigned int defaultMaterialId;
        if(!Utilities::stringToType(defaultMaterialId, matGenNode.attribute("defaultMaterialId").value())) {
            ERRORMSG("---> String conversion in MaterialLookUp tool: defaultMaterialId failed");
        }

        // Get default material

        auto it = m_materials->find(defaultMaterialId);


        if(it == m_materials->end()) {
            ERRORMSG("No default material found for MaterialLookUp tool!")
        }
        LOGMCLEVEL3(m_pLog, "Default Material set to: " << std::endl << it->second->getMaterialString() << std::endl)

        std::string type = matGenNode.attribute("inputType").value();
        LogicNode * node = nullptr;
        if( type == "unsigned int") {
             node = new LogicNodes::LookUpTable<unsigned int,
                                                        RenderMaterial *,
                                                        MaterialMapType >(id,m_materials,it->second);
        } else if(type =="unsigned long int") {
            node = new LogicNodes::LookUpTable<unsigned long int,
                                                        RenderMaterial *,
                                                        MaterialMapType >(id,m_materials,it->second);
        } else if(type =="unsigned long long int") {
           node = new LogicNodes::LookUpTable<unsigned long long int,
                                                        RenderMaterial *,
                                                        MaterialMapType >(id,m_materials,it->second);
        } else {
            ERRORMSG("---> String conversion in MaterialLookUp tool: inputType: '" << type << "' not found!");
        }

         m_renderScriptGen->addNode(node,false,false);
         m_renderScriptGen->addNodeToGroup(id,ExecGroups::BODY);
    }


    void createToolRendermanOutput(XMLNodeType & matGenNode, unsigned int id) {
        XMLAttributeType att;
        bool pipe = false;
        std::string command ="";
        std::string suffix ="";
        att = matGenNode.attribute("pipeToSubprocess");
        if(att) {
            if(!Utilities::stringToType(pipe, att.value())) {
                ERRORMSG("---> String conversion in RendermanWriter tool: pipeToSubprocess failed");
            }

            if( pipe ) {
                command = matGenNode.attribute("command").value();
                if(command.empty()) {
                    ERRORMSG("---> String conversion in RendermanWriter tool: command failed");
                }
            }

            suffix = matGenNode.attribute("suffix").value();

        }

        auto * node = new LogicNodes::RendermanWriter(id, m_geomMap,pipe,command,suffix);
        m_renderScriptGen->addNode(node,false,true);
    }

    ParserType * m_parser;
    LogType * m_pLog;
    RenderScriptGen * m_renderScriptGen;
    MaterialMapType * m_materials;
    GeometryMapType * m_geomMap;
};

};

#endif

