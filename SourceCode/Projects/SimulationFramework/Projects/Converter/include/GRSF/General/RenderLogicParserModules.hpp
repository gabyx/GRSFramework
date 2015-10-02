#ifndef GRSF_General_RenderLogicParserModules_hpp
#define GRSF_General_RenderLogicParserModules_hpp


#include <vector>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/General/RenderLogicParserTraitsMacro.hpp"

namespace RenderLogicParserModules {

    template<typename TParserTraits>
    class MaterialsModule {
    public:
        DEFINE_RENDERLOGICPARSER_TYPE_TRAITS(TParserTraits)

        using MaterialMapType = typename DataStorageType::MaterialMapType;

        MaterialsModule(ParserType * p, MaterialMapType * m):m_parser(p),m_materials(m), m_pLog(p->getLog()) {}

        void parse(XMLNodeType & parent) {

            XMLNodeType materialNode = parent.child("Materials");
            if(!materialNode) {
                return;
            }

            auto nodes = materialNode.children("Material");
            auto itNodeEnd = nodes.end();
            for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {
                unsigned int id;
                if(!Utilities::stringToType(id, itNode->attribute("id").value())) {
                    ERRORMSG("---> String conversion in Material: id failed");
                }

                ASSERTMSG(itNode->child_value()!=""," String in material id: " << id << "is empty!")
                m_materials->emplace(id, new RenderMaterial(id,itNode->child_value()) );

                LOGLPLEVEL3(m_pLog,"---> Parsed Material with id: " << id << std::endl;)
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

};


#include "GRSF/General/LogicParserModules.hpp"
#include "GRSF/General/RenderExecutionGraph.hpp"
#include "GRSF/General/RenderExecutionGraphNodes.hpp"

namespace RenderLogicParserModules {

template<typename TParserTraits>
class LogicModule : public LogicParserModules::LogicModule<TParserTraits> {
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    DEFINE_RENDERLOGICPARSER_TYPE_TRAITS(TParserTraits)

    using Base =  LogicParserModules::LogicModule<TParserTraits> ;
    using NodeGroups = typename Base::NodeGroups;

    using GeometryMapType = typename DataStorageType::GeometryMapType;
    using MaterialMapType = typename DataStorageType::MaterialMapType;



    LogicModule(ParserType * p, RenderExecutionGraph * g,
                      GeometryMapType * geomMap, MaterialMapType * materials)
        :Base(p,g), m_geomMap(geomMap), m_materials(materials), m_executionGraph(g) {}

    void parse(XMLNodeType & parent) {

        XMLNodeType logicNode = parent.child("Logic");
            if(!logicNode) {
                return;
            }
        // Add all tools into the execution list!
        auto nodes = logicNode.children("Tool");
        auto itNodeEnd = nodes.end();
        for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {
           parseTool(*itNode);
        }

        Base::addAllLinks(logicNode);
    }

    void parseTool(XMLNodeType & tool){
            unsigned int id;
            if(!Utilities::stringToType(id, tool.attribute("id").value())) {
                ERRORMSG("---> String conversion in Tool: id failed");
            }
            std::string type = tool.attribute("type").value();
            LOGLPLEVEL2(this->m_pLog,"---> Parsing Tool: " << type << " with id: " << id << std::endl;);

            if(type == "MaterialLookUp") {
                createToolMaterialLookUp(tool,id);
            } else if(type == "MatteMaterial") {
                createToolMatteMaterial(tool,id);
            } else if(type == "BxdfDisneyMaterial") {
                createToolBxdfDisneyMaterial(tool,id);
            } else if(type == "ColorList") {
                createToolColorList(tool,id);
            } else if(type == "ColorGradient") {
                createToolColorGradient(tool,id);
            }else if(type == "RendermanWriter") {
                createToolRendermanOutput(tool,id);
            } else {
                // Fall through here, check in parent, for other nodes
                Base::parseTool(tool);
            }
    }

    void cleanUp() {}

private:

     #define DEFINE_MAKEColorList \
        std::string g = logicNode.attribute("generate").value(); \
        if(g=="random"){ \
            unsigned int seed; \
            if(!Utilities::stringToType(seed, logicNode.attribute("seed").value())) { \
                ERRORMSG("---> String conversion in ColorList tool: seed failed"); \
            } \
            unsigned int count; \
            if(!Utilities::stringToType(count, logicNode.attribute("count").value())) { \
                ERRORMSG("---> String conversion in ColorList tool: count failed"); \
            } \
            double amp; \
            if(!Utilities::stringToType(amp, logicNode.attribute("amp").value())) { \
                ERRORMSG("---> String conversion in ColorList tool: amp failed"); \
            } \
            if(count==0){ \
                ERRORMSG("---> String conversion in ColorList tool: count == 0") \
            } \
            n = new LogicNodes::ColorList<T>(id,count,seed,amp);\
        }else{ \
            ERRORMSG("---> String conversion in ColorList tool: generator failed"); \
        }



    #define DEFINE_ColorList2(type, typeName) \
        ( t1 == #typeName ){ using T = type; \
                DEFINE_MAKEColorList \
        } \

    #define DEFINE_ColorList(type) DEFINE_ColorList2(type,type)

    void createToolColorList(XMLNodeType & logicNode, unsigned int id){

            std::string t = logicNode.attribute("generate").value();
            if(t=="random"){
            unsigned int seed;
            if(!Utilities::stringToType(seed, logicNode.attribute("seed").value())) {
                ERRORMSG("---> String conversion in ColorList tool: seed failed");
            }
            unsigned int count;
            if(!Utilities::stringToType(count, logicNode.attribute("count").value())) {
                ERRORMSG("---> String conversion in ColorList tool: count failed");
            }
            double amp;
            if(!Utilities::stringToType(amp, logicNode.attribute("amp").value())) {
                ERRORMSG("---> String conversion in ColorList tool: amp failed");
            }
            if(count==0){
                ERRORMSG("---> String conversion in ColorList tool: count == 0")
            }
            }else{
                ERRORMSG("---> String conversion in ColorList tool: generator failed");
            }

            std::string t1 = logicNode.attribute("inputType").value();
            LogicNode * n;
            if DEFINE_ColorList(char)
            else if DEFINE_ColorList(short)
            else if DEFINE_ColorList(int)
            else if DEFINE_ColorList(long int)
            else if DEFINE_ColorList(long long int)
            else if DEFINE_ColorList(unsigned char)
            else if DEFINE_ColorList(unsigned short)
            else if DEFINE_ColorList(unsigned int)
            else if DEFINE_ColorList(unsigned long int)
            else if DEFINE_ColorList(unsigned long long int)
            else{
                ERRORMSG("---> String conversion in Constant tool: inputType: '" << t1 << "' not found!");
            }

            m_executionGraph->addNode(n,false,false);
            this->addNodeToGroup(logicNode,id);
    }

    void createToolColorGradient(XMLNodeType & logicNode, unsigned int id){


            double min;
            if(!Utilities::stringToType(min, logicNode.attribute("min").value())) {
                ERRORMSG("---> String conversion in ColorList tool: amp failed");
            }
            double max;
            if(!Utilities::stringToType(max, logicNode.attribute("max").value())) {
                ERRORMSG("---> String conversion in ColorList tool: amp failed");
            }
            if(min>=max){
                ERRORMSG("---> String conversion in ColorGradient tool: min/max not feasible!");
            }

            LogicNodes::ColorGradientNode * n = new LogicNodes::ColorGradientNode(id,min,max);

            // Add all format Sockets links
            auto nodes = logicNode.children("Color");
            auto itNodeEnd = nodes.end();
            bool noColors = true;
            for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {

                noColors = false;

                Vector3 rgb;
                if(!Utilities::stringToVector3(rgb, itNode->attribute("rgb").value())) {
                    ERRORMSG("---> String conversion in ColorGradient tool: rgb failed");
                }
                PREC value;
                if(!Utilities::stringToType(value, itNode->attribute("value").value())) {
                    ERRORMSG("---> String conversion in ColorGradient tool: value failed");
                }
                n->addColorPoint(rgb,value);
            }
            if(noColors){
                n->createDefaultHeatMapGradient();
            }

            m_executionGraph->addNode(n,false,false);
            this->addNodeToGroup(logicNode,id);
    }

    void createToolMatteMaterial(XMLNodeType & logicNode, unsigned int id) {
        auto * node = new LogicNodes::MatteMaterial(id);
        m_executionGraph->addNode(node,false,false);
        this->addNodeToGroup(logicNode,id,"Body");
    }

    void createToolBxdfDisneyMaterial(XMLNodeType & logicNode, unsigned int id) {
        auto * node = new LogicNodes::BxdfDisneyMaterial(id);
        m_executionGraph->addNode(node,false,false);
        this->addNodeToGroup(logicNode,id,"Body");
    }

    void createToolMaterialLookUp(XMLNodeType & logicNode, unsigned int id) {


        unsigned int defaultMaterialId;
        if(!Utilities::stringToType(defaultMaterialId, logicNode.attribute("defaultMaterialId").value())) {
            ERRORMSG("---> String conversion in MaterialLookUp tool: defaultMaterialId failed");
        }

        // Get default material

        auto it = m_materials->find(defaultMaterialId);


        if(it == m_materials->end()) {
            ERRORMSG("No default material found for MaterialLookUp tool!")
        }
        LOGLPLEVEL3(this->m_pLog, "Default Material set to: " << std::endl << it->second->str() << std::endl)

        std::string type = logicNode.attribute("inputType").value();
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

         m_executionGraph->addNode(node,false,false);
         this->addNodeToGroup(logicNode,id);
    }

    void createToolRendermanOutput(XMLNodeType & logicNode, unsigned int id) {
        XMLAttributeType att;
        bool pipe = false;
        std::string command ="";
        att = logicNode.attribute("pipeToSubprocess");
        if(att) {
            if(!Utilities::stringToType(pipe, att.value())) {
                ERRORMSG("---> String conversion in RendermanWriter tool: pipeToSubprocess failed");
            }

            if( pipe ) {
                command = logicNode.attribute("command").value();
                if(command.empty()) {
                    ERRORMSG("---> String conversion in RendermanWriter tool: command failed");
                }
            }

        }

        auto * node = new LogicNodes::RendermanWriter(id, m_geomMap,pipe,command);
        m_executionGraph->addNode(node,false,true);
    }

    RenderExecutionGraph * m_executionGraph;
    MaterialMapType * m_materials;
    GeometryMapType * m_geomMap;
};

};

#undef DEFINE_MAKEColorList
#undef DEFINE_ColorList2
#undef DEFINE_ColorList

#endif

