#ifndef RenderScriptParserModules_hpp
#define RenderScriptParserModules_hpp


#include <vector>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/General/RenderScriptParserBaseTraits.hpp"

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

};


#include "GRSF/Logic/SimpleFunction.hpp"
#include "GRSF/Logic/StringFormatNode.hpp"
#include "GRSF/Logic/ConstantNode.hpp"
#include "GRSF/Logic/NormNode.hpp"
#include "GRSF/Logic/LookUpTable.hpp"
#include "GRSF/Logic/LineWriter.hpp"
#include "GRSF/General/RenderScriptGeneratorLogic.hpp"
#include "GRSF/General/RenderScriptGenerator.hpp"

namespace RenderScriptParserModules {

template<typename TParserTraits>
class ScriptGeneratorModule {
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    DEFINE_MATCOLPARSER_TYPE_TRAITS(TParserTraits)

    using GeometryMapType = typename CollectionType::GeometryMapType;
    using MaterialMapType = typename CollectionType::MaterialMapType;
    using RenderScriptGen = RenderScriptGenerator;

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
            } else if(type == "MatteMaterial") {
                createToolMatteMaterial(*itNode,id);
            } else if(type == "ColorList") {
                createToolColorList(*itNode,id);
            } else if(type == "ColorGradient") {
                createToolColorGradient(*itNode,id);
            } else if(type == "SimpleFunction") {
                createToolSimpleFunction(*itNode,id);
            } else if(type == "StringFormat") {
                createToolStringFormat(*itNode,id);
            } else if(type == "LineWriter") {
                createToolLineWriter(*itNode,id);
            } else if(type == "DisplacementToPosQuat") {
                createToolDisplacementToPosQuat(*itNode,id);
            } else if(type == "VelocityToVelRot") {
                createToolVelocityToVelRot(*itNode,id);
            }else if(type == "Norm") {
                createToolNorm(*itNode,id);
            }else if(type == "Constant"){
                createToolConstant(*itNode,id);
            } else if(type == "RendermanWriter") {
                createToolRendermanOutput(*itNode,id);
            } else {
                ERRORMSG("---> String conversion in Tool: type: " << type <<" not found!");
            }


        }


        // Add all Getter links
        nodes = matGenNode.children("Get");
        itNodeEnd = nodes.end();
        for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {

            unsigned int outNode;
            if(!Utilities::stringToType(outNode, itNode->attribute("outNode").value())) {
                ERRORMSG("---> String conversion in Get: outNode failed");
            }
            unsigned int outSocket;
            if(!Utilities::stringToType(outSocket, itNode->attribute("outSocket").value())) {
                ERRORMSG("---> String conversion in Get: outSocket failed");
            }

            unsigned int fromNode;
            if(!Utilities::stringToType(fromNode, itNode->attribute("fromNode").value())) {
                ERRORMSG("---> String conversion in Get: fromNode failed");
            }
            unsigned int fromSocket;
            if(!Utilities::stringToType(fromSocket, itNode->attribute("fromSocket").value())) {
                ERRORMSG("---> String conversion in Get: fromSocket failed");
            }

            LOGMCLEVEL3(m_pLog,"---> Linking Tool: Get " << outNode << " socket: " << outSocket << " --from--> "
                        << fromNode << " socket: " << fromSocket <<  std::endl;);
            // Link the nodes
            m_renderScriptGen->makeGetLink(outNode,outSocket,fromNode,fromSocket);


        }

         // Add all Writer links
        nodes = matGenNode.children("Write");
        itNodeEnd = nodes.end();
        for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {

            unsigned int outNode;
            if(!Utilities::stringToType(outNode, itNode->attribute("outNode").value())) {
                ERRORMSG("---> String conversion in Write: outNode failed");
            }
            unsigned int outSocket;
            if(!Utilities::stringToType(outSocket, itNode->attribute("outSocket").value())) {
                ERRORMSG("---> String conversion in Write: outSocket failed");
            }

            unsigned int toNode;
            if(!Utilities::stringToType(toNode, itNode->attribute("toNode").value())) {
                ERRORMSG("---> String conversion in Write: toNode failed");
            }
            unsigned int toSocket;
            if(!Utilities::stringToType(toSocket, itNode->attribute("toSocket").value())) {
                ERRORMSG("---> String conversion in Write: toSocket failed");
            }

            LOGMCLEVEL3(m_pLog,"---> Linking Tool: Write from" << outNode << " socket: " << outSocket << " ---to---> "
                        << "to: " << toNode << " socket: "<< toSocket << std::endl;);
            // Link the nodes
            m_renderScriptGen->makeWriteLink(outNode,outSocket,toNode,toSocket);

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

                std::string gid = matGenNode.attribute("groupId").value();
                if(gid == "Body"){
                    m_renderScriptGen->addNodeToGroup(id, ExecGroups::BODY);
                }else if(gid == "Frame"){
                    m_renderScriptGen->addNodeToGroup(id, ExecGroups::FRAME);
                }else{
                    ERRORMSG("---> String conversion in Constant tool: groupId: '" << gid << "' not found!");
                }
    }

    #define DEFINE_NORM2(type, typeName) \
        ( t == #typeName ){ using T = type; \
        n = new LogicNodes::NormNode<T>(id); \
        } \

    #define DEFINE_NORM(type) DEFINE_NORM2(type,type)
    void createToolNorm(XMLNodeType & matGenNode, unsigned int id){

            std::string t = matGenNode.attribute("inputType").value();
            LogicNode * n;
            if DEFINE_NORM(Vector3)
            else{
                ERRORMSG("---> String conversion in Constant tool: outputType: '" << t << "' not found!");
            }

            m_renderScriptGen->addNode(n,false,false);

            std::string gid = matGenNode.attribute("groupId").value();
            if(gid == "Body"){
                m_renderScriptGen->addNodeToGroup(id, ExecGroups::BODY);
            }else if(gid == "Frame"){
                m_renderScriptGen->addNodeToGroup(id, ExecGroups::FRAME);
            }else{
                ERRORMSG("---> String conversion in Constant tool: groupId: '" << gid << "' not found!");
            }
    }

    #define ADD_STRINGFORMAT_SOCKET2(type, typeName) \
        ( t == #typeName ){ \
            using T = type; \
            node->addInput<T>(); \
        } \

    #define ADD_STRINGFORMAT_SOCKET(type) ADD_STRINGFORMAT_SOCKET2(type,type)

    void createToolStringFormat(XMLNodeType & matGenNode, unsigned int id){

            std::string format = matGenNode.attribute("format").value();
            if(format.empty()){
                ERRORMSG("---> String conversion in StringFormat tool: format: not defined!");
            }
            LogicNodes::StringFormatNode * node = new LogicNodes::StringFormatNode(id,format);

             // Add all format Sockets links
            auto nodes = matGenNode.children("InputFormat");
            auto itNodeEnd = nodes.end();
            for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {


                std::string t = itNode->attribute("type").value();

                if ADD_STRINGFORMAT_SOCKET(float)
                else if ADD_STRINGFORMAT_SOCKET(double)
                else if ADD_STRINGFORMAT_SOCKET(char)
                else if ADD_STRINGFORMAT_SOCKET(short)
                else if ADD_STRINGFORMAT_SOCKET(int)
                else if ADD_STRINGFORMAT_SOCKET(long int)
                else if ADD_STRINGFORMAT_SOCKET(long long int)
                else if ADD_STRINGFORMAT_SOCKET(unsigned char)
                else if ADD_STRINGFORMAT_SOCKET(unsigned short)
                else if ADD_STRINGFORMAT_SOCKET(unsigned int)
                else if ADD_STRINGFORMAT_SOCKET(unsigned long int)
                else if ADD_STRINGFORMAT_SOCKET(unsigned long long int)
                else if ADD_STRINGFORMAT_SOCKET2(std::string,string)
                else if ADD_STRINGFORMAT_SOCKET2(boost::filesystem::path,path)
                else{
                    ERRORMSG("---> String conversion in Constant tool: outputType: '" << t << "' not found!");
                }
            }

            m_renderScriptGen->addNode(node,false,false);

                std::string gid = matGenNode.attribute("groupId").value();
                if(gid == "Body"){
                    m_renderScriptGen->addNodeToGroup(id, ExecGroups::BODY);
                }else if(gid == "Frame"){
                    m_renderScriptGen->addNodeToGroup(id, ExecGroups::FRAME);
                }else{
                    ERRORMSG("---> String conversion in Constant tool: groupId: '" << gid << "' not found!");
                }
    }

    #define DEFINE_MAKESimpleFunc \
        n = new LogicNodes::SimpleFunction<T1,T2>(id,inputs,expressionString);


    #define DEFINE_SIMPLEFUNCTION_S2(type, typeName) \
        ( t2 == #typeName ){ using T2 = type; \
                DEFINE_MAKESimpleFunc \
        }

    #define DEFINE_SIMPLEFUNCTION_S(type) DEFINE_SIMPLEFUNCTION_S2(type,type)

    #define DEFINE_SIMPLEFUNCTION2(type, typeName) \
        ( t1 == #typeName ){ using T1 = type; \
                std::string t2 = matGenNode.attribute("outputType").value(); \
                if(t2.empty()){ \
                    using T2 = T1;  \
                    DEFINE_MAKESimpleFunc \
                } \
                if DEFINE_SIMPLEFUNCTION_S(float)  \
                else if DEFINE_SIMPLEFUNCTION_S(double) \
                else if DEFINE_SIMPLEFUNCTION_S(bool) \
                else if DEFINE_SIMPLEFUNCTION_S(char) \
                else if DEFINE_SIMPLEFUNCTION_S(short) \
                else if DEFINE_SIMPLEFUNCTION_S(int) \
                else if DEFINE_SIMPLEFUNCTION_S(long int) \
                else if DEFINE_SIMPLEFUNCTION_S(unsigned char) \
                else if DEFINE_SIMPLEFUNCTION_S(unsigned short) \
                else if DEFINE_SIMPLEFUNCTION_S(unsigned int) \
                else if DEFINE_SIMPLEFUNCTION_S(unsigned long int) \
                else{ \
                    ERRORMSG("---> String conversion in SimpleFunction tool: outputType: '" << t2 << "' not found!");  \
                } \
        }

    #define DEFINE_SIMPLEFUNCTION(type) DEFINE_SIMPLEFUNCTION2(type,type)

    void createToolSimpleFunction(XMLNodeType & matGenNode, unsigned int id){

            XMLAttributeType att;
            att = matGenNode.attribute("inputs");
            unsigned int inputs = 1;
            if(att) {
                if(!Utilities::stringToType(inputs, matGenNode.attribute("inputs").value())) {
                    ERRORMSG("---> String conversion in tool: inputs failed");
                }
            }

            std::string expressionString = matGenNode.attribute("expression").value();

            std::string t1 = matGenNode.attribute("inputType").value();
            LogicNode * n;
            if DEFINE_SIMPLEFUNCTION(float)
            else if DEFINE_SIMPLEFUNCTION(double)
            else if DEFINE_SIMPLEFUNCTION(bool)
            else if DEFINE_SIMPLEFUNCTION(char)
            else if DEFINE_SIMPLEFUNCTION(short)
            else if DEFINE_SIMPLEFUNCTION(int)
            else if DEFINE_SIMPLEFUNCTION(long int)
            //else if DEFINE_SIMPLEFUNCTION(long long int)
            else if DEFINE_SIMPLEFUNCTION(unsigned char)
            else if DEFINE_SIMPLEFUNCTION(unsigned short)
            else if DEFINE_SIMPLEFUNCTION(unsigned int)
            else if DEFINE_SIMPLEFUNCTION(unsigned long int)
            //else if DEFINE_SIMPLEFUNCTION(unsigned long long int)
            else{
                ERRORMSG("---> String conversion in SimpleFunction tool: inputType: '" << t1 << "' not found!");
            }

            m_renderScriptGen->addNode(n,false,false);


                std::string gid = matGenNode.attribute("groupId").value();
                if(gid == "Body"){
                    m_renderScriptGen->addNodeToGroup(id, ExecGroups::BODY);
                }else if(gid == "Frame"){
                    m_renderScriptGen->addNodeToGroup(id, ExecGroups::FRAME);
                }else{
                    ERRORMSG("---> String conversion in SimpleFunction tool: groupId: '" << gid << "' not found!");
                }
    }

    #define DEFINE_LINEWRITER2(type, typeName) \
        ( t == #typeName ){ using T = type; \
            std::string file = matGenNode.attribute("file").value(); \
            if(file.empty()){ \
                    ERRORMSG("---> String conversion in LineWriter file: " << file << " failed!")\
            }\
            n = new LogicNodes::LineWriter<T>(id,file); \
        } \

    #define DEFINE_LINEWRITER(type) DEFINE_LINEWRITER2(type,type)

    void createToolLineWriter(XMLNodeType & matGenNode, unsigned int id){

            std::string t = matGenNode.attribute("inputType").value();

            LogicNode * n;
            if DEFINE_LINEWRITER(float)
            else if DEFINE_LINEWRITER(double)
            else if DEFINE_LINEWRITER(char)
            else if DEFINE_LINEWRITER(short)
            else if DEFINE_LINEWRITER(int)
            else if DEFINE_LINEWRITER(long int)
            else if DEFINE_LINEWRITER(long long int)
            else if DEFINE_LINEWRITER(unsigned char)
            else if DEFINE_LINEWRITER(unsigned short)
            else if DEFINE_LINEWRITER(unsigned int)
            else if DEFINE_LINEWRITER(unsigned long int)
            else if DEFINE_LINEWRITER(unsigned long long int)
            else if DEFINE_LINEWRITER2(std::string,string)
            else if DEFINE_LINEWRITER2(boost::filesystem::path,path)
            else{
                ERRORMSG("---> String conversion in Constant tool: inputType: '" << t << "' not found!");
            }

            m_renderScriptGen->addNode(n,false,false);

            std::string gid = matGenNode.attribute("groupId").value();
            if(gid == "Body"){
                m_renderScriptGen->addNodeToGroup(id, ExecGroups::BODY);
            }else if(gid == "Frame"){
                m_renderScriptGen->addNodeToGroup(id, ExecGroups::FRAME);
            }else{
                ERRORMSG("---> String conversion in Constant tool: groupId: '" << gid << "' not found!");
            }
    }


     #define DEFINE_MAKEColorList \
        std::string g = matGenNode.attribute("generate").value(); \
        if(g=="random"){ \
            unsigned int seed; \
            if(!Utilities::stringToType(seed, matGenNode.attribute("seed").value())) { \
                ERRORMSG("---> String conversion in ColorList tool: seed failed"); \
            } \
            unsigned int count; \
            if(!Utilities::stringToType(count, matGenNode.attribute("count").value())) { \
                ERRORMSG("---> String conversion in ColorList tool: count failed"); \
            } \
            double amp; \
            if(!Utilities::stringToType(amp, matGenNode.attribute("amp").value())) { \
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

    void createToolColorList(XMLNodeType & matGenNode, unsigned int id){

            std::string t = matGenNode.attribute("generate").value();
            if(t=="random"){
            unsigned int seed;
            if(!Utilities::stringToType(seed, matGenNode.attribute("seed").value())) {
                ERRORMSG("---> String conversion in ColorList tool: seed failed");
            }
            unsigned int count;
            if(!Utilities::stringToType(count, matGenNode.attribute("count").value())) {
                ERRORMSG("---> String conversion in ColorList tool: count failed");
            }
            double amp;
            if(!Utilities::stringToType(amp, matGenNode.attribute("amp").value())) {
                ERRORMSG("---> String conversion in ColorList tool: amp failed");
            }
            if(count==0){
                ERRORMSG("---> String conversion in ColorList tool: count == 0")
            }
            }else{
                ERRORMSG("---> String conversion in ColorList tool: generator failed");
            }

            std::string t1 = matGenNode.attribute("inputType").value();
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

            m_renderScriptGen->addNode(n,false,false);
            m_renderScriptGen->addNodeToGroup(id, ExecGroups::BODY);
    }

    void createToolColorGradient(XMLNodeType & matGenNode, unsigned int id){


            double min;
            if(!Utilities::stringToType(min, matGenNode.attribute("min").value())) {
                ERRORMSG("---> String conversion in ColorList tool: amp failed");
            }
            double max;
            if(!Utilities::stringToType(max, matGenNode.attribute("max").value())) {
                ERRORMSG("---> String conversion in ColorList tool: amp failed");
            }
            if(min>=max){
                ERRORMSG("---> String conversion in ColorGradient tool: min/max not feasible!");
            }

            LogicNodes::ColorGradientNode * n = new LogicNodes::ColorGradientNode(id,min,max);

            // Add all format Sockets links
            auto nodes = matGenNode.children("Color");
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

            m_renderScriptGen->addNode(n,false,false);
            m_renderScriptGen->addNodeToGroup(id, ExecGroups::BODY);
    }


    void createToolMatteMaterial(XMLNodeType & matGenNode, unsigned int id) {
        auto * node = new LogicNodes::MatteMaterial(id);
        m_renderScriptGen->addNode(node,false,false);
        m_renderScriptGen->addNodeToGroup(id,ExecGroups::BODY);
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

    void createToolVelocityToVelRot(XMLNodeType & matGenNode, unsigned int id) {

        auto * node = new LogicNodes::VelocityToVelRot(id);
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

#undef DEFINE_CONSTANT
#undef DEFINE_CONSTANT2

#undef DEFINE_NORM
#undef DEFINE_NORM2

#undef ADD_STRINGFORMAT_SOCKET2
#undef ADD_STRINGFORMAT_SOCKET

#undef DEFINE_MAKESimpleFunc
#undef DEFINE_SIMPLEFUNCTION_S2
#undef DEFINE_SIMPLEFUNCTION_S
#undef DEFINE_SIMPLEFUNCTION

#undef DEFINE_LINEWRITER2
#undef DEFINE_LINEWRITER


#undef DEFINE_MAKEColorList
#undef DEFINE_ColorList2
#undef DEFINE_ColorList

#endif

