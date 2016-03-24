#ifndef GRSF_general_LogicParserModules_hpp
#define GRSF_general_LogicParserModules_hpp

#include "GRSF/common/CommonFunctions.hpp"

#include "GRSF/dynamics/general/ParserFunctions.hpp"

#include "GRSF/logic/SimpleFunction.hpp"
#include "GRSF/logic/StringFormatNode.hpp"
#include "GRSF/logic/ConstantNode.hpp"
#include "GRSF/logic/NormNode.hpp"
#include "GRSF/logic/InnerProductNode.hpp"
#include "GRSF/logic/Transform3DNode.hpp"
#include "GRSF/logic/VectorToComponentsNode.hpp"
#include "GRSF/logic/LookUpTable.hpp"
#include "GRSF/logic/LineWriter.hpp"
#include "GRSF/logic/StopNode.hpp"
#include "GRSF/logic/XMLLineWriter.hpp"

#include "GRSF/general/SimFileExecutionGraph.hpp"
#include "GRSF/general/SimFileExecutionGraphNodes.hpp"
#include "GRSF/general/LogicParserTraitsMacro.hpp"


#define ERRORMSG_PARSERTOOL(mess,id) \
    ERRORMSG( mess << " Tool id: " << id )


/** Parser for all execution graph nodes except special ones for rendering */

namespace LogicParserModules{

    template<typename TParserTraits>
    class LogicModule {
    public:
        DEFINE_LAYOUT_CONFIG_TYPES
        DEFINE_LOGICPARSER_TYPE_TRAITS(TParserTraits)

        using Base = LogicModule; // no base so far

        using ExecutionGraphType = SimFileExecutionGraph;
        using NodeGroups = typename ExecutionGraphType::NodeGroups;

        LogicModule(ParserType * p, SimFileExecutionGraph * g)
            :m_parser(p),m_executionGraph(g), m_pLog(p->getLog())
        {
            ASSERTMSG(m_executionGraph,"null")
        }


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
                LOGLPLEVEL3(m_pLog,"---> Parsing Tool with id: " << id << " type: " << type << std::endl;);
                if(type == "BodyDataInput") {
                    createToolBodyData(tool,id);
                }else if(type == "StateDataInput") {
                    createToolStateData(tool,id);
                }else if(type == "SimFileInfo") {
                    createToolSimFileInfo(tool,id);
                } else if(type == "SimpleFunction") {
                    createToolSimpleFunction(tool,id);
                } else if(type == "StringFormat") {
                    createToolStringFormat(tool,id);
                } else if(type == "LineWriter") {
                    createToolLineWriter(tool,id);
                } else if(type == "XMLLineWriter") {
                    createToolXMLLineWriter(tool,id);
                } else if(type == "DisplacementToPosQuat") {
                    createToolDisplacementToPosQuat(tool,id);
                } else if(type == "VelocityToVelRot") {
                    createToolVelocityToVelRot(tool,id);
                }else if(type == "OOBBCollider") {
                    createToolOOBBCollider(tool,id);
                }else if(type == "Norm") {
                    createToolNorm(tool,id);
                }else if(type == "Transform3D") {
                    createToolTransform3D(tool,id);
                }else if(type == "InnerProduct") {
                    createToolInnerProduct(tool,id);
                }else if(type == "VectorToComponent") {
                    createToolVectorToComponent(tool,id);
                }else if(type == "Constant"){
                    createToolConstant(tool,id);
                }else if(type == "StopNode"){
                    createToolStopNode(tool,id);
                }else {
                    ERRORMSG_PARSERTOOL("---> String conversion 'type': " << type <<" not found!", id);
                }
        }

        void cleanUp() {
        }

        void addAllLinks(XMLNodeType & logicNode){
            // Add all Getter links
            auto nodes = logicNode.children("Get");
            auto itNodeEnd = nodes.end();
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

                LOGLPLEVEL3(m_pLog,"---> Linking Tool: Get " << outNode << " socket: " << outSocket << " --from--> "
                            << fromNode << " socket: " << fromSocket <<  std::endl;);
                // Link the nodes
                m_executionGraph->makeGetLink(outNode,outSocket,fromNode,fromSocket);

            }

             // Add all Writer links
            nodes = logicNode.children("Write");
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

                LOGLPLEVEL3(m_pLog,"---> Linking Tool: Write from" << outNode << " socket: " << outSocket << " ---to---> "
                            << "to: " << toNode << " socket: "<< toSocket << std::endl;);
                // Link the nodes
                m_executionGraph->makeWriteLink(outNode,outSocket,toNode,toSocket);
            }
        }

    protected:

        /** Adds the tool to the groupId, if not specified it is added to the BODY group */
        template<bool needsResetGroup = false>
        void addNodeToGroup(XMLNodeType & logicNode, unsigned int id, std::string defaultGroupName=""){

            std::vector<std::string> l;
            /** Executable groups */
            auto att = logicNode.attribute("groupId");
            if( att ){

                if(!Utilities::stringToType(l,att.value())){
                    ERRORMSG_PARSERTOOL("String conversion 'groupId' failed", id)
                }

                for(auto s : l){
                    m_executionGraph->addNodeToGroup(id, s);
                }
            }else{
                // add to standart group in execution graph
                m_executionGraph->addNodeToGroup(id,defaultGroupName);
            }

            if(needsResetGroup){
                auto att = logicNode.attribute("resetGroupId");
                if( att ){

                    l.clear();
                    if(!Utilities::stringToType(l,att.value())){
                        ERRORMSG_PARSERTOOL("String conversion 'resetGroupId' failed", id)
                    }

                    for(auto s : l){
                        m_executionGraph->addNodeToResetGroup(id, s);
                    }

                }else{
                    ERRORMSG_PARSERTOOL("You need to specify a resetGroupId", id);
                }
            }
        }

        #define DEFINE_CONSTANT3(type, typeName, arg) \
            ( t == #typeName ){ using T = type; \
            T tt; \
            if(!Utilities::stringToType(arg, logicNode.attribute("value").value())) { \
                ERRORMSG_PARSERTOOL("---> String conversion 'value' failed", id); \
            } \
            n = new LogicNodes::ConstantNode<T>(id,tt); \
            }

        #define DEFINE_CONSTANT2(type,typeName) DEFINE_CONSTANT3(type,typeName, tt )
        #define DEFINE_CONSTANT(type) DEFINE_CONSTANT2(type,type)

        void createToolConstant(XMLNodeType & logicNode, unsigned int id){

                std::string t = logicNode.attribute("outputType").value();
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
                else if DEFINE_CONSTANT(Vector3)
                else if DEFINE_CONSTANT3(Quaternion,Quaternion, tt.coeffs() )
                else if DEFINE_CONSTANT2(std::string, string)
                else if DEFINE_CONSTANT2(boost::filesystem::path, path)
                else{
                    ERRORMSG_PARSERTOOL("---> String conversion 'outputType': '" << t << "' not found!", id);
                }

                m_executionGraph->addNode(n,false,false);

                addNodeToGroup(logicNode,id);
        }

        #define DEFINE_NORM2(type, typeName) \
            ( t == #typeName ){ using T = type; \
            n = new LogicNodes::NormNode<T>(id); \
            } \

        #define DEFINE_NORM(type) DEFINE_NORM2(type,type)
        void createToolNorm(XMLNodeType & logicNode, unsigned int id){

                std::string t = logicNode.attribute("inputType").value();
                LogicNode * n;
                if DEFINE_NORM(Vector3)
                else{
                    ERRORMSG_PARSERTOOL("---> String conversion 'outputType': '" << t << "' not found!", id);
                }

                m_executionGraph->addNode(n,false,false);

                addNodeToGroup(logicNode,id);
        }

        #define DEFINE_INNERPRODUCT2(type, typeName) \
            ( t == #typeName ){ using T = type; \
                n = new LogicNodes::InnerProduct<T>(id); \
            }

        #define DEFINE_INNERPRODUCT(type) DEFINE_INNERPRODUCT2(type,type)
        void createToolInnerProduct(XMLNodeType & logicNode, unsigned int id){

                std::string t = logicNode.attribute("inputType").value();
                LogicNode * n;
                if DEFINE_INNERPRODUCT(Vector3)
                else{
                    ERRORMSG_PARSERTOOL("---> String conversion 'outputType': '" << t << "' not found!", id);
                }

                m_executionGraph->addNode(n,false,false);

                addNodeToGroup(logicNode,id);
        }


        #define DEFINE_VECTORTOCOMPONENT2(type, typeName) \
            ( t == #typeName ){ using T = type; \
            n = new LogicNodes::VectorToComponent<T>(id); \
            }

        #define DEFINE_VECTORTOCOMPONENT(type) DEFINE_VECTORTOCOMPONENT2(type,type)
        void createToolVectorToComponent(XMLNodeType & logicNode, unsigned int id){
                ERRORMSG("Implementation not finished, parse index!")

                std::string t = logicNode.attribute("inputType").value();
                LogicNode * n;
                if DEFINE_VECTORTOCOMPONENT(Vector3)
                //else if DEFINE_VECTORTOCOMPONENT(Quaternion)
                else{
                    ERRORMSG_PARSERTOOL("---> String conversion 'outputType': '" << t << "' not found!", id);
                }

                m_executionGraph->addNode(n,false,false);

                addNodeToGroup(logicNode,id);
        }


        #define ADD_STRINGFORMAT_SOCKET2(type, typeName, _InOROut_ ) \
            ( t == #typeName ){ \
                using T = type; \
                node->add ## _InOROut_<T>(); \
            }

        #define ADD_STRINGFORMAT_SOCKET2_IN(type, typeName ) ADD_STRINGFORMAT_SOCKET2(type,typeName, Input)
        #define ADD_STRINGFORMAT_SOCKET2_OUT(type, typeName ) ADD_STRINGFORMAT_SOCKET2(type,typeName, Output)

        #define ADD_STRINGFORMAT_SOCKET_IN(type) ADD_STRINGFORMAT_SOCKET2(type,type, Input )
        #define ADD_STRINGFORMAT_SOCKET_OUT(type) ADD_STRINGFORMAT_SOCKET2(type,type, Output )

        void createToolStringFormat(XMLNodeType & logicNode, unsigned int id){

                std::string format = logicNode.attribute("format").value();
                if(format.empty()){
                    ERRORMSG_PARSERTOOL("---> String conversion 'format': not defined!", id);
                }
                LogicNodes::StringFormatNode * node = new LogicNodes::StringFormatNode(id,format);

                 // Add all format Sockets links
                for (auto & n : logicNode.children("InputFormat")) {
                    std::string t = n.attribute("type").value();

                    if ADD_STRINGFORMAT_SOCKET_IN(float)
                    else if ADD_STRINGFORMAT_SOCKET_IN(double)
                    else if ADD_STRINGFORMAT_SOCKET_IN(char)
                    else if ADD_STRINGFORMAT_SOCKET_IN(short)
                    else if ADD_STRINGFORMAT_SOCKET_IN(int)
                    else if ADD_STRINGFORMAT_SOCKET_IN(long int)
                    else if ADD_STRINGFORMAT_SOCKET_IN(long long int)
                    else if ADD_STRINGFORMAT_SOCKET_IN(unsigned char)
                    else if ADD_STRINGFORMAT_SOCKET_IN(unsigned short)
                    else if ADD_STRINGFORMAT_SOCKET_IN(unsigned int)
                    else if ADD_STRINGFORMAT_SOCKET_IN(unsigned long int)
                    else if ADD_STRINGFORMAT_SOCKET_IN(unsigned long long int)
                    else if ADD_STRINGFORMAT_SOCKET2_IN(std::string,string)
                    else if ADD_STRINGFORMAT_SOCKET2_IN(boost::filesystem::path,path)
                    else{
                        ERRORMSG_PARSERTOOL("---> String conversion 'InputFormat': '" << t << "' not found!", id);
                    }
                }
                unsigned int outs = 0;
                for (auto & n : logicNode.children("OutputFormat")) {

                    std::string t = n.attribute("type").value();

                    if ADD_STRINGFORMAT_SOCKET2_OUT(std::string,string)
                    else if ADD_STRINGFORMAT_SOCKET2_OUT(boost::filesystem::path,path)
                    else{
                        ERRORMSG_PARSERTOOL("---> String conversion 'OutputFormat': '" << t << "' not found!", id);
                    }
                    ++outs;
                }
                if(outs == 0){
                    // Add standart string output
                    node->addOutput<std::string>();
                }

                m_executionGraph->addNode(node,false,false);

                addNodeToGroup(logicNode,id);
        }


        // Node
        #define DEFINE_MAKESimpleFunc \
            auto * s = new LogicNodes::SimpleFunction<T1,T2>(id); \
            createToolSimpleFunction_parseInputs(logicNode, id, s); \
            node = s;

        #define DEFINE_SIMPLEFUNCTION_S2(type, typeName) \
            ( t2 == #typeName ){ using T2 = type; \
                    DEFINE_MAKESimpleFunc \
            }

        #define DEFINE_SIMPLEFUNCTION_S(type) DEFINE_SIMPLEFUNCTION_S2(type,type)

        #define DEFINE_SIMPLEFUNCTION2_IFPART(type) \
            using T1 = type; \
            std::string t2 = logicNode.attribute("outputType").value(); \
            if(t2.empty()){ \
                using T2 = T1;  \
                DEFINE_MAKESimpleFunc \
            }else{ \
                if      DEFINE_SIMPLEFUNCTION_S(float)  \
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
                else if DEFINE_SIMPLEFUNCTION_S(unsigned long long int) \
                else{ \
                    ERRORMSG_PARSERTOOL("---> String conversion 'outputType': '" << t2 << "' not found!", id);  \
                } \
            } \


        #define DEFINE_SIMPLEFUNCTION2(type, typeName) \
            ( t1 == #typeName ){ \
                    DEFINE_SIMPLEFUNCTION2_IFPART(type) \
            }

        #define DEFINE_SIMPLEFUNCTION(type) DEFINE_SIMPLEFUNCTION2(type,type)

        void createToolSimpleFunction(XMLNodeType & logicNode, unsigned int id){

                XMLAttributeType att;
                att = logicNode.attribute("inputs");
                unsigned int inputs = 1;
                if(att) {
                    if(!Utilities::stringToType(inputs, logicNode.attribute("inputs").value())) {
                        ERRORMSG_PARSERTOOL("---> String conversion 'inputs' failed", id);
                    }
                }
                // Make tool
                std::string t1 = logicNode.attribute("evalType").value();
                LogicNode * node;
                if      DEFINE_SIMPLEFUNCTION(float)
                else if DEFINE_SIMPLEFUNCTION(double)
                // does not work with expression parser exprtk
                //else if DEFINE_SIMPLEFUNCTION(bool)
                //else if DEFINE_SIMPLEFUNCTION(char)
//                else if DEFINE_SIMPLEFUNCTION(short)
//                else if DEFINE_SIMPLEFUNCTION(int)
//                else if DEFINE_SIMPLEFUNCTION(long int)
//                else if DEFINE_SIMPLEFUNCTION(long long int)
//                else if DEFINE_SIMPLEFUNCTION(unsigned char)
//                else if DEFINE_SIMPLEFUNCTION(unsigned short)
//                else if DEFINE_SIMPLEFUNCTION(unsigned int)
//                else if DEFINE_SIMPLEFUNCTION(unsigned long int)
//                else if DEFINE_SIMPLEFUNCTION(unsigned long long int)
                else{
                    DEFINE_SIMPLEFUNCTION2_IFPART(double)
                }

                m_executionGraph->addNode(node,false,false);

                addNodeToGroup(logicNode,id);
        }
          // Inputs
        #define ADD_SIMPLEFUNCTION_SOCKET2(type, typeName, _InOROut_ ) \
            ( t == #typeName ){ \
                using T = type; \
                simpleFNode->template add ## _InOROut_<T>(); \
            }

        #define ADD_SIMPLEFUNCTION_SOCKET2_IN(type, typeName ) ADD_SIMPLEFUNCTION_SOCKET2(type,typeName, Input)

        #define ADD_SIMPLEFUNCTION_SOCKET_IN(type) ADD_SIMPLEFUNCTION_SOCKET2(type,type, Input )

        template<typename TSimpleFunction>
        void createToolSimpleFunction_parseInputs(XMLNodeType & logicNode, unsigned int id, TSimpleFunction * simpleFNode){

            std::string expressionString = logicNode.attribute("expression").value();
            if(expressionString.empty()){
                ERRORMSG_PARSERTOOL("No expressions string in SimpleFunction", id)
            }

            // Add inputs
             // Add all format Sockets links (only arithmetic types are allowed)
            for (auto & n : logicNode.children("InputFormat")) {
                std::string t = n.attribute("type").value();

                if      ADD_SIMPLEFUNCTION_SOCKET_IN(float)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(double)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(bool)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(char)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(short)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(int)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(long int)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(long long int)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(unsigned char)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(unsigned short)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(unsigned int)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(unsigned long int)
                else if ADD_SIMPLEFUNCTION_SOCKET_IN(unsigned long long int)
//                    else if ADD_STRINGFORMAT_SOCKET2_IN(std::string,string)
//                    else if ADD_STRINGFORMAT_SOCKET2_IN(boost::filesystem::path,path)
                else{
                    ERRORMSG_PARSERTOOL("---> String conversion 'InputFormat': '" << t << "' not found!", id);
                }
            }
            simpleFNode->compileExpression(expressionString);
        }

        #define DEFINE_LINEWRITER2(type, typeName) \
            ( t == #typeName ){ \
                using T = type; \
                XMLAttributeType att; \
                bool truncate = true; \
                att = logicNode.attribute("truncate"); \
                if(att){ \
                    if(!Utilities::stringToType(truncate, att.value())) { \
                        ERRORMSG_PARSERTOOL("---> String conversion 'inputs' failed", id); \
                    }\
                }\
                std::string file;\
                att = logicNode.attribute("file");\
                if(att) {\
                    file = logicNode.attribute("file").value(); \
                    if(file.empty()){ \
                        ERRORMSG_PARSERTOOL("---> String conversion 'file': " << file << " failed!", id)\
                    }\
                }\
                n = new LogicNodes::LineWriter<T>(id,file,truncate); \
            }

        #define DEFINE_LINEWRITER(type) DEFINE_LINEWRITER2(type,type)

        void createToolLineWriter(XMLNodeType & logicNode, unsigned int id){

                std::string t = logicNode.attribute("inputType").value();

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
                    ERRORMSG_PARSERTOOL("---> String conversion 'inputType': '" << t << "' not found!", id);
                }

                m_executionGraph->addNode(n,false,true);

                addNodeToGroup(logicNode,id);

        }

        #define DEFINE_XMLLINEWRITER2(type, typeName) \
            ( t == #typeName ){ \
                using T = type; \
                XMLAttributeType att; \
                std::string rootName = "Root"; \
                att = logicNode.attribute("rootName"); \
                if(att){ \
                    rootName = att.value(); \
                }\
                std::string childName = "Child"; \
                att = logicNode.attribute("childName"); \
                if(att){ \
                    childName = att.value(); \
                }\
                std::string file;\
                att = logicNode.attribute("file");\
                if(att) {\
                    file = logicNode.attribute("file").value(); \
                    if(file.empty()){ \
                        ERRORMSG_PARSERTOOL("---> String conversion 'file': " << file << " failed!", id)\
                    }\
                }\
                n = new LogicNodes::XMLLineWriter<T>(id,file,rootName,childName); \
            }

        #define DEFINE_XMLLINEWRITER(type) DEFINE_XMLLINEWRITER2(type,type)

        void createToolXMLLineWriter(XMLNodeType & logicNode, unsigned int id){

                std::string t = logicNode.attribute("inputType").value();

                LogicNode * n;
                if DEFINE_XMLLINEWRITER(float)
                else if DEFINE_XMLLINEWRITER(double)
                else if DEFINE_XMLLINEWRITER(char)
                else if DEFINE_XMLLINEWRITER(short)
                else if DEFINE_XMLLINEWRITER(int)
                else if DEFINE_XMLLINEWRITER(long int)
                else if DEFINE_XMLLINEWRITER(long long int)
                else if DEFINE_XMLLINEWRITER(unsigned char)
                else if DEFINE_XMLLINEWRITER(unsigned short)
                else if DEFINE_XMLLINEWRITER(unsigned int)
                else if DEFINE_XMLLINEWRITER(unsigned long int)
                else if DEFINE_XMLLINEWRITER(unsigned long long int)
                else if DEFINE_XMLLINEWRITER2(std::string,string)
                else if DEFINE_XMLLINEWRITER2(boost::filesystem::path,path)
                else{
                    ERRORMSG_PARSERTOOL("---> String conversion 'inputType': " << t << "' not found!", id);
                }

                m_executionGraph->addNode(n,false,true);

                addNodeToGroup(logicNode,id);

        }



        void createToolStateData(XMLNodeType & logicNode, unsigned int id) {
            auto * node = new LogicNodes::StateData(id);
            m_executionGraph->addNode(node,true,false);
            addNodeToGroup(logicNode,id,"Frame");
            m_executionGraph->setStateData(node);
        }

        void createToolBodyData(XMLNodeType & logicNode, unsigned int id) {
            auto * node = new LogicNodes::BodyData(id);
            m_executionGraph->addNode(node,true,false);
            addNodeToGroup(logicNode,id,"Body");
            m_executionGraph->setBodyData(node);
        }

        void createToolSimFileInfo(XMLNodeType & logicNode, unsigned int id) {
            auto * node = new LogicNodes::SimFileInfo(id);
            m_executionGraph->addNode(node,true,false);
            addNodeToGroup(logicNode,id,"Frame");
            m_executionGraph->setSimFileInfo(node);
        }

        void createToolDisplacementToPosQuat(XMLNodeType & logicNode, unsigned int id) {

            auto * node = new LogicNodes::DisplacementToPosQuat(id);
            m_executionGraph->addNode(node,false,false);
            addNodeToGroup(logicNode,id,"Body");
        }

        void createToolVelocityToVelRot(XMLNodeType & logicNode, unsigned int id) {

            auto * node = new LogicNodes::VelocityToVelRot(id);
            m_executionGraph->addNode(node,false,false);
            addNodeToGroup(logicNode,id,"Body");
        }

        void createToolStopNode(XMLNodeType & logicNode, unsigned int id) {
            std::string stopGroupId;
            stopGroupId = logicNode.attribute("stopGroupId").value();

            auto it = ExecutionGraphType::m_nameToExecGroupId.find(stopGroupId);
            if(it==ExecutionGraphType::m_nameToExecGroupId.end()){
                ERRORMSG_PARSERTOOL("No group found for " << "stopGroupId: " << stopGroupId, id)
            }

            auto * node = new LogicNodes::StopNode(id);
            m_executionGraph->addNode(node,false,false);
            //addNodeToGroup(logicNode,id,"Frame"); no add to group
            m_executionGraph->setStopNode(node,it->second);
        }


        void createToolOOBBCollider(XMLNodeType & logicNode, unsigned int id) {

            Vector3 minPoint;
            if(!Utilities::stringToType(minPoint,  logicNode.attribute("minPoint").value())) {
                ERRORMSG_PARSERTOOL("---> String conversion 'minPoint' failed", id);
            }
            Vector3 maxPoint;
            if(!Utilities::stringToType(maxPoint,  logicNode.attribute("maxPoint").value())) {
                ERRORMSG_PARSERTOOL("---> String conversion 'maxPoint' failed", id);
            }
            Quaternion q_KI;
            Vector3 I_r_IK;
            ParserFunctions::parseTransformSequence(logicNode,q_KI,I_r_IK);
            Matrix33 R_KI = q_KI.toRotationMatrix();
            I_r_IK = R_KI.transpose()*I_r_IK; // make K_r_IK  = A_KI * I_r_IK
            AABB3d aabb( I_r_IK + minPoint, I_r_IK + maxPoint);

            auto * node = new LogicNodes::OOBBCollider(id,aabb,R_KI);

            LOGLPLEVEL3(m_pLog,"---> OOBBCollider with aabb: min: "
                        << aabb.m_minPoint << "max: " << aabb.m_maxPoint << " R_KI: " << std::endl << R_KI << std::endl;  )

            m_executionGraph->addNode(node,false,false);
            addNodeToGroup<true>(logicNode,id,"Body");
        }

        void createToolTransform3D(XMLNodeType & logicNode, unsigned int id) {

            Quaternion q_KI;
            Vector3 I_r_IK;
            ParserFunctions::parseTransformSequence(logicNode,q_KI,I_r_IK);
            auto * node = new LogicNodes::Transform3D(id,q_KI,I_r_IK);
            LOGLPLEVEL3(m_pLog,"---> Transform3D with  R_KI: " << std::endl << q_KI.toRotationMatrix() << std::endl << " trans: " << I_r_IK.transpose() << std::endl;  )

            m_executionGraph->addNode(node,false,false);
            addNodeToGroup(logicNode,id,"Body");
        }



        ParserType * m_parser;
        LogType * m_pLog;
        SimFileExecutionGraph * m_executionGraph;
    };

};


#undef DEFINE_CONSTANT
#undef DEFINE_CONSTANT2

#undef DEFINE_NORM
#undef DEFINE_NORM2

#undef DEFINE_VECTORTOCOMPONENT2
#undef DEFINE_VECTORTOCOMPONENT

#undef ADD_STRINGFORMAT_SOCKET2
#undef ADD_STRINGFORMAT_SOCKET
#undef ADD_STRINGFORMAT_SOCKET2_IN
#undef ADD_STRINGFORMAT_SOCKET2_OUT
#undef ADD_STRINGFORMAT_SOCKET_IN
#undef ADD_STRINGFORMAT_SOCKET_OUT

#undef DEFINE_MAKESimpleFunc
#undef DEFINE_SIMPLEFUNCTION_S2
#undef DEFINE_SIMPLEFUNCTION_S
#undef DEFINE_SIMPLEFUNCTION

#undef ADD_SIMPLEFUNCTION_SOCKET2
#undef ADD_SIMPLEFUNCTION_SOCKET2_IN
#undef ADD_SIMPLEFUNCTION_SOCKET_IN


#undef DEFINE_LINEWRITER2
#undef DEFINE_LINEWRITER

#undef DEFINE_XMLLINEWRITER2
#undef DEFINE_XMLLINEWRITER

#endif // GRSF_General_LogicParserModules_hpp
