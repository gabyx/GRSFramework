#ifndef GRSF_General_LogicParserModules_hpp
#define GRSF_General_LogicParserModules_hpp


#include "GRSF/Logic/SimpleFunction.hpp"
#include "GRSF/Logic/StringFormatNode.hpp"
#include "GRSF/Logic/ConstantNode.hpp"
#include "GRSF/Logic/NormNode.hpp"
#include "GRSF/Logic/LookUpTable.hpp"
#include "GRSF/Logic/LineWriter.hpp"


#include "GRSF/General/SimFileExecutionGraphLogic.hpp"

namespace LogicParserModules{

    template<typename TParserTraits>
    class LogicModule {
    public:
        DEFINE_LAYOUT_CONFIG_TYPES
        DEFINE_RENDERLOGICPARSER_TYPE_TRAITS(TParserTraits)

        using Base = LogicModule; // no base so far

        using ExecGroups = typename SimFileExecutionGraph::ExecGroups;

        LogicModule(ParserType * p, SimFileExecutionGraph * g)
            :m_parser(p),m_executionGraph(g), m_pLog(p->getLog())
        {}


        void parse(XMLNodeType & parent) {

            XMLNodeType logicNode = parent.child("Materials");
            if(!logicNode) {
                return;
            }

            // Add all tools into the execution list!
            auto nodes = logicNode.children("Tool");
            auto itNodeEnd = nodes.end();
            for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {
                parseTool(*itNode);
            }

            Base::addAllLinks();

        }


        void parseTool(XMLNodeType & tool){
                unsigned int id;
                if(!Utilities::stringToType(id, tool.attribute("id").value())) {
                    ERRORMSG("---> String conversion in Tool: id failed");
                }
                LOGMCLEVEL3(m_pLog,"---> Parsing Tool with id: " << id << std::endl;);
                std::string type = tool.attribute("type").value();
                if(type == "BodyDataInput") {
                    createToolBodyData(tool,id);
                }else if(type == "FrameDataInput") {
                    createToolFrameData(tool,id);
                } else if(type == "SimpleFunction") {
                    createToolSimpleFunction(tool,id);
                } else if(type == "StringFormat") {
                    createToolStringFormat(tool,id);
                } else if(type == "LineWriter") {
                    createToolLineWriter(tool,id);
                } else if(type == "DisplacementToPosQuat") {
                    createToolDisplacementToPosQuat(tool,id);
                } else if(type == "VelocityToVelRot") {
                    createToolVelocityToVelRot(tool,id);
                }else if(type == "Norm") {
                    createToolNorm(tool,id);
                }else if(type == "Constant"){
                    createToolConstant(tool,id);
                }else {
                    ERRORMSG("---> String conversion in Tool: type: " << type <<" not found!");
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

                LOGMCLEVEL3(m_pLog,"---> Linking Tool: Get " << outNode << " socket: " << outSocket << " --from--> "
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

                LOGMCLEVEL3(m_pLog,"---> Linking Tool: Write from" << outNode << " socket: " << outSocket << " ---to---> "
                            << "to: " << toNode << " socket: "<< toSocket << std::endl;);
                // Link the nodes
                m_executionGraph->makeWriteLink(outNode,outSocket,toNode,toSocket);
            }
        }

    protected:

        /** Adds the tool to the groupId, if not specified it is added to the BODY group */
        void addNodeToGroup(XMLNodeType & logicNode, unsigned int id){
            auto att = logicNode.attribute("groupId");
            if( att ){
                std::string gid = att.value();
                if(gid == "Body"){
                    m_executionGraph->addNodeToGroup(id, ExecGroups::BODY);
                }else if(gid == "Frame"){
                    m_executionGraph->addNodeToGroup(id, ExecGroups::FRAME);
                }else{
                    ERRORMSG("---> String conversion in Constant tool: groupId: '" << gid << "' not found!");
                }
            }else{
                m_executionGraph->addNodeToGroup(id, ExecGroups::BODY);
            }
        }

        #define DEFINE_CONSTANT2(type, typeName) \
            ( t == #typeName ){ using T = type; \
            T tt; \
            if(!Utilities::stringToType(tt, logicNode.attribute("value").value())) { \
                ERRORMSG("---> String conversion in Constant tool: value failed"); \
            } \
            n = new LogicNodes::ConstantNode<T>(id,tt); \
            } \

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
                else if ( t == "string" ){
                    using T = std::string;
                    T tt = logicNode.attribute("value").value();
                    if(tt.empty()){
                        ERRORMSG("---> String conversion in Constant tool: value failed"); \
                    }
                    n = new LogicNodes::ConstantNode<T>(id,tt);
                }
                else if ( t == "path" ){
                    using T = boost::filesystem::path;
                    std::string tt = logicNode.attribute("value").value();
                    if(tt.empty()){
                        ERRORMSG("---> String conversion in Constant tool: value failed"); \
                    }
                    n = new LogicNodes::ConstantNode<T>(id,tt);
                }
                else{
                    ERRORMSG("---> String conversion in Constant tool: outputType: '" << t << "' not found!");
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
                    ERRORMSG("---> String conversion in Constant tool: outputType: '" << t << "' not found!");
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

        #define ADD_STRINGFORMAT_SOCKET(type, _InOROut_ ) ADD_STRINGFORMAT_SOCKET2(type,type, _InOROut_ )
        #define ADD_STRINGFORMAT_SOCKET_IN(type) ADD_STRINGFORMAT_SOCKET2(type,type, Input )
        #define ADD_STRINGFORMAT_SOCKET_OUT(type) ADD_STRINGFORMAT_SOCKET2(type,type, Output )

        void createToolStringFormat(XMLNodeType & logicNode, unsigned int id){

                std::string format = logicNode.attribute("format").value();
                if(format.empty()){
                    ERRORMSG("---> String conversion in StringFormat tool: format: not defined!");
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
                        ERRORMSG("---> String conversion in Constant tool: InputFormat: '" << t << "' not found!");
                    }
                }
                unsigned int outs = 0;
                for (auto & n : logicNode.children("OutputFormat")) {

                    std::string t = n.attribute("type").value();

                    if ADD_STRINGFORMAT_SOCKET2_OUT(std::string,string)
                    else if ADD_STRINGFORMAT_SOCKET2_OUT(boost::filesystem::path,path)
                    else{
                        ERRORMSG("---> String conversion in Constant tool: OutputFormat: '" << t << "' not found!");
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

        #define DEFINE_MAKESimpleFunc \
            n = new LogicNodes::SimpleFunction<T1,T2>(id,inputs,expressionString);


        #define DEFINE_SIMPLEFUNCTION_S2(type, typeName) \
            ( t2 == #typeName ){ using T2 = type; \
                    DEFINE_MAKESimpleFunc \
            }

        #define DEFINE_SIMPLEFUNCTION_S(type) DEFINE_SIMPLEFUNCTION_S2(type,type)

        #define DEFINE_SIMPLEFUNCTION2(type, typeName) \
            ( t1 == #typeName ){ using T1 = type; \
                    std::string t2 = logicNode.attribute("outputType").value(); \
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

        void createToolSimpleFunction(XMLNodeType & logicNode, unsigned int id){

                XMLAttributeType att;
                att = logicNode.attribute("inputs");
                unsigned int inputs = 1;
                if(att) {
                    if(!Utilities::stringToType(inputs, logicNode.attribute("inputs").value())) {
                        ERRORMSG("---> String conversion in SimpleFunction tool: inputs failed");
                    }
                }

                std::string expressionString = logicNode.attribute("expression").value();

                std::string t1 = logicNode.attribute("inputType").value();
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

                m_executionGraph->addNode(n,false,false);

                addNodeToGroup(logicNode,id);
        }

        #define DEFINE_LINEWRITER2(type, typeName) \
            ( t == #typeName ){ \
                using T = type; \
                XMLAttributeType att; \
                bool truncate = true; \
                att = logicNode.attribute("truncate"); \
                if(att){ \
                    if(!Utilities::stringToType(truncate, att.value())) { \
                        ERRORMSG("---> String conversion in LineWriter tool: inputs failed"); \
                    }\
                }\
                std::string file;\
                att = logicNode.attribute("file");\
                if(att) {\
                    file = logicNode.attribute("file").value(); \
                    if(file.empty()){ \
                        ERRORMSG("---> String conversion in LineWriter tool file: " << file << " failed!")\
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
                    ERRORMSG("---> String conversion in Constant tool: inputType: '" << t << "' not found!");
                }

                m_executionGraph->addNode(n,false,false);

                addNodeToGroup(logicNode,id);

        }


        void createToolFrameData(XMLNodeType & logicNode, unsigned int id) {
            auto * node = new LogicNodes::FrameData(id);
            m_executionGraph->addNode(node,true,false);
            m_executionGraph->addNodeToGroup(id,ExecGroups::FRAME);
            m_executionGraph->setFrameData(node);
        }

        void createToolBodyData(XMLNodeType & logicNode, unsigned int id) {
            auto * node = new LogicNodes::BodyData(id);
            m_executionGraph->addNode(node,true,false);
            m_executionGraph->addNodeToGroup(id,ExecGroups::BODY);
            m_executionGraph->setBodyData(node);
        }

        void createToolDisplacementToPosQuat(XMLNodeType & logicNode, unsigned int id) {

            auto * node = new LogicNodes::DisplacementToPosQuat(id);
            m_executionGraph->addNode(node,false,false);
            m_executionGraph->addNodeToGroup(id,ExecGroups::BODY);

        }

        void createToolVelocityToVelRot(XMLNodeType & logicNode, unsigned int id) {

            auto * node = new LogicNodes::VelocityToVelRot(id);
            m_executionGraph->addNode(node,false,false);
            m_executionGraph->addNodeToGroup(id,ExecGroups::BODY);

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

#undef DEFINE_LINEWRITER2
#undef DEFINE_LINEWRITER

#endif // GRSF_General_LogicParserModules_hpp
