#ifndef GRSF_General_LogicParserModules_hpp
#define GRSF_General_LogicParserModules_hpp

#include "GRSF/Common/CommonFunctions.hpp"

#include "GRSF/Dynamics/General/ParserFunctions.hpp"

#include "GRSF/Logic/SimpleFunction.hpp"
#include "GRSF/Logic/StringFormatNode.hpp"
#include "GRSF/Logic/ConstantNode.hpp"
#include "GRSF/Logic/NormNode.hpp"
#include "GRSF/Logic/LookUpTable.hpp"
#include "GRSF/Logic/LineWriter.hpp"
#include "GRSF/Logic/StopNode.hpp"
#include "GRSF/Logic/XMLLineWriter.hpp"

#include "GRSF/General/SimFileExecutionGraphNodes.hpp"
#include "GRSF/General/LogicParserTraitsMacro.hpp"

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
                LOGLPLEVEL3(m_pLog,"---> Parsing Tool with id: " << id << std::endl;);
                std::string type = tool.attribute("type").value();
                if(type == "BodyDataInput") {
                    createToolBodyData(tool,id);
                }else if(type == "FrameDataInput") {
                    createToolFrameData(tool,id);
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
                }else if(type == "Constant"){
                    createToolConstant(tool,id);
                }else if(type == "StopNode"){
                    createToolStopNode(tool,id);
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
                    ERRORMSG("String conversion fail for tool id: " << id)
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
                        ERRORMSG("String conversion fail for tool id: " << id)
                    }

                    for(auto s : l){
                        m_executionGraph->addNodeToResetGroup(id, s);
                    }

                }else{
                    ERRORMSG("You need to specify a resetGroupId for this tool id: " << id);
                }
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
                        ERRORMSG("---> String conversion in StringFormatNode tool: InputFormat: '" << t << "' not found!");
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
                        ERRORMSG("---> String conversion in XMLLineWriter tool file: " << file << " failed!")\
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
                    ERRORMSG("---> String conversion in XMLLineWriter tool: inputType: '" << t << "' not found!");
                }

                m_executionGraph->addNode(n,false,true);

                addNodeToGroup(logicNode,id);

        }



        void createToolFrameData(XMLNodeType & logicNode, unsigned int id) {
            auto * node = new LogicNodes::FrameData(id);
            m_executionGraph->addNode(node,true,false);
            addNodeToGroup(logicNode,id,"Frame");
            m_executionGraph->setFrameData(node);
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
                ERRORMSG("No group found for " << "stopGroupId: " << stopGroupId)
            }

            auto * node = new LogicNodes::StopNode(id);
            m_executionGraph->addNode(node,false,false);
            //addNodeToGroup(logicNode,id,"Frame"); no add to group
            m_executionGraph->setStopNode(node,it->second);
        }


        void createToolOOBBCollider(XMLNodeType & logicNode, unsigned int id) {

            Vector3 minPoint;
            if(!Utilities::stringToVector3(minPoint,  logicNode.attribute("minPoint").value())) {
                ERRORMSG("---> String conversion in parseMPISettings: minPoint failed");
            }
            Vector3 maxPoint;
            if(!Utilities::stringToVector3(maxPoint,  logicNode.attribute("maxPoint").value())) {
                ERRORMSG("---> String conversion in parseMPISettings: maxPoint failed");
            }
            Quaternion q_KI;
            Vector3 I_r_IK;
            ParserFunctions::parseTransformSequence(logicNode,q_KI,I_r_IK);
            Matrix33 R_KI = q_KI.toRotationMatrix();
            I_r_IK = R_KI.transpose()*I_r_IK; // make K_r_IK  = A_KI * I_r_IK
            AABB3d aabb( I_r_IK + minPoint, I_r_IK + maxPoint);

            auto * node = new LogicNodes::OOBBCollider(id,aabb,R_KI);

            m_executionGraph->addNode(node,false,false);
            addNodeToGroup<true>(logicNode,id,"Body");
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

#undef DEFINE_XMLLINEWRITER2
#undef DEFINE_XMLLINEWRITER

#endif // GRSF_General_LogicParserModules_hpp
