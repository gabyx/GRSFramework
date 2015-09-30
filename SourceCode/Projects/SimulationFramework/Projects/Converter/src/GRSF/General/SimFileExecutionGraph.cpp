#include "GRSF/General/SimFileExecutionGraph.hpp"

#include "GRSF/Logic/StopNode.hpp"

#include "GRSF/General/SimFileExecutionGraphNodes.hpp"


const std::map<std::string, unsigned int > SimFileExecutionGraph::m_nameToExecGroupId = {
    {"Body", SimFileExecutionGraph::NodeGroups::BODY_EXEC}, {"Frame", SimFileExecutionGraph::NodeGroups::FRAME_EXEC}};
const std::map<std::string, unsigned int > SimFileExecutionGraph::m_nameToInitGroupId = {
    {"Body", SimFileExecutionGraph::NodeGroups::BODY_RESET}, {"Frame", SimFileExecutionGraph::NodeGroups::FRAME_RESET}};

void SimFileExecutionGraph::setup() {


    ExecutionTreeInOut::setup();

    LOG(m_log, this->getExecutionOrderInfo() );

    if( !m_bodyDataNode  ) {
        ERRORMSG("Execution tree has no input node of type 'BodyData' ")
    }
    if( !m_frameData  ) {
        ERRORMSG("Execution tree has no input node of type 'FrameData' ")
    }

}

void SimFileExecutionGraph::initFrame(boost::filesystem::path folder,
                                      std::string filename,
                                      double time,
                                      unsigned int frameNr)
{

     // Reset all nodes in FRAME_RESET group
     this->reset(NodeGroups::FRAME_RESET);

     if(filename.empty()){
        filename = "Frame";
     }
     // Set outputs in FrameData
     if(m_frameData){
        m_frameData->setOutput(folder,filename,time,frameNr);
     }else{
        ERRORMSG("There is no FrameData node present! Please add one!")
     }

     // Execute all nodes in FRAME group
     this->execute(NodeGroups::FRAME_EXEC);
}

void SimFileExecutionGraph::initSimInfo(std::size_t nBodies,std::size_t nStates){
    if(m_simFileInfo){
        m_simFileInfo->setOutput(nBodies,nStates);
    }
}

void SimFileExecutionGraph::finalizeFrame(){
     this->finalize(NodeGroups::FRAME_FINAL);
}

void SimFileExecutionGraph::addBodyState(RigidBodyStateAdd * s) {
     // Reset all nodes in BODY_RESET group
    this->reset(NodeGroups::BODY_RESET);

    // Set body data
    m_bodyDataNode->setOutputs(s);

    // Execute all nodes in BODY group
    this->execute(NodeGroups::BODY_EXEC);
    this->finalize(NodeGroups::BODY_FINAL);
}

void SimFileExecutionGraph::setStopNode(LogicNodes::StopNode *n, unsigned int stopGroupId){

    if(stopGroupId == NodeGroups::FRAME_EXEC){
        m_stopNodes[0] = n;
    }else if(stopGroupId == NodeGroups::BODY_EXEC){
        m_stopNodes[1] = n;
    }else{
        ERRORMSG("No stop node for group id:" << stopGroupId)
    }
}

bool SimFileExecutionGraph::checkStop(unsigned int groupId){
    if(groupId == NodeGroups::FRAME_EXEC && m_stopNodes[0]){
        return GET_ISOCKET_VALUE_PTR(m_stopNodes[0], Enable);
    }else if(groupId == NodeGroups::BODY_EXEC && m_stopNodes[1]){
        return GET_ISOCKET_VALUE_PTR(m_stopNodes[1], Enable);
    }
    return false;
}

void SimFileExecutionGraph::addNodeToGroup(unsigned int id, std::string groupId){
    if(groupId == "Body"){
        Base::addNodeToGroup(id, NodeGroups::BODY_EXEC);
    }
    else if(groupId == "Frame"){
        Base::addNodeToGroup(id, NodeGroups::FRAME_EXEC);
    }
    else if(groupId == "BodyFinal"){
        Base::addNodeToGroup(id, NodeGroups::BODY_FINAL);
    }
    else if(groupId == "FrameFinal"){
        Base::addNodeToGroup(id, NodeGroups::FRAME_FINAL);
    }else{
         ERRORMSG("No valid group: " << groupId << " for node id: " << id)
//        unsigned int g;
//        if(!Utilities::stringToType(g,groupId)){
//            ERRORMSG("---> String conversion for groupId: '" << groupId << "' not successful!");
//        }
//        Base::addNodeToGroup(id, g);
    }
}

void SimFileExecutionGraph::addNodeToResetGroup(unsigned int id, std::string groupId){

    if(groupId == "Body"){
         Base::addNodeToGroup(id, NodeGroups::BODY_RESET);
    }
    else if(groupId == "Frame"){
         Base::addNodeToGroup(id, NodeGroups::FRAME_RESET);
    }else{
          ERRORMSG("No valid group: " << groupId << " for node id: " << id)
//        unsigned int g;
//        if(!Utilities::stringToType(g,groupId)){
//            ERRORMSG("---> String conversion for: groupId: '" << groupId << "' not found!");
//        }
//        Base::addNodeToGroup(id, g);
    }
}
