#include "GRSF/General/SimFileExecutionGraph.hpp"

#include "GRSF/Logic/StopNode.hpp"

#include "GRSF/General/SimFileExecutionGraphNodes.hpp"

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

     // Reset all nodes in FRAME_INIT group
     this->reset(NodeGroups::FRAME_INIT);

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

void SimFileExecutionGraph::finalizeFrame(){
     // Nothing to do here
}

void SimFileExecutionGraph::generateFrameData(RigidBodyStateAdd * s) {
     // Reset all nodes in FRAME_INIT group
    this->reset(NodeGroups::BODY_INIT);

    // Set body data
    m_bodyDataNode->setOutputs(s);

    // Execute all nodes in BODY group
    this->execute(NodeGroups::BODY_EXEC);
}

void SimFileExecutionGraph::setStopNode(LogicNodes::StopNode *n, unsigned int stopGroupId){
    if(stopGroupId < m_stopNodes.size()){
        m_stopNodes[stopGroupId] = n;
    }
}

bool SimFileExecutionGraph::checkStop(unsigned int groupId){
     if(groupId < m_stopNodes.size() && m_stopNodes[groupId] ){
        return GET_ISOCKET_VALUE_PTR(m_stopNodes[groupId], Enable);
     }
     return false;
}

