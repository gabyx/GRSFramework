#include "GRSF/General/SimFileExecutionGraph.hpp"

#include "GRSF/Logic/StopNode.hpp"

#include "GRSF/General/SimFileExecutionGraphLogic.hpp"

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
     this->execute(ExecGroups::FRAME);
}

void SimFileExecutionGraph::finalizeFrame(){
     // Nothing to do here
}

void SimFileExecutionGraph::generateFrameData(RigidBodyStateAdd * s) {
    // Set body data
    m_bodyDataNode->setOutputs(s);

    // Execute all nodes in BODY group
    this->execute(ExecGroups::BODY);
}

bool SimFileExecutionGraph::checkStop(){
     if(m_stopNode){
        return GET_ISOCKET_VALUE_PTR(m_stopNode, Enable);
     }
     return false;
}

