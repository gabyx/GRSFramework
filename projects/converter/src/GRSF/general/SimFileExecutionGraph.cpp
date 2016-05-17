// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/general/SimFileExecutionGraph.hpp"

#include "GRSF/logic/StopNode.hpp"

#include "GRSF/general/SimFileExecutionGraphNodes.hpp"


const std::map<std::string, unsigned int > SimFileExecutionGraph::m_nameToExecGroupId = {
    {"Body", SimFileExecutionGraph::NodeGroups::BODY_EXEC},
    {"Frame", SimFileExecutionGraph::NodeGroups::FRAME_EXEC},
    {"File", SimFileExecutionGraph::NodeGroups::FILE_EXEC}
};
const std::map<std::string, unsigned int > SimFileExecutionGraph::m_nameToInitGroupId = {
    {"Body", SimFileExecutionGraph::NodeGroups::BODY_RESET},
    {"Frame", SimFileExecutionGraph::NodeGroups::FRAME_RESET},
    {"File", SimFileExecutionGraph::NodeGroups::FILE_RESET}
};

void SimFileExecutionGraph::setup() {


    ExecutionTreeInOut::setup();

    LOG(m_log, this->getExecutionOrderInfo() );

    if( !m_bodyDataNode  ) {
        GRSF_ERRORMSG("Execution tree has no input node of type 'BodyData' ")
    }
    if( !m_stateData  ) {
        GRSF_ERRORMSG("Execution tree has no input node of type 'FrameData' ")
    }

}

void SimFileExecutionGraph::initState(boost::filesystem::path outputfilePath,
                                      double time, unsigned int stateNr)
{
         // Reset all nodes in FRAME_RESET group
         this->reset(NodeGroups::FRAME_RESET);

         // Set outputs in FrameData
         if(m_stateData){
            m_stateData->setOutput(outputfilePath,time,stateNr);
         }else{
            GRSF_ERRORMSG("There is no FrameData node present! Please add one!")
         }

         // Execute all nodes in FRAME group
         this->execute(NodeGroups::FRAME_EXEC);
}

void SimFileExecutionGraph::initSimInfo(boost::filesystem::path simFile,
                                        boost::filesystem::path outputfilePath,
                                        std::size_t nBodies,
                                        std::size_t nStates){
    this->reset(NodeGroups::FILE_RESET);

    if(m_simFileInfo){
        m_simFileInfo->setOutput(simFile,outputfilePath,nBodies,nStates);
    }

    this->execute(NodeGroups::FILE_EXEC);

}

void SimFileExecutionGraph::finalizeState(){
     this->execute(NodeGroups::FRAME_FINAL);
}

void SimFileExecutionGraph::addBodyState(RigidBodyStateAdd * s) {
     // Reset all nodes in BODY_RESET group
    this->reset(NodeGroups::BODY_RESET);

    // Set body data
    m_bodyDataNode->setOutputs(s);


    // Execute all nodes in BODY group
    this->execute(NodeGroups::BODY_EXEC);
    this->execute(NodeGroups::BODY_FINAL);
}

void SimFileExecutionGraph::setStopNode(LogicNodes::StopNode *n, unsigned int stopGroupId){
    if(stopGroupId == NodeGroups::FILE_EXEC){
        m_stopNodes[0] = n;
    }else if(stopGroupId == NodeGroups::FRAME_EXEC){
        m_stopNodes[1] = n;
    }else if(stopGroupId == NodeGroups::BODY_EXEC){
        m_stopNodes[2] = n;
    }else{
        GRSF_ERRORMSG("No stop node for group id:" << stopGroupId)
    }
}

bool SimFileExecutionGraph::checkStop(unsigned int groupId){
    if(groupId == NodeGroups::FILE_EXEC && m_stopNodes[0]){
        return GET_ISOCKET_VALUE_PTR(m_stopNodes[0], Enable);
    }else if(groupId == NodeGroups::FRAME_EXEC && m_stopNodes[1]){
        return GET_ISOCKET_VALUE_PTR(m_stopNodes[1], Enable);
    }else if(groupId == NodeGroups::BODY_EXEC && m_stopNodes[2]){
        return GET_ISOCKET_VALUE_PTR(m_stopNodes[2], Enable);
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
    else if(groupId == "File"){
        Base::addNodeToGroup(id, NodeGroups::FILE_EXEC);
    }
    else if(groupId == "BodyFinal"){
        Base::addNodeToGroup(id, NodeGroups::BODY_FINAL);
    }
    else if(groupId == "FrameFinal"){
        Base::addNodeToGroup(id, NodeGroups::FRAME_FINAL);
    }
//    else if(groupId == "FileFinal"){
//        Base::addNodeToGroup(id, NodeGroups::FILE_FINAL);
//    }
    else{
         GRSF_ERRORMSG("No valid group: " << groupId << " for node id: " << id)
//        unsigned int g;
//        if(!Utilities::stringToType(g,groupId)){
//            GRSF_ERRORMSG("---> String conversion for groupId: '" << groupId << "' not successful!");
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
    }
    else if(groupId == "File"){
         Base::addNodeToGroup(id, NodeGroups::FILE_RESET);
    }else{
          GRSF_ERRORMSG("No valid group: " << groupId << " for node id: " << id)
//        unsigned int g;
//        if(!Utilities::stringToType(g,groupId)){
//            GRSF_ERRORMSG("---> String conversion for: groupId: '" << groupId << "' not found!");
//        }
//        Base::addNodeToGroup(id, g);
    }
}
