// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/converters/renderer/RenderExecutionGraph.hpp"


#include "GRSF/converters/renderer/RenderExecutionGraphNodes.hpp"


void RenderExecutionGraph::setup() {

    Base::setup();

    m_scriptWritterNodes.clear();
    auto & outNodes = this->getOutputNodes();
    for(auto & n : outNodes){
        LogicNodes::RenderScriptWriter * r = dynamic_cast<LogicNodes::RenderScriptWriter * >(n);
        if(!r) {
            continue;
        }
        m_scriptWritterNodes.emplace(r);
    }

    if( m_scriptWritterNodes.size() == 0  ) {
            ERRORMSG("Execution tree has no output node 'RenderScriptWriter = [RendermanWriter]' ")
    }


}

void RenderExecutionGraph::initState(boost::filesystem::path outputFilePath,
                                      double time,
                                      unsigned int frameNr)
{
     Base::initState(outputFilePath,time,frameNr);

     // TODO RendermanWriter node should be LogicNodeGroup (which consist of several LogicNodes by composition)
     // one for InitNode, AddBodyStateNode, FinalizeFrameNode,  which then can be connected to
     // and the group assigned to
     // InitNode -> FRAME_INIT,
     // AddBodyStateNode -> BODY_FINAL
     // ExecutionTreeInOut needs then support to add LogicNodeGroup, adds all members nodes of a LogicGroupNode
     // LogicGroupNode is something special, no compute() function, but provides a ways to expose multiple functionality
     //

     // Call all render script writters
     for(auto & n : m_scriptWritterNodes){
        n->initFrame();
     }
}

void RenderExecutionGraph::finalizeState(){

     Base::finalizeState();

     // Call all render script writters
     for(auto & n : m_scriptWritterNodes){
        n->finalizeFrame();
     }
}

void RenderExecutionGraph::addBodyState(RigidBodyStateAdd * s) {

    Base::addBodyState(s);

    // Call all render script writters
     for(auto & n : m_scriptWritterNodes){
        n->writeBody();
     }
}
