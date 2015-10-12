#include "GRSF/General/RenderExecutionGraph.hpp"


#include "GRSF/General/RenderExecutionGraphNodes.hpp"


void RenderExecutionGraph::setup() {

    Base::setup();

    m_scriptWritterNodes.clear();
    auto & outNodes = this->getOutputNodes();
    for(auto & n : outNodes){
        LogicNodes::RenderScriptWriter * r = dynamic_cast<LogicNodes::RenderScriptWriter * >(n);
        if(!r) {
            ERRORMSG("Output node is not of type 'RenderScriptWriter = [RendermanWriter]' ")
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
