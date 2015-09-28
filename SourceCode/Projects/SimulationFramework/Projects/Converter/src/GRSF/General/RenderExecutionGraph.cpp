#include "GRSF/General/RenderExecutionGraph.hpp"


#include "GRSF/General/SimFileExecutionGraphLogic.hpp"
#include "GRSF/General/RenderExecutionGraphLogic.hpp"


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

void RenderExecutionGraph::initFrame(boost::filesystem::path folder,
                                      std::string filename,
                                      double time,
                                      unsigned int frameNr)
{
     Base::initFrame(folder,filename,time,frameNr);

     // Call all render script writters
     for(auto & n : m_scriptWritterNodes){
        n->initFrame();
     }
}

void RenderExecutionGraph::finalizeFrame(){

     Base::finalizeFrame();

     // Call all render script writters
     for(auto & n : m_scriptWritterNodes){
        n->finalizeFrame();
     }
}

void RenderExecutionGraph::generateFrameData(RigidBodyStateAdd * s) {

    Base::generateFrameData(s);

    // Call all render script writters
     for(auto & n : m_scriptWritterNodes){
        n->writeBody();
     }
}
