#include "GRSF/General/RenderScriptGenerator.hpp"

#include "GRSF/General/RenderMaterial.hpp"
#include "GRSF/General/RenderScriptGeneratorLogic.hpp"


void RenderScriptGenerator::setup() {


    ExecutionTreeInOut::setup();

    LOG(m_log, this->getExecutionOrderInfo() );

    if( !m_bodyDataNode  ) {
        ERRORMSG("Execution tree has no input node of type 'BodyData' ")
    }
    if( !m_frameData  ) {
        ERRORMSG("Execution tree has no input node of type 'FrameData' ")
    }

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

void RenderScriptGenerator::initFrame(boost::filesystem::path folder, std::string filename, double time, unsigned int frameNr)
{

     // Set outputs in FrameData
     if(m_frameData){
        m_frameData->setOutput(folder,filename,time,frameNr);
     }else{
        ERRORMSG("There is no FrameData node present! Please add one!")
     }

     // Execute all nodes in FRAME group
     this->execute(ExecGroups::FRAME);

     // Call all render script writters
     for(auto & n : m_scriptWritterNodes){
        n->initFrame();
     }
}

void RenderScriptGenerator::finalizeFrame(){
    // Call all render script writters
     for(auto & n : m_scriptWritterNodes){
        n->finalizeFrame();
     }
}

void RenderScriptGenerator::generateFrameData(RigidBodyStateAdd * s) {
    // Set body data
    m_bodyDataNode->setOutputs(s);

    // Execute all nodes in BODY group
    this->execute(ExecGroups::BODY);

    // Call all render script writters
     for(auto & n : m_scriptWritterNodes){
        n->writeBody();
     }
}
