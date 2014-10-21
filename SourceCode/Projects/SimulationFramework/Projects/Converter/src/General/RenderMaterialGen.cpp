#include "RenderMaterialGen.hpp"

#include "RenderMaterial.hpp"
#include "RenderMaterialGenLogic.hpp"
#include "RenderOutputLogic.hpp"

void RenderMaterialGenerator::fillInput(RigidBodyStateAdd * s) {
    m_inputNode->setOSocketValue(0,s->m_id);
    m_inputNode->setOSocketValue(1,s->m_q);
    m_inputNode->setOSocketValue(2,s->m_u);

    if(s->m_data) {
        switch(s->m_data->m_type) {

        case AdditionalBodyData::TypeEnum::PROCESS: {
            auto * p = static_cast<AdditionalBodyData::Process *>(s->m_data);
            m_inputNode->setOSocketValue(4,p->m_processId);
        }
        break;

        case AdditionalBodyData::TypeEnum::PROCESS_MATERIAL: {
            auto * p = static_cast<AdditionalBodyData::ProcessMaterial *>(s->m_data);
            m_inputNode->setOSocketValue(3,p->m_materialId);
            m_inputNode->setOSocketValue(4,p->m_processId);
        }
        break;

        default:
            ERRORMSG("Additional bytes could not be filled into input Node!")
        }
    }
};

void RenderMaterialGenerator::setup() {
    ExecutionTreeInOut::setup();

    m_bodyDataNode = dynamic_cast<LogicNodes::BodyData * >(this->getInputNode());
    if( !m_bodyDataNode  ) {
        ERRORMSG("Execution tree has no input node of type 'BodyData' ")
    }

    m_renderOutputNodes.clear();
    auto & outNodes = this->getOutputNodes();
    for(auto & n : outNodes){
        LogicNodes::RenderOutput * r = dynamic_cast<LogicNodes::RenderOutput * >(n.second);
        if(!r) {
            ERRORMSG("Output node id: " << n.first << " is not of type 'RenderOutput = [RendermanOutput]' ")
        }
        m_renderOutputNodes.emplace(n.first, r);
    }

    if( m_renderOutputNodes.size() == 0  ) {
            ERRORMSG("Execution tree has no output node 'RenderOutput = [RendermanOutput]' ")
    }


}

void RenderMaterialGenerator::initFrame(boost::filesystem::path folder, std::string filename, double time)
{
     for(auto & n : m_renderOutputNodes){
        n.second->initFrame(folder,filename,time);
     }
}

std::shared_ptr<RenderMaterial> RenderMaterialGenerator::generateMaterial() {
    execute();
    return nullptr;
    //return m_outputNode->getOSocketValue<std::shared_ptr<RenderMaterial> >(0);
}
