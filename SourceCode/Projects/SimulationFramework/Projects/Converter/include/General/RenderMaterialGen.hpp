#ifndef  RenderMaterialGenerator_hpp
#define  RenderMaterialGenerator_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "MultiBodySimFile.hpp"
#include "ExecutionTreeInOut.hpp"
#include "RenderMaterial.hpp"

class RenderMaterialGenerator : public ExecutionTreeInOut{
    public:

        RenderMaterialGenerator(){}

        void fillInput(RigidBodyStateAdd * s){
            m_inputNode->setOSocketValue(0,s->m_id);
            m_inputNode->setOSocketValue(1,s->m_q);
            m_inputNode->setOSocketValue(2,s->m_u);

            if(s->m_data){
                switch(s->m_data->m_type) {

                    case AdditionalBodyData::TypeEnum::PROCESS:
                        {
                        auto * p = static_cast<AdditionalBodyData::Process *>(s->m_data);
                        m_inputNode->setOSocketValue(4,p->m_processId);
                        }
                        break;

                    case AdditionalBodyData::TypeEnum::PROCESS_MATERIAL:
                        {
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

        std::shared_ptr<RenderMaterial> generateMaterial(){
            execute();

            return m_outputNode->getOSocketValue<std::shared_ptr<RenderMaterial> >(0);
        }
};


#endif // RenderMaterialGenerator_hpp
