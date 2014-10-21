#ifndef  RenderMaterialGenerator_hpp
#define  RenderMaterialGenerator_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "MultiBodySimFile.hpp"
#include "ExecutionTreeInOut.hpp"


namespace LogicNodes{
    class BodyData;
    class RenderOutput;
};

class RenderMaterial;

class RenderMaterialGenerator : public ExecutionTreeInOut{
    public:

        RenderMaterialGenerator(){}

        void fillInput(RigidBodyStateAdd * s);

        virtual void setup();

        std::shared_ptr<RenderMaterial> generateMaterial();

        void initFrame(boost::filesystem::path folder, std::string filename, double time);

    private:
        LogicNodes::BodyData * m_bodyDataNode = nullptr;
        std::unordered_map<unsigned int, LogicNodes::RenderOutput *> m_renderOutputNodes;

};


#endif // RenderMaterialGenerator_hpp
