#ifndef  RenderExecutionGraph_hpp
#define  RenderExecutionGraph_hpp

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/General/SimFileExecutionGraph.hpp"


namespace LogicNodes{
    class RenderScriptWriter;
};

class RenderMaterial;

class RenderExecutionGraph : public SimFileExecutionGraph
{
    public:
        using Base = SimFileExecutionGraph;
        using ExecGroups = Base::ExecGroups;

        RenderExecutionGraph(){};

        virtual void setup();
        virtual void initFrame(boost::filesystem::path folder, std::string filename, double time, unsigned int frameNr);
        virtual void finalizeFrame();
        virtual void generateFrameData(RigidBodyStateAdd * s);

    private:
        std::unordered_set<LogicNodes::RenderScriptWriter *> m_scriptWritterNodes;
};


#endif // RenderExecutionGraph_hpp
