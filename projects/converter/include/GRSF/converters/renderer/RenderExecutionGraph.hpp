#ifndef GRSF_converters_renderer_RenderExecutionGraph_hpp
#define GRSF_converters_renderer_RenderExecutionGraph_hpp

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/general/SimFileExecutionGraph.hpp"


namespace LogicNodes{
    class RenderScriptWriter;
};

class RenderMaterial;

class RenderExecutionGraph : public SimFileExecutionGraph
{
    public:
        using Base = SimFileExecutionGraph;
        using NodeGroups = Base::NodeGroups;

        RenderExecutionGraph(){};

        void setup();
        void initState(boost::filesystem::path outputFilePath, double time, unsigned int frameNr);
        void finalizeState();
        void addBodyState(RigidBodyStateAdd * s);

    private:
        std::unordered_set<LogicNodes::RenderScriptWriter *> m_scriptWritterNodes;
};


#endif // RenderExecutionGraph_hpp
