#ifndef  RenderScriptGenerator_hpp
#define  RenderScriptGenerator_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "MultiBodySimFile.hpp"
#include "ExecutionTreeInOut.hpp"


namespace LogicNodes{
    class BodyData;
    class FrameData;
    class RenderScriptWriter;
};

class RenderMaterial;

class RenderScriptGenerator : public ExecutionTreeInOut{
    public:

        struct ExecGroups{
            enum {
                BODY, /** all nodes which need to update for producing an render output from an input node BodyData*/
                FRAME, /** all nodes which need to update for producing the inputs for the current frame */
                NGROUPS
            };
        };

        RenderScriptGenerator(){};


        void setup();

        void initFrame(boost::filesystem::path folder, std::string filename, double time, unsigned int frameNr);
        void generateFrameData(RigidBodyStateAdd * s);

        void setFrameData(LogicNodes::FrameData *n){m_frameData = n;};
        void setBodyData(LogicNodes::BodyData *n){m_bodyDataNode = n;};

    private:
        LogicNodes::BodyData * m_bodyDataNode = nullptr;
        std::unordered_set<LogicNodes::RenderScriptWriter *> m_scriptWritterNodes;

        LogicNodes::FrameData * m_frameData =  nullptr;

};


#endif // RenderScriptGenerator_hpp