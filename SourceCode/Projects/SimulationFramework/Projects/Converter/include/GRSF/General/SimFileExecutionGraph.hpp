#ifndef  GRSF_General_SimFileExecutionGraph_hpp
#define  GRSF_General_SimFileExecutionGraph_hpp

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/SimpleLogger.hpp"

#include "GRSF/Dynamics/General/MultiBodySimFile.hpp"
#include "GRSF/Logic/ExecutionTreeInOut.hpp"


namespace LogicNodes{
    class BodyData;
    class FrameData;
    class StopNode;
};


class SimFileExecutionGraph : public ExecutionTreeInOut{
    public:

        struct NodeGroups{
            enum {
                FRAME_EXEC = 0, /** all nodes which need to update for producing the inputs for the current frame */
                BODY_EXEC = 1, /** all nodes which need to update for producing an render output from an input node BodyData*/
                BODY_INIT,
                FRAME_INIT,
                NNODE_GROUPS
            };
        };

        SimFileExecutionGraph(){};

        inline void setLog(Logging::Log * log){ m_log = log; }

        virtual void setup();
        virtual void initFrame(boost::filesystem::path folder, std::string filename, double time, unsigned int frameNr);
        virtual void finalizeFrame();
        virtual void generateFrameData(RigidBodyStateAdd * s);

        inline void setFrameData(LogicNodes::FrameData *n){m_frameData = n;}
        inline void setBodyData(LogicNodes::BodyData *n){m_bodyDataNode = n;}
        void setStopNode(LogicNodes::StopNode *n, unsigned int stopGroupId);


        bool checkStop(unsigned int groupId);

    private:

        LogicNodes::BodyData  * m_bodyDataNode =  nullptr;
        LogicNodes::FrameData * m_frameData    =  nullptr;

        /** Stop nodes for Body and Frame group */
        std::array<LogicNodes::StopNode*,2> m_stopNodes =  {{nullptr,nullptr}};

        Logging::Log * m_log = nullptr;
};


#endif // GRSF_General_SimFileExecutionGraph_hpp

