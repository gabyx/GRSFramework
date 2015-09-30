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
    class SimFileInfo;
};


class SimFileExecutionGraph : public ExecutionTreeInOut{
    public:

        struct NodeGroups{
            enum {
                FRAME_RESET,
                FRAME_EXEC,    /** all nodes which need to update for producing the inputs for the current frame */
                    BODY_RESET,
                    BODY_EXEC,  /** all nodes which need to update for producing an render output from an input node BodyData*/
                    BODY_FINAL,
                FRAME_FINAL,
                NNODE_GROUPS
            };
        };
        /** Each node has reset(), compute() and finalize()
            This class call the above groups in the implemented order here
            If a node happens to be in BODY_RESET group its reset() function will be called when this->reset(BODY_RESET)

        */

        using Base = ExecutionTreeInOut;

        SimFileExecutionGraph(){};

        inline void setLog(Logging::Log * log){ m_log = log; }

        void setup();
        void initSimInfo(std::size_t nBodies,std::size_t nStates);
        void initFrame(boost::filesystem::path folder, std::string filename, double time, unsigned int frameNr);
        void addBodyState(RigidBodyStateAdd * s);
        void finalizeFrame();
        inline bool isStopBodyLoop(){  return checkStop(NodeGroups::BODY_EXEC); }
        inline bool isStopFrameLoop(){ return checkStop(NodeGroups::FRAME_EXEC); }

        inline void setFrameData(LogicNodes::FrameData *n){m_frameData = n;}
        inline void setBodyData(LogicNodes::BodyData *n){m_bodyDataNode = n;}
        inline void setSimFileInfo(LogicNodes::SimFileInfo *n){m_simFileInfo = n;}
        void setStopNode(LogicNodes::StopNode *n, unsigned int stopGroupId);

        void addNodeToGroup(unsigned int nodeId, std::string groupId = "Body");
        void addNodeToResetGroup(unsigned int nodeId, std::string groupId = "Body");


        static const std::map<std::string, unsigned int > m_nameToExecGroupId;
        static const std::map<std::string, unsigned int > m_nameToInitGroupId;

    private:

        bool checkStop(unsigned int groupId);

        LogicNodes::BodyData    * m_bodyDataNode =  nullptr;
        LogicNodes::FrameData   * m_frameData    =  nullptr;
        LogicNodes::SimFileInfo * m_simFileInfo  =  nullptr;

        /** Stop nodes for Body and Frame group */
        std::array<LogicNodes::StopNode*,2> m_stopNodes =  {{nullptr,nullptr}};

        Logging::Log * m_log = nullptr;
};


#endif // GRSF_General_SimFileExecutionGraph_hpp

