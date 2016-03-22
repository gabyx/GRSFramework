#ifndef GRSF_General_SimFileExecutionGraph_hpp
#define GRSF_General_SimFileExecutionGraph_hpp

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/SimpleLogger.hpp"

#include "GRSF/Dynamics/Buffers/RigidBodyState.hpp"
#include "GRSF/Logic/ExecutionTreeInOut.hpp"


namespace LogicNodes{
    class BodyData;
    class StateData;
    class StopNode;
    class SimFileInfo;
};


class SimFileExecutionGraph : public ExecutionTreeInOut{
    public:

        struct NodeGroups{
            enum {
                FILE_RESET, // 0
                FILE_EXEC,  // 1
                    FRAME_RESET, // 2
                    FRAME_EXEC,    /** 3, all nodes which need to update for producing the inputs for the current frame */
                        BODY_RESET, // 4
                        BODY_EXEC,  /** 5, all nodes which need to update for producing an render output from an input node BodyData*/
                        BODY_FINAL, // 6
                    FRAME_FINAL, // 7
                FILE_FINAL, /* 8, not yet implemented */
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

        /** provide function for SimFileConverter ==================================*/

        void initSimInfo(boost::filesystem::path simFile,
                         boost::filesystem::path outputFilePath,
                         std::size_t nBodies,std::size_t nStates);

        void initState(boost::filesystem::path outputFilePath,
                       double time, unsigned int frameNr);

        void addBodyState(RigidBodyStateAdd * s);

        template<typename RigidBodyStateCont>
        void addState(const RigidBodyStateCont & s){
            ERRORMSG("This function should not be called")
        }

        void finalizeState();

        inline bool isStopBodyLoop(){  return checkStop(NodeGroups::BODY_EXEC); }
        inline bool isStopFrameLoop(){ return checkStop(NodeGroups::FRAME_EXEC); }
        inline bool isStopFileLoop(){ return checkStop(NodeGroups::FILE_EXEC); }
        /** ========================================================================*/


        inline void setStateData(LogicNodes::StateData *n){m_stateData = n;}
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
        LogicNodes::StateData   * m_stateData    =  nullptr;
        LogicNodes::SimFileInfo * m_simFileInfo  =  nullptr;

        /** Stop nodes for Body and Frame group, FILE_EXEC, FRAME_EXEC, BODY_EXEC */
        std::array<LogicNodes::StopNode*,3> m_stopNodes =  {{nullptr,nullptr,nullptr}};

        Logging::Log * m_log = nullptr;
};


#endif // GRSF_General_SimFileExecutionGraph_hpp

