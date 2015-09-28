#ifndef  GRSF_General_LogicNodeGeneratorSimFile_hpp
#define  GRSF_General_LogicNodeGeneratorSimFile_hpp

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


class LogicNodeGeneratorSimFile : public ExecutionTreeInOut{
    public:

        struct ExecGroups{
            enum {
                BODY, /** all nodes which need to update for producing an render output from an input node BodyData*/
                FRAME, /** all nodes which need to update for producing the inputs for the current frame */
                NEXECGROUPS
            };
        };

        struct InitializeGroups{
            enum {
                BODY,  /** all nodes which are initialized before the BODY ExecGroup is executed*/
                FRAME, /** all nodes which are initialized before the FRAME ExecGroup is executed*/
                NINITGROUPS
            };
        };



        LogicNodeGeneratorSimFile(){};

        inline void setLog(Logging::Log * log){ m_log = log; }

        virtual void setup();
        virtual void initFrame(boost::filesystem::path folder, std::string filename, double time, unsigned int frameNr);
        virtual void finalizeFrame();
        virtual void generateFrameData(RigidBodyStateAdd * s);

        inline void setFrameData(LogicNodes::FrameData *n){m_frameData = n;}
        inline void setBodyData(LogicNodes::BodyData *n){m_bodyDataNode = n;}
        inline void setStopNode(LogicNodes::StopNode *n){m_stopNode = n;}


        bool checkStop();

    private:

        LogicNodes::BodyData  * m_bodyDataNode =  nullptr;
        LogicNodes::FrameData * m_frameData    =  nullptr;
        LogicNodes::StopNode  * m_stopNode     =  nullptr;

        Logging::Log * m_log = nullptr;
};


#endif // GRSF_General_LogicNodeGeneratorSimFile_hpp

