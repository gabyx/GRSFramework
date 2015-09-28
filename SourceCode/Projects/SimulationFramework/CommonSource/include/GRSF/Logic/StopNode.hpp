#ifndef GRSF_Logic_StopNode_hpp
#define GRSF_Logic_StopNode_hpp

#include "GRSF/Logic/LogicNode.hpp"

namespace LogicNodes{

    class StopNode: public LogicNode {
    public:

        struct Inputs {
            enum {
                Enable,
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST - Inputs::INPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_ISOCKET_TYPE(Enable, bool);


        StopNode(unsigned int id) : LogicNode(id) {
            ADD_ISOCK(Enable,false);
        }

        virtual ~StopNode() {}

        virtual void compute(){}
        virtual void initialize(){}
    };
};

#endif // GRSF_Logic_StopNode_hpp


