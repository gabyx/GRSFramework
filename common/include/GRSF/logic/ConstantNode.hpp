#ifndef GRSF_logic_ConstantNode_hpp
#define GRSF_logic_ConstantNode_hpp

#include "GRSF/logic/LogicNode.hpp"

namespace LogicNodes{

    template<typename T>
    class ConstantNode: public LogicNode {
    public:

        struct Inputs {
            enum {
                INPUTS_LAST = 0
            };
        };

        struct Outputs {
            enum {
                Value,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST - Inputs::INPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_OSOCKET_TYPE(Value, T );


        ConstantNode(unsigned int id, const T & d) : LogicNode(id) {
            ADD_OSOCK(Value,d);
        }

        virtual ~ConstantNode() {
        }
    };
};

#endif // ConstantNode_hpp


