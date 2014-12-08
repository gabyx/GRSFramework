#ifndef GMSF_Logic_NormNode_hpp
#define GMSF_Logic_NormNode_hpp

#include "LogicNode.hpp"

namespace LogicNodes{

    template<typename T>
    class NormNode: public LogicNode {
    public:

        using PREC = typename T::Scalar;

        struct Inputs {
            enum {
                Value,
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                Result,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST - Inputs::INPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_ISOCKET_TYPE(Value, T );
        DECLARE_OSOCKET_TYPE(Result, PREC );


        NormNode(unsigned int id) : LogicNode(id) {
            ADD_ISOCK(Value,T());
            ADD_OSOCK(Result,PREC());
        }

        virtual ~NormNode() {
        }

        virtual void compute(){
            //std::cout << GET_ISOCKET_REF_VALUE(Value) << std::endl;
            SET_OSOCKET_VALUE(Result,GET_ISOCKET_REF_VALUE(Value).norm());
        }
        virtual void initialize(){}
    };
};

#endif // NormNode_hpp



