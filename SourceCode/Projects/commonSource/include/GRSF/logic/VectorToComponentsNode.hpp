#ifndef GRSF_Logic_VectorToComponentsNode_hpp
#define GRSF_Logic_VectorToComponentsNode_hpp

#include "GRSF/Logic/LogicNode.hpp"

namespace LogicNodes{

    template<typename T>
    class VectorToComponent: public LogicNode {
    public:

        using PREC = typename T::Scalar;

        struct Inputs {
            enum {
                Value,
                Index,
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                Component,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST - Inputs::INPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_ISOCKET_TYPE(Value, T );
        DECLARE_ISOCKET_TYPE(Index, unsigned long int );
        DECLARE_OSOCKET_TYPE(Component, PREC );


        VectorToComponent(unsigned int id) : LogicNode(id) {
            ADD_ISOCK(Value,T());
            ADD_ISOCK(Index, 0 );
            ADD_OSOCK(Component,PREC());
        }

        virtual ~VectorToComponent() {
        }

        virtual void compute(){
            //std::cout << GET_ISOCKET_REF_VALUE(Value) << std::endl;
            ASSERTMSG( GET_ISOCKET_REF_VALUE(Index) <=  GET_ISOCKET_REF_VALUE(Value).rows() ,
                      "Wrong index: " << GET_ISOCKET_REF_VALUE(Index) << " in node id: " << this->m_id )

            GET_OSOCKET_REF_VALUE(Component) =  GET_ISOCKET_REF_VALUE(Value)[ GET_ISOCKET_REF_VALUE(Index) ] ;
        }
    };
};

#endif // NormNode_hpp



