// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_logic_VectorToComponentsNode_hpp
#define GRSF_logic_VectorToComponentsNode_hpp

#include "GRSF/logic/LogicNode.hpp"

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
            GRSF_ASSERTMSG( GET_ISOCKET_REF_VALUE(Index) <=  GET_ISOCKET_REF_VALUE(Value).rows() ,
                      "Wrong index: " << GET_ISOCKET_REF_VALUE(Index) << " in node id: " << this->m_id )

            GET_OSOCKET_REF_VALUE(Component) =  GET_ISOCKET_REF_VALUE(Value)[ GET_ISOCKET_REF_VALUE(Index) ] ;
        }
    };
};

#endif // NormNode_hpp



