// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

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


