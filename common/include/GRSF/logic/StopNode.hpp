// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_logic_StopNode_hpp
#define GRSF_logic_StopNode_hpp

#include "GRSF/logic/LogicNode.hpp"

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

        StopNode(unsigned int id) : LogicNode(id){
            ADD_ISOCK(Enable,false);
        }

        virtual ~StopNode() {}

    };
};

#endif // GRSF_Logic_StopNode_hpp


