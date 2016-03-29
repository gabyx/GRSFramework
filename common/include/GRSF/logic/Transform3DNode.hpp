// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_logic_Transform3DNode_hpp
#define GRSF_logic_Transform3DNode_hpp

#include "GRSF/logic/LogicNode.hpp"

namespace LogicNodes{

    class Transform3D: public LogicNode {
    public:

        DEFINE_MATRIX_TYPES

        struct Inputs {
            enum {
                Value,
                Rotation,
                Translation,
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

        DECLARE_ISOCKET_TYPE(Value, Vector3 );
        DECLARE_ISOCKET_TYPE(Rotation, Quaternion );
        DECLARE_ISOCKET_TYPE(Translation, Vector3 );
        DECLARE_OSOCKET_TYPE(Result, Vector3 );



        Transform3D(unsigned int id ) : LogicNode(id) {
            ADD_ISOCK(Value,Vector3(0,0,0) );
            ADD_ISOCK(Rotation,Quaternion{});
            ADD_ISOCK(Translation,Vector3(0,0,0) );
            ADD_OSOCK(Result,Vector3(0,0,0) );
        }

        Transform3D(unsigned int id,
                      const Quaternion & rot,
                      const Vector3 & trans,
                      Vector3 value=Vector3(0,0,0) ) : LogicNode(id) {
            ADD_ISOCK(Value,value );
            ADD_ISOCK(Rotation,rot);
            ADD_ISOCK(Translation, trans );
            ADD_OSOCK(Result,Vector3(0,0,0) );
        }

        virtual ~Transform3D() {
        }

        virtual void compute(){
            GET_OSOCKET_REF_VALUE(Result) = GET_ISOCKET_REF_VALUE(Rotation)  * GET_ISOCKET_REF_VALUE(Value) + GET_ISOCKET_REF_VALUE(Translation) ;
        }
    };
};

#endif



