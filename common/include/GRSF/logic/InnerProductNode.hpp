// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_logic_InnerProductNode_hpp
#define GRSF_logic_InnerProductNode_hpp

#include "GRSF/logic/LogicNode.hpp"

namespace LogicNodes
{
template <typename T>
class InnerProduct : public LogicNode
{
public:
    using PREC = typename T::Scalar;

    struct Inputs
    {
        enum
        {
            Value1,
            Value2,
            INPUTS_LAST
        };
    };

    struct Outputs
    {
        enum
        {
            Result,
            OUTPUTS_LAST
        };
    };

    enum
    {
        N_INPUTS  = Inputs::INPUTS_LAST,
        N_OUTPUTS = Outputs::OUTPUTS_LAST - Inputs::INPUTS_LAST,
        N_SOCKETS = N_INPUTS + N_OUTPUTS
    };

    DECLARE_ISOCKET_TYPE(Value1, T);
    DECLARE_ISOCKET_TYPE(Value2, T);
    DECLARE_OSOCKET_TYPE(Result, PREC);

    InnerProduct(unsigned int id) : LogicNode(id)
    {
        ADD_ISOCK(Value1, T());
        ADD_ISOCK(Value2, T());
        ADD_OSOCK(Result, PREC());
    }

    virtual ~InnerProduct()
    {
    }

    virtual void compute()
    {
        // std::cout << GET_ISOCKET_REF_VALUE(Value) << std::endl;
        GET_OSOCKET_REF_VALUE(Result) = GET_ISOCKET_REF_VALUE(Value1).dot(GET_ISOCKET_REF_VALUE(Value2));
    }
};
};

#endif  // NormNode_hpp
