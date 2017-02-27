// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_logic_DummyNode_hpp
#define GRSF_logic_DummyNode_hpp

#include "GRSF/logic/LogicNode.hpp"

template <unsigned int NIN, unsigned int NOUT>
class DummyLogicNode : public LogicNode
{
public:
    DummyLogicNode(unsigned int id) : LogicNode(id)
    {
        for (unsigned int i = 0; i < NIN; i++)
        {
            addISock(0.0);
        }
        for (unsigned int i = 0; i < NOUT; i++)
        {
            addOSock(0.0);
        }
    }

    virtual ~DummyLogicNode()
    {
    }

    void compute()
    {
        double t = 0;
        for (unsigned int i = 0; i < NIN; i++)
        {
            t += getISocketValue<double>(i);
        }
        std::cout << m_id << ": " << t << std::endl;
        for (unsigned int i = 0; i < NOUT; i++)
        {
            setOSocketValue(i, t + 1);
        };
    }
};
#endif
