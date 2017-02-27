// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_logic_LookUpTable_hpp
#define GRSF_logic_LookUpTable_hpp

#include "GRSF/logic/LogicNode.hpp"

namespace LogicNodes
{
template <typename TKey, typename TValue, typename TMap>
class LookUpTable : public LogicNode
{
    public:
    struct Inputs
    {
        enum
        {
            Enable,
            Key,
            INPUTS_LAST
        };
    };

    struct Outputs
    {
        enum
        {
            Value,
            OUTPUTS_LAST
        };
    };

    enum
    {
        N_INPUTS  = Inputs::INPUTS_LAST,
        N_OUTPUTS = Outputs::OUTPUTS_LAST - Inputs::INPUTS_LAST,
        N_SOCKETS = N_INPUTS + N_OUTPUTS
    };

    DECLARE_ISOCKET_TYPE(Enable, bool);
    DECLARE_ISOCKET_TYPE(Key, TKey);
    DECLARE_OSOCKET_TYPE(Value, TValue);

    LookUpTable(unsigned int id, TMap* m, const TValue& d) : LogicNode(id), m_map(m), m_default(d)
    {
        ADD_ISOCK(Enable, true);
        ADD_ISOCK(Key, TKey());
        ADD_OSOCK(Value, TValue());
    }

    virtual ~LookUpTable()
    {
    }

    virtual void compute()
    {
        if (GET_ISOCKET_REF_VALUE(Enable))
        {
            // std::cout << "Value Key: " << GET_ISOCKET_VALUE(Key) << std::endl;
            auto id = GET_ISOCKET_VALUE(Key);
            auto it = m_map->find(id);

            if (it == m_map->end())
            {
                SET_OSOCKET_VALUE(Value, m_default);
            }
            else
            {
                SET_OSOCKET_VALUE(Value, it->second);
            }
        }
    }

    virtual void initialize(){};

    private:
    TMap*  m_map;
    TValue m_default;
};
};

#endif  // LookUpTable_hpp
