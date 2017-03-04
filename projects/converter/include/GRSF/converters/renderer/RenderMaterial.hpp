// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_renderer_RenderMaterial_hpp
#define GRSF_converters_renderer_RenderMaterial_hpp

#include <sstream>
#include "GRSF/common/TypeDefs.hpp"

class RenderMaterial
{
public:
    RenderMaterial() : m_id(0)
    {
    }
    RenderMaterial(unsigned int id, const std::string& s) : m_id(id)
    {
        m_s << s;
    }

    ~RenderMaterial(){};

    template <typename T>
    inline RenderMaterial& operator<<(const T& t)
    {
        m_s << t;
        return *this;
    }

    inline std::string str()
    {
        return m_s.str();
    }

    inline void clear()
    {
        m_s.str("");
    }

    inline void write(std::stringstream& s)
    {
        s << m_s.rdbuf() << "\n";
        s.clear();
        m_s.seekp(0);
        m_s.seekg(0);
    }

private:
    std::stringstream m_s;
    unsigned int m_id;
};

#endif
