// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef PlaneGeometry_hpp
#define PlaneGeometry_hpp

#include <TypeDefs.hpp>

#include <boost/serialization/access.hpp>

template <class PREC>
class PlaneGeometry
{
public:
    DEFINE_MATRIX_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PlaneGeometry()
    {
        m_normal.setZero();
        m_pos.setZero();
    }
    PlaneGeometry(const Vector3& n, const Vector3& p) : m_normal(n), m_pos(p)
    {
    }

    Vector3 m_normal;  // in K frame
    Vector3 m_pos;     // in K frame

protected:
    friend class boost::serialization::access;
};

#endif
