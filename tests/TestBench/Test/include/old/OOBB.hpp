// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef OOBB_hpp
#define OOBB_hpp

#include "AABB.hpp"
#include "TypeDefs.hpp"

class OOBB
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DEFINE_MATRIX_TYPES

    OOBB()
    {
        this->reset();
    };

    OOBB(const Vector3& l, const Vector3& u, const Matrix33& A_IK) : m_q_KI(A_IK)
    {
        m_q_KI.normalize();
        m_minPoint = Vector3(std::min(l(0), u(0)), std::min(l(1), u(1)), std::min(l(2), u(2)));
        m_maxPoint = Vector3(std::max(l(0), u(0)), std::max(l(1), u(1)), std::max(l(2), u(2)));
    }

    inline void setZAxisLongest()
    {
        typename Vector3::Index i;
        maxExtent(i);
        if (i < 2)
        {
            switchZAxis(i);
        }
    }

    /** Switch the z-Axis to the axis with index i (default x-Axis)*/
    void switchZAxis(unsigned int i)
    {
        if (i > 1)
        {
            return;
        }
        if (i == 0)
        {
            // Make new x-Axis the z-Axis
            // R_NK = Rotate around 90° around Y, and 90° around X (always in K frame) )
            // A_IN = A_IK * A_KN = R_KI * R_NK
            m_q_KI = m_q_KI * Quaternion(0.5, 0.5, 0.5, 0.5);
            // Change points  Coordinates I_[x,y,z] -> K_[y,z,x]
            std::swap(m_minPoint(0), m_minPoint(1));
            std::swap(m_minPoint(1), m_minPoint(2));

            std::swap(m_maxPoint(0), m_maxPoint(1));
            std::swap(m_maxPoint(1), m_maxPoint(2));
        }
        else
        {
            // Make new y-Axis the z-Axis
            // R_NK = Rotate around 90° around -X, and 90° around -Y (always in K frame) )
            // A_IN = A_IK * A_KN = R_KI * R_NK
            m_q_KI = m_q_KI * Quaternion(0.5, -0.5, -0.5, -0.5);
            // Change points  Coordinates I_[x,y,z] -> K_[z,x,y]
            std::swap(m_minPoint(0), m_minPoint(2));
            std::swap(m_minPoint(1), m_minPoint(2));

            std::swap(m_maxPoint(0), m_maxPoint(2));
            std::swap(m_maxPoint(1), m_maxPoint(2));
        }
    }

    inline void reset()
    {
        // Violating the constraint min<max for making a completey empty box!
        m_minPoint(0) = std::numeric_limits<PREC>::max();
        m_maxPoint(0) = std::numeric_limits<PREC>::min();
        m_minPoint(1) = std::numeric_limits<PREC>::max();
        m_maxPoint(1) = std::numeric_limits<PREC>::min();
        m_minPoint(2) = std::numeric_limits<PREC>::max();
        m_maxPoint(2) = std::numeric_limits<PREC>::min();
    }

    inline Array3 extent() const
    {
        return (m_maxPoint - m_minPoint).array();
    };

    inline PREC maxExtent() const
    {
        return (m_maxPoint - m_minPoint).maxCoeff();
    };

    inline PREC maxExtent(typename Vector3::Index& i) const
    {
        return (m_maxPoint - m_minPoint).maxCoeff(&i);
    };

    inline bool isEmpty() const
    {
        return m_maxPoint(0) <= m_minPoint(0) || m_maxPoint(1) <= m_minPoint(1) || m_maxPoint(2) <= m_minPoint(2);
    }

    inline void expand(PREC d)
    {
        GRSF_ASSERTMSG(d >= 0, "d>=0")
        m_minPoint -= Vector3(d, d, d);
        m_maxPoint += Vector3(d, d, d);
    };

    inline void expand(Vector3 d)
    {
        GRSF_ASSERTMSG(d(0) >= 0 && d(1) >= 0 && d(2) >= 0, "d>=0")
        m_minPoint -= d;
        m_maxPoint += d;
    };

    inline PREC volume() const
    {
        Vector3 d = m_maxPoint - m_minPoint;
        return d(0) * d(1) * d(2);
    };

    /** Get direction vectors in I Frame */
    inline Vector3 getDirection(unsigned int i)
    {
        GRSF_ASSERTMSG(i < 3, "Index wrong: " << i)
        Vector3 d;
        d.setZero();
        d(i) = 1.0;
        return m_q_KI * d;  // A_IK* d;
    }

    Quaternion m_q_KI;      ///< Rotation of frame I to frame K, corresponds to a transformation A_IK;
    Vector3    m_minPoint;  ///< in K Frame
    Vector3    m_maxPoint;  ///< in K Frame
};

#endif  // OOBB_hpp
