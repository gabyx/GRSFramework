// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef MatrixHelpers_hpp
#define MatrixHelpers_hpp

#include <Eigen/Dense>

#include <boost/math/special_functions/fpclassify.hpp>

#include "TypeDefs.hpp"

/**
* @ingroup Common
* @defgroup MatrixHelpers Matrix helper function
*/
/* @{ */

namespace MatrixHelpers
{
template <typename PREC, typename PREC2>
void setHomogeneousTransform(const typename MyMatrix<PREC>::Matrix33& A_IK,
                             const typename MyMatrix<PREC>::Vector3& I_t_IK,
                             typename MyMatrix<PREC2>::Matrix44& H_IK)
{
    // Sets the matrix H_IK, ==> I_r_P = H_IK * K_r_p;
    H_IK.setIdentity();
    H_IK.block<3, 3>(0, 0) = A_IK.template cast<PREC2>();
    H_IK.block<3, 1>(0, 3) = I_t_IK.template cast<PREC2>();
}

template <typename Derived>
bool isfinite(const Eigen::MatrixBase<Derived>& mat)
{
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
    for (int i = 0; i < mat.cols(); i++)
    {
        if (!(boost::math::isfinite)(mat(i)))
        {
            return false;
        }
    }
    return true;
}
};

/* @} */
#endif
