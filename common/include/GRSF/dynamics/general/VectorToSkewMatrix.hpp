// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_VectorToSkewMatrix_hpp
#define GRSF_dynamics_general_VectorToSkewMatrix_hpp

/**
* @ingroup Common
* @defgroup VectorToSkewMatrix Vector To Skew Matrix Conversion */
/** @{ */

/**
* @brief This function converts a 3x1 Vector into a skew symmetric matrix 3x3.
* @param v The input 3x1 vector.
* @return The output skew symmetric 3x3 matrix.
*/
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> getSkewSymmetricMatrix(const Eigen::MatrixBase<Derived>& v)
{
    using PREC = typename Derived::Scalar;
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

    Eigen::Matrix<PREC, 3, 3> M;
    setSkewSymmetricMatrix(v, M);
    return M;
}

/**
* @brief This function converts a 3x1 Vector into a skew symmetric matrix 3x3.
* @param v The input 3x1 vector.
* @param M The output skew symmetric 3x3 matrix.
*/
template <typename Derived, typename DerivedOther>
void setSkewSymmetricMatrix(const Eigen::MatrixBase<Derived>& v, Eigen::MatrixBase<DerivedOther>& M)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedOther, 3, 3);

    M.setZero();
    M(0, 1) = -v(2);
    M(0, 2) = v(1);
    M(1, 0) = v(2);
    M(1, 2) = -v(0);
    M(2, 0) = -v(1);
    M(2, 1) = v(0);
}

/**
* @brief This function updates a 3x3 skew symmetric matrix with a vector v.
* @param v The input 3x1 vector.
* @param M The output skew symmetric matrix.
* This function only updates the reference 3x3 matrix, it assumes that the diagonal is zero!
*/
template <typename Derived, typename DerivedOther>
void updateSkewSymmetricMatrix(const Eigen::MatrixBase<Derived>& v, Eigen::MatrixBase<DerivedOther>& M)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedOther, 3, 3);

    M(0, 1) = -v(2);
    M(0, 2) = v(1);
    M(1, 0) = v(2);
    M(1, 2) = -v(0);
    M(2, 0) = -v(1);
    M(2, 1) = v(0);
}

/* @} */

#endif
