// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_LayoutConfigTypeDefs_hpp
#define GRSF_dynamics_general_LayoutConfigTypeDefs_hpp

#include "GRSF/dynamics/general/MyContainerTypeDefs.hpp"
#include "GRSF/dynamics/general/MyMatrixTypeDefs.hpp"

/**
* @brief A dynamic layout specialization.
*/
template <int nDOFqBody, int nDOFuBody>
struct GeneralLayout
{
    static int const NDOFqBody = nDOFqBody;
    static int const NDOFuBody = nDOFuBody;
};

// ================================================================================================
/**
* @brief This is the LayoutConfig which specifies the layout of the system.
*/
template <typename TPREC, typename TLayout>
struct LayoutConfig
{
    using PREC       = TPREC;
    using LayoutType = TLayout;

    // Static Vectors/Matrices
    DEFINE_MATRIX_TYPES_OF(PREC);

    using MatrixQBodyUBody = MatrixStatStat<LayoutType::NDOFqBody, LayoutType::NDOFuBody>;
    using VectorQBody      = MatrixStatStat<LayoutType::NDOFqBody, 1>;
    using VectorUBody      = MatrixStatStat<LayoutType::NDOFuBody, 1>;

    using MatrixQBodyDyn = MatrixStatDyn<LayoutType::NDOFqBody>;
    using MatrixUBodyDyn = MatrixStatDyn<LayoutType::NDOFuBody>;
    using MatrixDynQBody = MatrixDynStat<LayoutType::NDOFqBody>;
    using MatrixDynUBody = MatrixDynStat<LayoutType::NDOFuBody>;

    using MatrixUBody3 = MatrixStatStat<LayoutType::NDOFuBody, 3>;
    using MatrixQBody3 = MatrixStatStat<LayoutType::NDOFqBody, 3>;
};

/**
* @brief This macro is used to typedef all vector/matrices in a class
*/
#define DEFINE_LAYOUT_CONFIG_TYPES_OF(_LayoutConfigName_)                       \
    using PREC = typename _LayoutConfigName_::PREC;                             \
    DEFINE_CONTAINER_TYPES                                                      \
    DEFINE_MATRIX_TYPES_OF(PREC);                                               \
    static int const NDOFqBody = _LayoutConfigName_::LayoutType::NDOFqBody;     \
    static int const NDOFuBody = _LayoutConfigName_::LayoutType::NDOFuBody;     \
    using VectorQBody          = typename _LayoutConfigName_::VectorQBody;      \
    using VectorUBody          = typename _LayoutConfigName_::VectorUBody;      \
    using MatrixQBodyDyn       = typename _LayoutConfigName_::MatrixQBodyDyn;   \
    using MatrixUBodyDyn       = typename _LayoutConfigName_::MatrixUBodyDyn;   \
    using MatrixQBodyUBody     = typename _LayoutConfigName_::MatrixQBodyUBody; \
    using MatrixDynQBody       = typename _LayoutConfigName_::MatrixDynQBody;   \
    using MatrixDynUBody       = typename _LayoutConfigName_::MatrixDynUBody;   \
    using MatrixUBody3         = typename _LayoutConfigName_::MatrixUBody3;     \
    using MatrixQBody3         = typename _LayoutConfigName_::MatrixQBody3;

#endif
