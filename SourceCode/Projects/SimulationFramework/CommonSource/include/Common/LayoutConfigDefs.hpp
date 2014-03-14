/*
 *  LayoutConfigDefs.hpp
 *
 *  Created by Gabriel NÃ¼tzi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef LayoutConfigDefs_hpp
#define LayoutConfigDefs_hpp

#include <Eigen/Dense>

#include <Eigen/Geometry>

#include "MyMatrixDefs.hpp"

/**
* @brief A dynamic layout specialization.
*/
template <int nDOFqBody, int nDOFuBody>
struct GeneralLayout {
   static int const NDOFqBody = nDOFqBody;
   static int const NDOFuBody = nDOFuBody;
};

// ================================================================================================
/**
* @brief This is the LayoutConfig which specifies the layout of the system.
*/
template <typename TPREC, typename TLayout>
struct LayoutConfig{
   typedef TPREC PREC;
   typedef TLayout LayoutType;

   typedef Eigen::Matrix<PREC, LayoutType::NDOFqBody, LayoutType::NDOFuBody>    MatrixQBodyUBody;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFqBody, 1>                        VectorQBody;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFuBody, 1>                        VectorUBody;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFqBody, Eigen::Dynamic >          MatrixQBodyDyn;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFuBody, Eigen::Dynamic >          MatrixUBodyDyn;

   // Static Vectors/Matrices
   DEFINE_MATRIX_TYPES_OF( PREC );

};



/**
* @brief This macro is used to typedef all template arguments in a class with e.g template argument typename â€œLayoutConfigâ€
*/
#define DEFINE_LAYOUT_CONFIG_TYPES_OF( _LayoutConfigName_ ) \
   typedef typename _LayoutConfigName_::PREC PREC;             \
   static int const NDOFqBody = _LayoutConfigName_::LayoutType::NDOFqBody; \
   static int const NDOFuBody = _LayoutConfigName_::LayoutType::NDOFuBody; \
   typedef typename _LayoutConfigName_::VectorQBody VectorQBody;       \
   typedef typename _LayoutConfigName_::VectorUBody VectorUBody;       \
   typedef typename _LayoutConfigName_::MatrixQBodyDyn MatrixQBodyDyn;       \
   typedef typename _LayoutConfigName_::MatrixUBodyDyn MatrixUBodyDyn;       \
   typedef typename _LayoutConfigName_::MatrixQBodyUBody MatrixQBodyUBody; \
   DEFINE_MATRIX_TYPES_OF( PREC );


#endif
