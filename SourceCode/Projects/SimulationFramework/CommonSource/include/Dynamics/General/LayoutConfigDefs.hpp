/*
 *  LayoutConfigDefs.hpp
 *
 *  Created by Gabriel NÃ¼tzi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef LayoutConfigDefs_hpp
#define LayoutConfigDefs_hpp


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
   using PREC = TPREC;
   using LayoutType = TLayout;

   using MatrixQBodyUBody = Eigen::Matrix<PREC, LayoutType::NDOFqBody, LayoutType::NDOFuBody>   ;
   using VectorQBody = Eigen::Matrix<PREC, LayoutType::NDOFqBody, 1>                       ;
   using VectorUBody = Eigen::Matrix<PREC, LayoutType::NDOFuBody, 1>                       ;

   using MatrixQBodyDyn = Eigen::Matrix<PREC, LayoutType::NDOFqBody, Eigen::Dynamic >         ;
   using MatrixUBodyDyn = Eigen::Matrix<PREC, LayoutType::NDOFuBody, Eigen::Dynamic >         ;
   using MatrixDynQBody = Eigen::Matrix<PREC, Eigen::Dynamic, LayoutType::NDOFqBody >         ;
   using MatrixDynUBody = Eigen::Matrix<PREC, Eigen::Dynamic, LayoutType::NDOFuBody>          ;

   // Static Vectors/Matrices
   DEFINE_MATRIX_TYPES_OF( PREC );

};



/**
* @brief This macro is used to typedef all template arguments in a class with e.g template argument typename â€œLayoutConfigâ€
*/
#define DEFINE_LAYOUT_CONFIG_TYPES_OF( _LayoutConfigName_ ) \
   using PREC = typename _LayoutConfigName_::PREC;             \
   static int const NDOFqBody = _LayoutConfigName_::LayoutType::NDOFqBody; \
   static int const NDOFuBody = _LayoutConfigName_::LayoutType::NDOFuBody; \
   using VectorQBody = typename _LayoutConfigName_::VectorQBody;       \
   using VectorUBody = typename _LayoutConfigName_::VectorUBody;       \
   using MatrixQBodyDyn = typename _LayoutConfigName_::MatrixQBodyDyn;       \
   using MatrixUBodyDyn = typename _LayoutConfigName_::MatrixUBodyDyn;       \
   using MatrixQBodyUBody = typename _LayoutConfigName_::MatrixQBodyUBody; \
   using MatrixDynQBody = typename _LayoutConfigName_::MatrixDynQBody;       \
   using MatrixDynUBody = typename _LayoutConfigName_::MatrixDynUBody;       \
   DEFINE_MATRIX_TYPES_OF( PREC );


#endif
