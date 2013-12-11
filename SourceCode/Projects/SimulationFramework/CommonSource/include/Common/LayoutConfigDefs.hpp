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
template <int nDOFqObj, int nDOFuObj>
struct GeneralLayout {
   static int const NDOFqObj = nDOFqObj;
   static int const NDOFuObj = nDOFuObj;
};

// ================================================================================================
/**
* @brief This is the LayoutConfig which specifies the layout of the system.
*/
template <typename TPREC, typename TLayout>
struct LayoutConfig{
   typedef TPREC PREC;
   typedef TLayout LayoutType;

   typedef Eigen::Matrix<PREC, LayoutType::NDOFqObj, LayoutType::NDOFuObj>     MatrixQObjUObj;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFqObj, 1>                        VectorQObj;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFuObj, 1>                        VectorUObj;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFqObj, Eigen::Dynamic >          MatrixQObjDyn;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFuObj, Eigen::Dynamic >          MatrixUObjDyn;

   // Static Vectors/Matrices
   DEFINE_MATRIX_TYPES_OF( PREC );

};



/**
* @brief This macro is used to typedef all template arguments in a class with e.g template argument typename â€œLayoutConfigâ€
*/
#define DEFINE_LAYOUT_CONFIG_TYPES_OF( _LayoutConfigName_ ) \
   typedef typename _LayoutConfigName_::PREC PREC;             \
   static int const NDOFqObj = _LayoutConfigName_::LayoutType::NDOFqObj; \
   static int const NDOFuObj = _LayoutConfigName_::LayoutType::NDOFuObj; \
   typedef typename _LayoutConfigName_::VectorQObj VectorQObj;       \
   typedef typename _LayoutConfigName_::VectorUObj VectorUObj;       \
   typedef typename _LayoutConfigName_::MatrixQObjDyn MatrixQObjDyn;       \
   typedef typename _LayoutConfigName_::MatrixUObjDyn MatrixUObjDyn;       \
   typedef typename _LayoutConfigName_::MatrixQObjUObj MatrixQObjUObj; \
   typedef typename _LayoutConfigName_::Matrix44 Matrix44; \
   typedef typename _LayoutConfigName_::Matrix33 Matrix33; \
   typedef typename _LayoutConfigName_::Matrix43 Matrix43; \
   typedef typename _LayoutConfigName_::Matrix34 Matrix34; \
   typedef typename _LayoutConfigName_::Vector3 Vector3;   \
   typedef typename _LayoutConfigName_::Vector2 Vector2;   \
   typedef typename _LayoutConfigName_::Vector4 Vector4;   \
   typedef typename _LayoutConfigName_::Quaternion Quaternion; \
   typedef typename _LayoutConfigName_::VectorDyn VectorDyn;   \
   typedef typename _LayoutConfigName_::MatrixDyn MatrixDyn;   \
   typedef typename _LayoutConfigName_::MatrixDiagDyn MatrixDiagDyn;   \
   typedef typename _LayoutConfigName_::MatrixDynRow MatrixDynRow; \
   typedef typename _LayoutConfigName_::AffineTrafo AffineTrafo;

/**
* @brief This macro is used to typedef all template arguments outside of a class for a specific LayoutConfig.
*/
#define DEFINE_LAYOUT_CONFIG_TYPES_OF_OUTSIDE_TEMPLATE( _LayoutConfigName_ ) \
   typedef  _LayoutConfigName_::PREC PREC;             \
   static int const NDOFqObj = _LayoutConfigName_::LayoutType::NDOFqObj; \
   static int const NDOFuObj = _LayoutConfigName_::LayoutType::NDOFuObj; \
   typedef  _LayoutConfigName_::VectorQObj VectorQObj;       \
   typedef  _LayoutConfigName_::VectorUObj VectorUObj;       \
   typedef  _LayoutConfigName_::MatrixQObjUObj MatrixQObjUObj; \
   typedef  _LayoutConfigName_::MatrixQObjDyn MatrixQObjDyn;       \
   typedef  _LayoutConfigName_::MatrixUObjDyn MatrixUObjDyn;       \
   typedef  _LayoutConfigName_::Matrix44 Matrix44; \
   typedef  _LayoutConfigName_::Matrix33 Matrix33; \
   typedef  _LayoutConfigName_::Matrix43 Matrix43; \
   typedef  _LayoutConfigName_::Matrix34 Matrix34; \
   typedef  _LayoutConfigName_::Vector3 Vector3;   \
   typedef  _LayoutConfigName_::Vector2 Vector2;   \
   typedef  _LayoutConfigName_::Vector4 Vector4;   \
   typedef  _LayoutConfigName_::Quaternion Quaternion; \
   typedef  _LayoutConfigName_::VectorDyn VectorDyn;   \
   typedef  _LayoutConfigName_::MatrixDyn MatrixDyn;   \
   typedef  _LayoutConfigName_::MatrixDiagDyn MatrixDiagDyn;   \
   typedef  _LayoutConfigName_::MatrixDynRow MatrixDynRow; \
   typedef _LayoutConfigName_::AffineTrafo AffineTrafo;

#endif
