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
struct DynamicLayout {
   static int const NDOFq = -1;
   static int const NDOFu = -1;
   static int const NDOFqObj = nDOFqObj;
   static int const NDOFuObj = nDOFuObj;
};

/**
* @brief A static layout specialization.
*/
template <int NObjects, int nDOFqObj, int nDOFuObj>
struct StaticLayout {
   static int const NDOFqObj = nDOFqObj;
   static int const NDOFuObj = nDOFuObj;
   static int const NDOFq = NObjects*NDOFqObj;
   static int const NDOFu = NObjects*NDOFuObj;
};

// ================================================================================================
/**
* @brief This is the LayoutConfig which specifies the layout of the system.
*/
template <typename TPREC, typename TLayout>
struct LayoutConfig{
   typedef TPREC PREC;
   typedef TLayout LayoutType;

   // Dynamic or Static Vectors/Matrices
   typedef Eigen::Matrix<PREC, LayoutType::NDOFq, LayoutType::NDOFq>           MatrixQQ;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFu, LayoutType::NDOFu>           MatrixUU;
   typedef Eigen::DiagonalMatrix<PREC, LayoutType::NDOFq>                  MatrixDiagQQ;
   typedef Eigen::DiagonalMatrix<PREC, LayoutType::NDOFu>                  MatrixDiagUU;

   typedef Eigen::Matrix<PREC, LayoutType::NDOFq, LayoutType::NDOFu>           MatrixQU;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFu, LayoutType::NDOFq>           MatrixUQ;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFu, Eigen::Dynamic >         MatrixUDyn;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFq, Eigen::Dynamic >         MatrixQDyn;

   // Dynamic or Static  Vectors/Matrices
   typedef Eigen::Matrix<PREC, LayoutType::NDOFq, 1>                       VectorQ;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFu, 1>                       VectorU;

   // Static  Vectors/Matrices
   typedef Eigen::Matrix<PREC, LayoutType::NDOFqObj, LayoutType::NDOFuObj>     MatrixQObjUObj;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFqObj, 1>                    VectorQObj;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFuObj, 1>                    VectorUObj;

   // Static Vectors/Matrices
   typedef typename MyMatrix< PREC >::Matrix44 Matrix44;
   typedef typename MyMatrix< PREC >::Matrix33 Matrix33;
   typedef typename MyMatrix< PREC >::Matrix43 Matrix43;
   typedef typename MyMatrix< PREC >::Matrix34 Matrix34;
   typedef typename MyMatrix< PREC >::Vector2 Vector2;
   typedef typename MyMatrix< PREC >::Vector3 Vector3;
   typedef typename MyMatrix< PREC >::Vector4 Vector4;
   typedef typename MyMatrix< PREC >::Quaternion Quaternion;
   typedef typename MyMatrix< PREC >::VectorDyn VectorDyn;
   typedef typename MyMatrix< PREC >::MatrixDyn MatrixDyn;
   typedef typename MyMatrix< PREC >::MatrixDiagDyn MatrixDiagDyn;
   typedef typename MyMatrix< PREC >::MatrixDynRow MatrixDynRow;

   typedef Eigen::Transform<PREC,3,Eigen::TransformTraits::Affine> AffineTrafo;

};



/**
* @brief This macro is used to typedef all template arguments in a class with e.g template argument typename â€œLayoutConfigâ€
*/
#define DEFINE_LAYOUT_CONFIG_TYPES_OF( _LayoutConfigName_ ) \
   typedef typename _LayoutConfigName_::PREC PREC;             \
   static int const NDOFq = _LayoutConfigName_::LayoutType::NDOFq; \
   static int const NDOFu = _LayoutConfigName_::LayoutType::NDOFu; \
   static int const NDOFqObj = _LayoutConfigName_::LayoutType::NDOFqObj; \
   static int const NDOFuObj = _LayoutConfigName_::LayoutType::NDOFuObj; \
   typedef typename _LayoutConfigName_::MatrixQU MatrixQU;     \
   typedef typename _LayoutConfigName_::MatrixQQ MatrixQQ;     \
   typedef typename _LayoutConfigName_::MatrixUU MatrixUU;     \
   typedef typename _LayoutConfigName_::MatrixUQ MatrixUQ;     \
   typedef typename _LayoutConfigName_::MatrixDiagUU MatrixDiagUU;     \
   typedef typename _LayoutConfigName_::MatrixDiagQQ MatrixDiagQQ;     \
   typedef typename _LayoutConfigName_::MatrixQDyn MatrixQDyn; \
   typedef typename _LayoutConfigName_::MatrixUDyn MatrixUDyn; \
   typedef typename _LayoutConfigName_::VectorQ VectorQ;       \
   typedef typename _LayoutConfigName_::VectorU VectorU;             \
   typedef typename _LayoutConfigName_::VectorQObj VectorQObj;       \
   typedef typename _LayoutConfigName_::VectorUObj VectorUObj;       \
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
   static int const NDOFq = _LayoutConfigName_::LayoutType::NDOFq; \
   static int const NDOFu = _LayoutConfigName_::LayoutType::NDOFu; \
   static int const NDOFqObj = _LayoutConfigName_::LayoutType::NDOFqObj; \
   static int const NDOFuObj = _LayoutConfigName_::LayoutType::NDOFuObj; \
   typedef  _LayoutConfigName_::MatrixQU MatrixQU;     \
   typedef  _LayoutConfigName_::MatrixQQ MatrixQQ;     \
   typedef  _LayoutConfigName_::MatrixUU MatrixUU;     \
   typedef  _LayoutConfigName_::MatrixUQ MatrixUQ;     \
   typedef  _LayoutConfigName_::MatrixDiagUU MatrixDiagUU;     \
   typedef  _LayoutConfigName_::MatrixDiagQQ MatrixDiagQQ;     \
   typedef  _LayoutConfigName_::MatrixQDyn MatrixQDyn; \
   typedef  _LayoutConfigName_::MatrixUDyn MatrixUDyn; \
   typedef  _LayoutConfigName_::VectorQ VectorQ;       \
   typedef  _LayoutConfigName_::VectorU VectorU;             \
   typedef  _LayoutConfigName_::VectorQObj VectorQObj;       \
   typedef  _LayoutConfigName_::VectorUObj VectorUObj;       \
   typedef  _LayoutConfigName_::MatrixQObjUObj MatrixQObjUObj; \
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
   typedef  _LayoutConfigName_::MatrixDynRow MatrixDynRow;

#endif
