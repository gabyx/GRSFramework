/*
 *  MyMatrixDefs.hpp
 *
 *  Created by Gabriel NÃ¼tzi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef MyMatrixDefs_hpp
#define MyMatrixDefs_hpp

#include <Eigen/Dense>

// ================================================================================================
/** @brief This
*	These are some small matrix definitions.
*/
template<typename TPREC>
struct MyMatrix{
   typedef TPREC PREC;
   //Static assigned Matrices
   typedef Eigen::Matrix<PREC, 4, 4> Matrix44;
   typedef Eigen::Matrix<PREC, 4, 3> Matrix43;
   typedef Eigen::Matrix<PREC, 3, 4> Matrix34;
   typedef Eigen::Matrix<PREC, 3, 3> Matrix33;
   typedef Eigen::Matrix<PREC, 3, 1> Vector3;
   typedef Eigen::Matrix<PREC, 2, 1> Vector2;
   typedef Eigen::Matrix<PREC, 4, 1> Quaternion;
   typedef Eigen::Matrix<PREC, 4, 1> Vector4;
   typedef Eigen::Matrix<PREC, Eigen::Dynamic , 1 >                    VectorDyn;
   typedef Eigen::Matrix<PREC, Eigen::Dynamic , Eigen::Dynamic >       MatrixDyn;
   typedef Eigen::DiagonalMatrix<PREC, Eigen::Dynamic >                MatrixDiagDyn;
   typedef Eigen::Matrix<PREC, Eigen::Dynamic , Eigen::Dynamic, Eigen::RowMajor> MatrixDynRow;
};

/**
* @brief This macro is used to typedef all custom matrix types which have nothing to do with the system.
*/
#define DEFINE_MATRIX_TYPES \
   typedef typename MyMatrix< PREC >::Matrix44 Matrix44; \
   typedef typename MyMatrix< PREC >::Matrix33 Matrix33; \
   typedef typename MyMatrix< PREC >::Matrix43 Matrix43; \
   typedef typename MyMatrix< PREC >::Matrix34 Matrix34; \
   typedef typename MyMatrix< PREC >::Vector3 Vector3;   \
   typedef typename MyMatrix< PREC >::Vector2 Vector2;   \
   typedef typename MyMatrix< PREC >::Vector4 Vector4;   \
   typedef typename MyMatrix< PREC >::Quaternion Quaternion; \
   typedef typename MyMatrix< PREC >::VectorDyn VectorDyn; \
   typedef typename MyMatrix< PREC >::MatrixDyn MatrixDyn; \
   typedef typename MyMatrix< PREC >::MatrixDiagDyn MatrixDiagDyn; \
   typedef typename MyMatrix< PREC >::MatrixDynRow MatrixDynRow;


#endif
