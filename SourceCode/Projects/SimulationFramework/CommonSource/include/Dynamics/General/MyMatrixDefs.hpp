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
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#include <Eigen/Eigenvalues>

// ================================================================================================
/** @brief This
*	These are some small matrix definitions.
*/
template<typename TPREC>
struct MyMatrix{
   using PREC = TPREC;
   //Static assigned Matrices
   using Matrix44 = Eigen::Matrix<PREC, 4, 4>;
   using Matrix43 = Eigen::Matrix<PREC, 4, 3>;
   using Matrix34 = Eigen::Matrix<PREC, 3, 4>;
   using Matrix33 = Eigen::Matrix<PREC, 3, 3>;
   using Vector3 = Eigen::Matrix<PREC, 3, 1>;
   using Vector2 = Eigen::Matrix<PREC, 2, 1>;
   using Quaternion = Eigen::Matrix<PREC, 4, 1>;
   using Vector4 = Eigen::Matrix<PREC, 4, 1>;
   using Vector6 = Eigen::Matrix<PREC, 6, 1>;
   using VectorDyn = Eigen::Matrix<PREC, Eigen::Dynamic , 1 >                   ;
   using MatrixDyn = Eigen::Matrix<PREC, Eigen::Dynamic , Eigen::Dynamic >      ;
   using MatrixDiagDyn = Eigen::DiagonalMatrix<PREC, Eigen::Dynamic >               ;
   using MatrixDynRow = Eigen::Matrix<PREC, Eigen::Dynamic , Eigen::Dynamic, Eigen::RowMajor>;

   using AffineTrafo = Eigen::Transform<PREC,3,Eigen::TransformTraits::Affine>;

   using MatrixSparse = Eigen::SparseMatrix<PREC>   ;       // declares a column-major sparse matrix of type PREC
   using MatrixSparseTriplet = Eigen::Triplet<PREC>        ;

   template<typename EigenType> using MatrixRef = Eigen::Ref<EigenType>;

};


struct MyMatrixDecomposition{;

    template<typename TMatrix>
    using EigenSolverSelfAdjoint = Eigen::SelfAdjointEigenSolver<TMatrix> ;

};

/**
* @brief This macro is used to typedef all custom matrix types which have nothing to do with the system.
*/
#define DEFINE_MATRIX_TYPES_OF( _PREC_ ) \
   using Matrix44 = typename MyMatrix< _PREC_ >::Matrix44; \
   using Matrix33 = typename MyMatrix< _PREC_ >::Matrix33; \
   using Matrix43 = typename MyMatrix< _PREC_ >::Matrix43; \
   using Matrix34 = typename MyMatrix< _PREC_ >::Matrix34; \
   using Vector3 = typename MyMatrix< _PREC_ >::Vector3;   \
   using Vector2 = typename MyMatrix< _PREC_ >::Vector2;   \
   using Vector4 = typename MyMatrix< _PREC_ >::Vector4;   \
   using Vector6 = typename MyMatrix< _PREC_ >::Vector6;   \
   using Quaternion = typename MyMatrix< _PREC_ >::Quaternion; \
   using VectorDyn = typename MyMatrix< _PREC_ >::VectorDyn; \
   using MatrixDyn = typename MyMatrix< _PREC_ >::MatrixDyn; \
   using MatrixDiagDyn = typename MyMatrix< _PREC_ >::MatrixDiagDyn; \
   using MatrixDynRow = typename MyMatrix< _PREC_ >::MatrixDynRow; \
   using AffineTrafo = typename MyMatrix< _PREC_ >::AffineTrafo; \
   using MatrixSparse = typename MyMatrix< _PREC_ >::MatrixSparse; \
   using MatrixSparseTriplet = typename MyMatrix< _PREC_ >::MatrixSparseTriplet;
#endif

