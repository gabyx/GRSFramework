/*
 *  GRSF/Dynamics/General/MyMatrixDefs.hpp
 *
 *  Created by Gabriel NÃ¼tzi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef GRSF_Dynamics_General_MyMatrixDefs_hpp
#define GRSF_Dynamics_General_MyMatrixDefs_hpp

//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <vector>
#include <map>
#include <unordered_map>
//#include <Eigen/StdVector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#include <Eigen/Eigenvalues>

#include <Eigen/CXX11/Tensor>

// ================================================================================================
/** @brief This
*	These are some small matrix definitions.
*/
template<typename TScalar>
struct MyMatrix {
    using PREC = TScalar;
    //Static assigned Matrices
    using Matrix44 = Eigen::Matrix<PREC, 4, 4>;
    using Matrix43 = Eigen::Matrix<PREC, 4, 3>;
    using Matrix34 = Eigen::Matrix<PREC, 3, 4>;
    using Matrix33 = Eigen::Matrix<PREC, 3, 3>;
    using Matrix32 = Eigen::Matrix<PREC, 3, 2>;
    using Matrix23 = Eigen::Matrix<PREC, 2, 3>;
    using Matrix22 = Eigen::Matrix<PREC, 2, 2>;
    using Vector3 = Eigen::Matrix<PREC, 3, 1>;
    using Vector2 = Eigen::Matrix<PREC, 2, 1>;

    using Quaternion = Eigen::Quaternion<PREC>;
    using AngleAxis = Eigen::AngleAxis<PREC>;

    using Vector4 = Eigen::Matrix<PREC, 4, 1>;
    using Vector6 = Eigen::Matrix<PREC, 6, 1>;
    using VectorDyn = Eigen::Matrix<PREC, Eigen::Dynamic , 1 >                   ;


    using MatrixDynDyn = Eigen::Matrix<PREC, Eigen::Dynamic , Eigen::Dynamic >      ;
    using MatrixDiagDyn = Eigen::DiagonalMatrix<PREC, Eigen::Dynamic >               ;
    using MatrixDynDynRow = Eigen::Matrix<PREC, Eigen::Dynamic , Eigen::Dynamic, Eigen::RowMajor>;

    template<unsigned int M>
    using MatrixStatDyn = Eigen::Matrix<PREC, M, Eigen::Dynamic >;
    template<unsigned int N>
    using MatrixDynStat = Eigen::Matrix<PREC, Eigen::Dynamic, N >;

    template<unsigned int M, unsigned int N>
    using MatrixStatStat = Eigen::Matrix<PREC, M, N >;

    template<unsigned int M>
    using VectorStat = Eigen::Matrix<PREC, M, 1 >;

    using AffineTrafo = Eigen::Transform<PREC,3,Eigen::TransformTraits::Affine>;
    using AffineTrafo2d = Eigen::Transform<PREC,2,Eigen::TransformTraits::Affine>;

    using MatrixSparse = Eigen::SparseMatrix<PREC>   ;       // declares a column-major sparse matrix of type PREC
    using MatrixSparseTriplet = Eigen::Triplet<PREC>        ;



    template<unsigned int M>
    using ArrayStatDyn = Eigen::Array<PREC, M, Eigen::Dynamic >;
    template<unsigned int N>
    using ArrayDynStat = Eigen::Array<PREC, Eigen::Dynamic, N >;
    template<unsigned int M, unsigned int N>
    using ArrayStatStat = Eigen::Array<PREC, M, N >;
    template<unsigned int M>
    using ArrayStat = Eigen::Array<PREC, M,1>;

    using Array3 = Eigen::Array<PREC, 3, 1>;
    using Array2 = Eigen::Array<PREC, 2, 1>;


    // Tensor stuff (unsupported eigen3)
    template<std::size_t Indices, int Options = Eigen::ColMajor>
    using TensorDyn = Eigen::Tensor<PREC,Indices,Options>;
};

struct MyMatrixSpecial{

    template<typename Derived> using MatrixBase = Eigen::MatrixBase<Derived>;
    template<typename Derived> using ArrayBase = Eigen::ArrayBase<Derived>;

    template<typename Derived>                 using VectorBDyn     = Eigen::VectorBlock<Derived,Eigen::Dynamic>;
    template<typename Derived, unsigned int M> using VectorBStat    = Eigen::VectorBlock<Derived,M>;

    template<typename Derived>                 using MatrixBDynDyn  = Eigen::Block<Derived>;
    template<typename Derived, unsigned int M> using MatrixBStatDyn = Eigen::Block<Derived,M, Eigen::Dynamic>;
    template<typename Derived, unsigned int N> using MatrixBDynStat = Eigen::Block<Derived,Eigen::Dynamic,N>;

    template<typename EigenType> using MatrixRef = Eigen::Ref<EigenType>;

    template<typename EigenType> using MatrixMap = Eigen::Map<EigenType>;

    // Tensor stuff (unsupported eigen3)
    template<typename EigenType>
    using TensorMap  = Eigen::TensorMap<EigenType>;

    template<typename EigenType>
    using TensorBaseReadOnly = Eigen::TensorBase<EigenType,Eigen::ReadOnlyAccessors>;
    template<typename EigenType>
    using TensorBase = Eigen::TensorBase<EigenType,Eigen::WriteAccessors>;
};

struct MyMatrixStorageOptions{
    // Eigen Options
    static const int MatrixRowMajorOption  = Eigen::RowMajor;
    static const int MatrixColMajorOption = Eigen::ColMajor;
};

struct MyMatrixDecomposition {

    template<typename TMatrix>
    using EigenSolverSelfAdjoint = Eigen::SelfAdjointEigenSolver<TMatrix> ;

};

struct MyMatrixIOFormat {
    static Eigen::IOFormat Matlab;
    static Eigen::IOFormat CommaSep;
    static Eigen::IOFormat SpaceSep;
};

#define DEFINE_MATRIX_STORAGEOPTIONS \
    static const auto MatrixRowMajorOption = MyMatrixStorageOptions::MatrixRowMajorOption; \
    static const auto MatrixColMajorOption = MyMatrixStorageOptions::MatrixColMajorOption;

#define DEFINE_MATRIX_SPECIALTYPES \
   template<typename Derived> using MatrixBase = typename MyMatrixSpecial::MatrixBase<Derived>; \
   template<typename Derived> using ArrayBase  = typename MyMatrixSpecial::template ArrayBase<Derived>; \
   \
   template<typename Derived> using VectorBDyn = typename MyMatrixSpecial::VectorBDyn<Derived>; \
   template<typename Derived,unsigned int M> using VectorBStat = typename MyMatrixSpecial::VectorBStat<Derived,M>; \
   \
   template<typename Derived> using MatrixBDynDyn = typename MyMatrixSpecial::MatrixBDynDyn<Derived>; \
   template<typename Derived, unsigned int N> using MatrixBDynStat = typename MyMatrixSpecial::MatrixBDynStat<Derived,N>; \
   template<typename Derived, unsigned int M> using MatrixBStatDyn = typename MyMatrixSpecial::MatrixBStatDyn<Derived,M>; \
    \
   template<typename EigenType> using MatrixRef = typename MyMatrixSpecial::MatrixRef< EigenType >; \
   template<typename EigenType> using MatrixMap = typename MyMatrixSpecial::MatrixMap< EigenType >; \
   \
   template<typename T> \
   using TensorMap = typename MyMatrixSpecial::template TensorMap<T>; \
   template<typename T> \
   using TensorBase = typename MyMatrixSpecial::template TensorBase<T>; \
   template<typename T> \
   using TensorBaseReadOnly = typename MyMatrixSpecial::template TensorBaseReadOnly<T>;

/**
* @brief This macro is used to typedef all custom matrix types which have nothing to do with the system.
*/
#define DEFINE_MATRIX_TYPES_OF( _PREC_ ) \
   using Matrix44 = typename MyMatrix< _PREC_ >::Matrix44; \
   using Matrix33 = typename MyMatrix< _PREC_ >::Matrix33; \
   using Matrix22 = typename MyMatrix< _PREC_ >::Matrix22; \
   using Matrix32 = typename MyMatrix< _PREC_ >::Matrix32; \
   using Matrix23 = typename MyMatrix< _PREC_ >::Matrix23; \
   using Matrix43 = typename MyMatrix< _PREC_ >::Matrix43; \
   using Matrix34 = typename MyMatrix< _PREC_ >::Matrix34; \
   using Vector3 = typename MyMatrix< _PREC_ >::Vector3;   \
   using Vector2 = typename MyMatrix< _PREC_ >::Vector2;   \
   using Vector4 = typename MyMatrix< _PREC_ >::Vector4;   \
   using Vector6 = typename MyMatrix< _PREC_ >::Vector6;   \
   using Quaternion = typename MyMatrix< _PREC_ >::Quaternion; \
   using AngleAxis = typename MyMatrix< _PREC_ >::AngleAxis; \
   using VectorDyn = typename MyMatrix< _PREC_ >::VectorDyn; \
   using MatrixDynDyn = typename MyMatrix< _PREC_ >::MatrixDynDyn; \
   using MatrixDiagDyn = typename MyMatrix< _PREC_ >::MatrixDiagDyn; \
   using MatrixDynDynRow = typename MyMatrix< _PREC_ >::MatrixDynDynRow; \
   \
   template<unsigned int M> using MatrixStatDyn = typename MyMatrix< _PREC_ >::template MatrixStatDyn<M>; \
   template<unsigned int N> using MatrixDynStat = typename MyMatrix< _PREC_ >::template MatrixDynStat<N>; \
   template<unsigned int M,unsigned int N> using MatrixStatStat = typename MyMatrix< _PREC_ >::template MatrixStatStat<M,N>; \
   template<unsigned int M> using VectorStat = typename MyMatrix< _PREC_ >::template VectorStat<M>; \
   \
   using AffineTrafo = typename MyMatrix< _PREC_ >::AffineTrafo; \
   using AffineTrafo2d = typename MyMatrix< _PREC_ >::AffineTrafo2d; \
   using MatrixSparse = typename MyMatrix< _PREC_ >::MatrixSparse; \
   using MatrixSparseTriplet = typename MyMatrix< _PREC_ >::MatrixSparseTriplet; \
   \
   template<unsigned int M> using ArrayStatDyn = typename MyMatrix< _PREC_ >::template ArrayStatDyn<M>; \
   template<unsigned int N> using ArrayDynStat = typename MyMatrix< _PREC_ >::template ArrayDynStat<N>; \
   template<unsigned int M,unsigned int N> using ArrayStatStat = typename MyMatrix< _PREC_ >::template ArrayStatStat<M,N>; \
   template<unsigned int M> using ArrayStat = typename MyMatrix< _PREC_ >::template ArrayStat<M>; \
   using Array3 = typename MyMatrix< _PREC_ >::Array3;   \
   using Array2 = typename MyMatrix< _PREC_ >::Array2; \
   \
   template<unsigned int Indices, int Options = Eigen::ColMajor> \
   using TensorDyn = typename MyMatrix< _PREC_ >::template TensorDyn<Indices,Options>; \
   \
   DEFINE_MATRIX_SPECIALTYPES

#endif

