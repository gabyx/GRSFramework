// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <Eigen/CXX11/Tensor>
#include <iostream>
#include <memory>
#include <type_traits>

#include <meta/meta.hpp>
//
//
using namespace Eigen;

// template<typename T>
// void print(T & t){
//    std::cout << t << std::endl;
//}
//
// template<typename... T>
// void printIndices(const T&... t){
//    int a[] = { (print(t) , 0)...  };
//}
//
// template<unsigned int N>
// struct Print{
//    template<typename... T, typename Vec>
//    static void apply(Vec & v , T&...i ){
//        Print<N-1>::apply( v, v(N), i... );
//    }
//};
//
// template<>
// struct Print<0>{
//
//    template<typename... T, typename Vec>
//    static void apply(Vec & v, T&...i){
//        printIndices(v(0),i...);
//    }
//};
//
// template<typename T>
// void printIndicesW(const T& t){
//    const unsigned int N = T::RowsAtCompileTime;
//    using IntRange = meta::integer_range<unsigned int,0,N-1>;
//    Print<N-1>::apply(t);
//}
//
// template<typename Derived, int Accc>
// void foo(TensorBase<Derived,Accc> & b){
//    b;
//}
//
//
// template<typename Derived>
// void foo2(MatrixBase<Derived> & b){
//
//}

template <typename TFileGroup,
          typename Tensor,
          typename std::enable_if<std::is_arithmetic<typename Tensor::Scalar>::value, int>::type = 0>
void gurke(const TFileGroup& fg, const TensorRef<Tensor>& m)
{
}

template <typename TFileGroup,
          typename Tensor,
          typename std::enable_if<!std::is_arithmetic<typename Tensor::Scalar>::value, int>::type = 0>
void gurke(const TFileGroup& fg, const TensorRef<Tensor>& m)
{
}

void eigenTensorTest()
{
    //    Matrix<Vector3d,3,3> o;
    //    foo2(o);
    //
    //    using TT = Tensor<double, 3>;
    //    using TTMap = TensorMap<TT>;
    //
    //    TT a(4,4,4);
    //
    //    for(int i = 0; i< a.size() ; ++i){
    //        *(a.data()+i) = i;
    //    }
    //    std::cout << a.size() << std::endl;
    //
    //
    //  TT slice1(4,4,1);
    //  DSizes<ptrdiff_t, 3> indices(0,0,0);
    //  Matrix<ptrdiff_t,3,1> ind(0,0,0);
    //  DSizes<ptrdiff_t, 3> sizes(4,4,1);
    //  slice1 = a.slice(indices, sizes);
    //  std::cout << slice1 << TT::Layout <<  std::endl;
    //
    //  std::cout << a.chip<2>(3) << std::endl;
    //
    //  std::cout << typeid(slice1.dimensions()).name() <<std::endl;
    //
    //  auto buf = new double[32];
    //  for(int i = 0; i< 32 ; ++i){
    //        buf[i] = i;
    //  }
    //
    //
    //  TensorMap< Tensor<double,3> > map(buf,1,2,3);
    //  std::cout << map << std::endl;
    //
    //  std::unique_ptr<TTMap> m;
    //

    //    printIndicesW(indices2);
    //
    //
    //    using T = Tensor<double, 3>;
    //    T tensor(5,5,5);
    //    int a;
    //    gurke(a,TensorRef<T>(tensor));
    //
    //
    //    using CustomIndex = Matrix<unsigned int,3,1>;
    //    CustomIndex indicesC(3,1,2);
    //
    //    using NormalIndex = DSizes<ptrdiff_t, 3>;
    //    NormalIndex indices(3,1,2);
    //
    //
    //    EIGEN_ASM_COMMENT("Normal INDEX");
    //    auto s = tensor.coeff(indices);
    //    EIGEN_ASM_COMMENT("Normal INDEX END");
    //    std::cout << s << std::endl;
    //    EIGEN_ASM_COMMENT("CUSTOM INDEX");
    //    auto s2 = tensor.coeff(indicesC);
    //    EIGEN_ASM_COMMENT("CUSTOM INDEX END");
    //    std::cout << s << s2 << std::endl;
    //
    //    {
    //      using CustomIndex = Matrix<unsigned int,4,1>;
    //      Tensor<float, 4> tensor(2,3,5,7);
    //      tensor.setRandom();
    //      CustomIndex strides;
    //      strides[0] = 1;
    //      strides[1] = 1;
    //      strides[2] = 1;
    //      strides[3] = 1;
    //
    //      Tensor<float, 4> no_stride;
    //      no_stride = tensor.stride(strides);
    //    }
    {
        //         PlainObjectBase<Eigen::Matrix<double, 2, 1, 0, 2, 1> > f;
        //         f.resize(0);
        using S    = Matrix3d;
        using T    = Array<S, Eigen::Dynamic, Eigen::Dynamic>;
        using TRef = Ref<T>;

        T a(3, 3);
        T b(3, 3);
        T c = a * b;
        std::cout << a * b;
        ////
        //         using T = Tensor<S, 2>;
        //         using TRef = TensorRef<T>;
        //
        //         T t(3,3);
        //         t.resize(2,2);
        //         TRef ref(t);
        ////
        //         ref.coeffRef(1,1) = S(1,2,3,4);
        //         std::cout << ref(1,1) << std::endl;
    }

    //    {
    //      using CustomIndex = Matrix<unsigned int,2,1>;
    //      Tensor<float, 4> tensor(2, 3, 5, 7);
    //  tensor.setRandom();
    //  CustomIndex reduction_axis2;
    //  reduction_axis2[0] = 1;
    //  reduction_axis2[1] = 3;
    //
    //  Tensor<float, 2> result = tensor.sum(reduction_axis2);
    //    }
}
