#ifndef GRSF_Common_Hdf5Helpers_hpp
#define GRSF_Common_Hdf5Helpers_hpp

#include <type_traits>

#include <H5Cpp.h>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/SfinaeMacros.hpp"

#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"

namespace Hdf5Helpers
{

DEFINE_LAYOUT_CONFIG_TYPES

namespace detail
{

    template<typename T>
    H5::PredType getNativeType_imp();

    template<>
    inline H5::PredType getNativeType_imp<float>()
    {
        return H5::PredType::NATIVE_FLOAT;
    };

    template<>
    inline H5::PredType getNativeType_imp<double>()
    {
        return H5::PredType::NATIVE_DOUBLE;
    };

    template<>
    inline H5::PredType getNativeType_imp<bool>()
    {
        return H5::PredType::NATIVE_UCHAR;
    };


    template<>
    inline H5::PredType getNativeType_imp<char>()
    {
        return H5::PredType::NATIVE_CHAR;
    };

    template<>
    inline H5::PredType getNativeType_imp<unsigned char>()
    {
        return H5::PredType::NATIVE_UCHAR;
    };

    template<>
    inline H5::PredType getNativeType_imp<int>()
    {
        return H5::PredType::NATIVE_INT;
    };

    template<>
    inline H5::PredType getNativeType_imp<unsigned int>()
    {
        return H5::PredType::NATIVE_UINT;
    };

    template<>
    inline H5::PredType getNativeType_imp<unsigned long int>()
    {
        return H5::PredType::NATIVE_ULONG;
    };

    template<>
    inline H5::PredType getNativeType_imp<long long int>()
    {
        return H5::PredType::NATIVE_LLONG;
    };
}

/** Get native types */
template<typename T>
inline H5::PredType getNativeType()
{
   return detail::getNativeType_imp< typename std::remove_reference<
                                            typename std::remove_const<T>::type
                                           >::type
                                    >();
}


namespace detail
{
    /** Map native types To little endian ones*/
    template<typename T, std::size_t N = sizeof(T) >
    H5::PredType mapNativeTypeToLE_imp();


    template<>
    inline H5::PredType mapNativeTypeToLE_imp<float, 4 >()
    {
        return H5::PredType::IEEE_F32LE;
    };

    template<>
    inline H5::PredType mapNativeTypeToLE_imp<double, 8 >()
    {
        return H5::PredType::IEEE_F64LE;
    };


    template<>
    inline H5::PredType mapNativeTypeToLE_imp<bool, 1 >()
    {
        return H5::PredType::STD_U8LE;
    };


    template<>
    inline H5::PredType mapNativeTypeToLE_imp<char, 1 >()
    {
        return H5::PredType::STD_I8LE;
    };

    template<>
    inline H5::PredType mapNativeTypeToLE_imp<unsigned char, 1 >()
    {
        return H5::PredType::STD_U8LE;
    };

    template<>
    inline H5::PredType mapNativeTypeToLE_imp<int, 4 >()
    {
        return H5::PredType::STD_I32LE;
    };

    template<>
    inline H5::PredType mapNativeTypeToLE_imp<unsigned int, 4 >()
    {
        return H5::PredType::STD_U32LE;
    };

    template<>
    inline H5::PredType mapNativeTypeToLE_imp<unsigned long int, 8>()
    {
        return H5::PredType::STD_U64LE;
    };

    template<>
    inline H5::PredType mapNativeTypeToLE_imp<long long int, 8 >()
    {
        return H5::PredType::STD_U64LE;
    };
}

/** Map native types To little endian ones*/
template<typename T, std::size_t N = sizeof(T) >
inline H5::PredType mapNativeTypeToLE()
{
    return detail::mapNativeTypeToLE_imp< typename std::remove_reference<
                                            typename std::remove_const<T>::type
                                        >::type, N>();
}



/** Write simple attribute */
template<typename TFileGroupData, typename T>
inline void saveAttribute(const TFileGroupData & fg, const T & attr, std::string name)
{
    hsize_t dims=1;
    H5::DataSpace d(1, &dims /*dimension*/);
    H5::Attribute a = fg.createAttribute(name, Hdf5Helpers::mapNativeTypeToLE<T>(),d );
    a.write( Hdf5Helpers::getNativeType<T>(), &attr );
}




namespace details
{

template<typename TFileGroup, typename Scalar>
inline H5::DataSet saveArithmeticArray(const TFileGroup & fg,
        Scalar * data,
        H5::DataSpace & dataSpace,
        std::string name)
{
    STATIC_ASSERTM(std::is_arithmetic<Scalar>::value, "Your scalar is not arithmetic!")
    H5::DataSet s = fg.createDataSet(name, Hdf5Helpers::mapNativeTypeToLE<Scalar>(), dataSpace);
    s.write(data, Hdf5Helpers::getNativeType<Scalar>() );
    return s;
}

template<typename Derived,
        typename TFileGroup,
        typename T
        >
void saveMatrixOrArray_artihmScalar(const TFileGroup & fg, const T & m, std::string name)
{
    /* Derived = Matrix<Scalar,...> or similar */
    /* Scalar = arithmetic */
    hsize_t dims[2] = {static_cast<hsize_t>(m.rows()),
                       static_cast<hsize_t>(m.cols())
                      };

    H5::DataSpace dataSpace(2, dims);

    H5::DataSet s = saveArithmeticArray(fg, m.derived().data(),dataSpace,name);

    // Hdf5 writes in row-major order, so if matrix is col-major (as default in eigen)
    // we mark an atttribute which tells if this matrix should be transformed to colmajor storage after read in the file
    if(dims[1]!=1)
    {
        saveAttribute(s, !static_cast<char>(Derived::Flags & Eigen::RowMajorBit), "convertToColMajor");
    }
}

template<typename Tensor, typename TFileGroup>
void saveTensor_artihmScalar(const TFileGroup & fg, const TensorRef<Tensor> & m, std::string name)
{
    /* Tensor = Tensor<Scalar,...> or similar */
    /* Scalar  = arithmetic */
    using Scalar = typename Tensor::Scalar;
    STATIC_ASSERTM(std::is_arithmetic<Scalar>::value, "Your scalar of the Vector::Scalar is not arithmetic!")

    auto & d = m.dimensions();
    std::array<hsize_t, Tensor::NumIndices > dims;
    for( std::size_t i=0;  i < Tensor::NumIndices;  ++i)
    {
        dims[i] = static_cast<hsize_t>(d[i]);
    }

    H5::DataSpace dataSpace(Tensor::NumIndices, &dims[0]);

    H5::DataSet s = saveArithmeticArray(fg, m.data(),dataSpace,name);

    // Hdf5 writes in row-major order, so if matrix is col-major (as default in eigen)
    // we mark an atttribute which tells if this matrix should be transformed to colmajor storage after read in the file

    saveAttribute(s, !static_cast<char>(Tensor::Layout & Eigen::RowMajorBit), "convertToColMajor");

}

template<typename Tensor, typename TFileGroup>
void saveTensor_vectorScalar(const TFileGroup & fg, const TensorRef<Tensor> & m, std::string name)
{
    /* Tensor = Tensor<Scalar,...> or similar */
    /* Scalar  = VectorStat<N> , fixed size vector */
    static const unsigned int RowsAtCompileTime = Tensor::Scalar::RowsAtCompileTime;
    using Scalar = typename Tensor::Scalar::Scalar;

    STATIC_ASSERTM(std::is_arithmetic<Scalar>::value, "Your scalar of the Vector::Scalar is not arithmetic!")

    auto & d = m.dimensions();
    std::array<hsize_t, Tensor::NumIndices + 1 > dims;
    for( std::size_t i=0;  i < Tensor::NumIndices;  ++i)
    {
        dims[i] = static_cast<hsize_t>(d[i]);
    }
    dims[Tensor::NumIndices + 1] = RowsAtCompileTime;

    H5::DataSpace dataSpace(Tensor::NumIndices+1, &dims[0]);

    H5::DataSet s = saveArithmeticArray(fg, m.data()->data(),dataSpace,name);

    // Hdf5 writes in row-major order, so if matrix is col-major (as default in eigen)
    // we mark an atttribute which tells if this matrix should be transformed to colmajor storage after read in the file
    saveAttribute(s, !static_cast<char>(Tensor::Layout & Eigen::RowMajorBit), "convertToColMajor");

}

}

/** Vector and Matrix Saving */
template<typename TFileGroup,
        typename Derived,
        SFINAE_ENABLE_IF( std::is_arithmetic<typename Derived::Scalar>::value )
        >
void saveData(const TFileGroup & fg, const MatrixBase<Derived> & m, std::string name)
{
    details::saveMatrixOrArray_artihmScalar<Derived>(fg,m,name);
}
template<typename TFileGroup,
        typename Derived,
        SFINAE_ENABLE_IF( std::is_arithmetic<typename Derived::Scalar>::value )
        >
void saveData(const TFileGroup & fg, const ArrayBase<Derived> & m, std::string name)
{
    details::saveMatrixOrArray_artihmScalar<Derived>(fg,m,name);
}

/** Tensor saving, we accept here only the Tensor sub class, not TensorRef becaus TensorRef of Tensor with fixed size vector type as Scalar
*   has some non fixed bug (since TensorRef<Tensor<Vector3>>> is not yet fully supported, Tensor<Vector3>> however works.
*   Bug : http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1082
*/

template<typename TFileGroup,
        typename TScalar,
        int Options,
        SFINAE_ENABLE_IF( std::is_arithmetic<TScalar>::value )
        >
void saveData(const TFileGroup & fg, const TensorDyn<TScalar,Options> & m, std::string name)
{
    details::saveTensor_artihmScalar(fg,m,name);
}

template<typename TFileGroup,
        typename TScalar,
        int Options,
        SFINAE_ENABLE_IF( !std::is_arithmetic<TScalar>::value &&
                                TScalar::IsVectorAtCompileTime
                                             )
        >
void saveData(const TFileGroup & fg, const TensorDyn<TScalar,Options> & m, std::string name)
{
    details::saveTensor_vectorScalar(fg,m,name);
}


template<typename TFileGroup,
        typename Derived,
        SFINAE_ENABLE_IF( !std::is_arithmetic<typename Derived::Scalar>::value )
        >
void saveData(const TFileGroup & fg, const MatrixBase<Derived> & m, std::string name)
{

    ERRORMSG("NOT IMPLEMENTED")
}


/** AABB saving*/
template<typename TFileGroup,
        typename T,
        SFINAE_ENABLE_IF((std::is_same<T,AABB3d>::value ||
                std::is_same<T,AABB2d>::value) )
        >
H5::Group saveData(const TFileGroup & fg, const T & aabb, std::string name="AABB")
{
    H5::Group group = fg.createGroup(name);
    saveData(group,aabb.m_minPoint,"minPoint");
    saveData(group,aabb.m_maxPoint,"maxPoint");
    return group;
}


}

#endif
