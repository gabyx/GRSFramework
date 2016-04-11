// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_HDF5Helpers_hpp
#define GRSF_common_HDF5Helpers_hpp

#include <type_traits>

#include <H5Cpp.h>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/SfinaeMacros.hpp"

#include "GRSF/dynamics/collision/geometry/AABB.hpp"

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

    template<>
    inline H5::PredType getNativeType_imp<std::string>()
    {
        return H5::PredType::C_S1;
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

    template<>
    inline H5::PredType mapNativeTypeToLE_imp<std::string, 8 >()
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




/** Hash functor for storing derived H5::Location objects in maps */
template<typename T>
struct Hasher{
    GRSF_STATIC_ASSERTM((std::is_base_of<H5::H5Location,T>::value), "Your class does not inherit from H5Location!")
    inline hid_t  operator()(const T & t) const{
        return t.getId();
    }
};

/** KeyEqual functor for storing derived H5::Location objects in maps */
template<typename T>
struct KeyEqual: std::binary_function <T,T,bool>{
    GRSF_STATIC_ASSERTM((std::is_base_of<H5::H5Location,T>::value), "Your class does not inherit from H5Location!")
    inline bool operator()(const T & a, const T & b) const{
        return a.getId() == b.getId();
    }
};


/** Write simple attribute */
template<typename TFileGroupData, typename T>
inline void saveAttribute(const TFileGroupData & fg, const T & attr, std::string name)
{
    GRSF_STATIC_ASSERTM((!std::is_same<T,std::string>::value), "string should not be saved with this function")
    hsize_t dims=1;
    H5::DataSpace d(1, &dims /*dimension*/);
    H5::Attribute a = fg.createAttribute(name, Hdf5Helpers::mapNativeTypeToLE<T>(),d );
    a.write( Hdf5Helpers::getNativeType<T>(), &attr );
}

/** Write string attribute */
template<typename TFileGroupData>
inline void saveAttribute(const TFileGroupData & fg, const std::string & s, std::string name)
{

    H5::StrType sT(Hdf5Helpers::getNativeType<std::string>());
    sT.setSize(s.size()+1); // nulltermination character as well

    hsize_t dims=1;
    H5::DataSpace d(1, &dims /*dimension*/); // one string
    H5::Attribute a = fg.createAttribute(name, sT , d ); // data type as c string
    a.write( sT, s.c_str() ); // write as c string
}


template<typename TFileGroupData>
void saveRefAttribute(const TFileGroupData & fg , const hobj_ref_t & ref, std::string name = "StateRefs"){
    hsize_t s = 1;
    H5::DataSpace ds( 1, &s);
    H5::Attribute a = fg.createAttribute(name, H5::PredType::STD_REF_OBJ, ds);
    a.write(H5::PredType::STD_REF_OBJ, &ref);
}


namespace details
{

template<typename TFileGroup, typename Scalar>
inline H5::DataSet saveArithmeticArray(const TFileGroup & fg,
        Scalar * data,
        H5::DataSpace & dataSpace,
        std::string name)
{
    GRSF_STATIC_ASSERTM(std::is_arithmetic<Scalar>::value, "Your scalar is not arithmetic!")
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
void saveTensor_artihmScalar(const TFileGroup & fg, const Tensor & m, std::string name)
{
    /* Tensor = Tensor<Scalar,...> or similar */
    /* Scalar  = arithmetic */
    using Scalar = typename Tensor::Scalar;
    GRSF_STATIC_ASSERTM(std::is_arithmetic<Scalar>::value, "Your scalar of the Vector::Scalar is not arithmetic!")

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
void saveTensor_vectorScalar(const TFileGroup & fg, const Tensor & m, std::string name)
{
    /* Tensor = Tensor<Scalar,...> or similar */
    /* Scalar  = VectorStat<N> , fixed size vector */
    static const unsigned int RowsAtCompileTime = Tensor::Scalar::RowsAtCompileTime;
    using Scalar = typename Tensor::Scalar::Scalar;

    GRSF_STATIC_ASSERTM(std::is_arithmetic<Scalar>::value, "Your scalar of the Vector::Scalar is not arithmetic!")

    auto & d = m.dimensions();
    std::array<hsize_t, Tensor::NumIndices + 1 > dims;
    for( std::size_t i=0;  i < Tensor::NumIndices;  ++i)
    {
        dims[i] = static_cast<hsize_t>(d[i]);
    }
    dims[Tensor::NumIndices ] = RowsAtCompileTime;

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
        std::size_t Indices, int Options,
        SFINAE_ENABLE_IF( std::is_arithmetic<TScalar>::value )
        >
void saveData(const TFileGroup & fg, const MyMatrix::TensorDyn<TScalar,Indices,Options> & m, std::string name)
{
    details::saveTensor_artihmScalar(fg,m,name);
}

template<typename TFileGroup,
        typename TScalar,
        std::size_t Indices, int Options,
        SFINAE_ENABLE_IF( !std::is_arithmetic<TScalar>::value &&
                                TScalar::IsVectorAtCompileTime
                                             )
        >
void saveData(const TFileGroup & fg, const MyMatrix::TensorDyn<TScalar,Indices,Options> & m, std::string name)
{
    details::saveTensor_vectorScalar(fg,m,name);
}


template<typename TFileGroup,
        typename Derived,
        SFINAE_ENABLE_IF( !std::is_arithmetic<typename Derived::Scalar>::value )
        >
void saveData(const TFileGroup & fg, const MatrixBase<Derived> & m, std::string name)
{

    GRSF_ERRORMSG("NOT IMPLEMENTED")
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

template<typename TFileGroup>
void saveRefData(const TFileGroup & fg , const std::vector<hobj_ref_t> & refs, std::string name = "StateRefs"){
    hsize_t s = refs.size();
    H5::DataSpace ds( 1, &s);
    auto refset = fg.createDataSet(name, H5::PredType::STD_REF_OBJ, ds);
    refset.write(&refs[0], H5::PredType::STD_REF_OBJ);
}




}

#endif
