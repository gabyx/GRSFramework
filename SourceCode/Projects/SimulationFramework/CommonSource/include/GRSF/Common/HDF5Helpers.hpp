#ifndef GRSF_Common_Hdf5Helpers_hpp
#define GRSF_Common_Hdf5Helpers_hpp

#include <type_traits>

#include <H5Cpp.h>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/SfinaeMacros.hpp"

#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"

namespace Hdf5Helpers{

    DEFINE_LAYOUT_CONFIG_TYPES

    /** Get native types */
    template<typename T>
    inline H5::PredType getNativeType();

    template<>
    H5::PredType getNativeType<float>(){
        return H5::PredType::NATIVE_FLOAT;
    };

    template<>
    H5::PredType getNativeType<double>(){
        return H5::PredType::NATIVE_DOUBLE;
    };

    template<>
    H5::PredType getNativeType<bool>(){
        return H5::PredType::NATIVE_UCHAR;
    };


    template<>
    H5::PredType getNativeType<char>(){
        return H5::PredType::NATIVE_CHAR;
    };

    template<>
    H5::PredType getNativeType<unsigned char>(){
        return H5::PredType::NATIVE_UCHAR;
    };

    template<>
    H5::PredType getNativeType<int>(){
        return H5::PredType::NATIVE_INT;
    };

    template<>
    H5::PredType getNativeType<unsigned int>(){
        return H5::PredType::NATIVE_UINT;
    };

    template<>
    H5::PredType getNativeType<unsigned long int>(){
        return H5::PredType::NATIVE_ULONG;
    };

    template<>
    H5::PredType getNativeType<long long int>(){
        return H5::PredType::NATIVE_LLONG;
    };

    /** Map native types To little endian ones*/
    template<typename T, std::size_t N = sizeof(T) >
    inline H5::PredType mapNativeTypeToLE();

    template<>
    H5::PredType mapNativeTypeToLE<float, 4 >(){
        return H5::PredType::IEEE_F32LE;
    };

    template<>
    H5::PredType mapNativeTypeToLE<double, 8 >(){
        return H5::PredType::IEEE_F64LE;
    };


    template<>
    H5::PredType mapNativeTypeToLE<bool, 1 >(){
        return H5::PredType::STD_U8LE;
    };


    template<>
    H5::PredType mapNativeTypeToLE<char, 1 >(){
        return H5::PredType::STD_I8LE;
    };

    template<>
    H5::PredType mapNativeTypeToLE<unsigned char, 1 >(){
        return H5::PredType::STD_U8LE;
    };

    template<>
    H5::PredType mapNativeTypeToLE<int, 4 >(){
        return H5::PredType::STD_I32LE;
    };

    template<>
    H5::PredType mapNativeTypeToLE<unsigned int, 4 >(){
        return H5::PredType::STD_U32LE;
    };

    template<>
    H5::PredType mapNativeTypeToLE<unsigned long int, 8>(){
        return H5::PredType::STD_U64LE;
    };

    template<>
    H5::PredType mapNativeTypeToLE<long long int, 8 >(){
        return H5::PredType::STD_U64LE;
    };

    /** Write simple attribute */
    template<typename TFileGroupData, typename T>
    void saveAttribute(const TFileGroupData & fg, const T & attr , std::string name){
        hsize_t dims=1;
        H5::DataSpace d(1, &dims /*dimension*/);
        H5::Attribute a = fg.createAttribute(name, Hdf5Helpers::mapNativeTypeToLE<T>() ,d );
        a.write( Hdf5Helpers::getNativeType<T>() , &attr );
    }




    namespace details {
        template<typename Derived, typename TFileGroup, typename T>
        void saveMatrixOrArray(const TFileGroup & fg, const T & m , std::string name){

            hsize_t dims[2] = {static_cast<hsize_t>(m.rows()),
                               static_cast<hsize_t>(m.cols())};

            H5::DataSpace d(2, dims);
            H5::DataSet s = fg.createDataSet(name, Hdf5Helpers::mapNativeTypeToLE<typename Derived::Scalar>(), d);

            s.write(m.derived().data(), Hdf5Helpers::getNativeType<typename Derived::Scalar>() );

            // Hdf5 writes in row-major order, so if matrix is col-major (as default in eigen)
            // we mark an atttribute which tells if this matrix is the transpose of the original
            if(dims[1]!=1){
                saveAttribute(s, !static_cast<char>(Derived::Flags & Eigen::RowMajorBit), "isTransposed");
            }
        }
    }

     /** Vector and Matrix Saving */
    template<typename TFileGroup,
             typename Derived,
             SFINAE_ENABLE_IF( std::is_arithmetic<typename Derived::Scalar>::value )
    >
    void saveData(const TFileGroup & fg, const MatrixBase<Derived> & m , std::string name){
        details::saveMatrixOrArray<Derived>(fg,m,name);
    }
    template<typename TFileGroup,
             typename Derived,
             SFINAE_ENABLE_IF( std::is_arithmetic<typename Derived::Scalar>::value )
    >
    void saveData(const TFileGroup & fg, const ArrayBase<Derived> & m , std::string name){
        details::saveMatrixOrArray<Derived>(fg,m,name);
    }


    template<typename TFileGroup,
             typename Derived,
             SFINAE_ENABLE_IF( !std::is_arithmetic<typename Derived::Scalar>::value )
    >
    void saveData(const TFileGroup & fg, const MatrixBase<Derived> & m , std::string name){

        hsize_t dims[2] = {static_cast<hsize_t>(m.rows()),
                           static_cast<hsize_t>(m.cols())};

        H5::DataSpace d(2, dims);
        H5::DataSet s = fg.createDataSet(name, Hdf5Helpers::mapNativeTypeToLE<typename Derived::Scalar>(), d);

        s.write(m.derived().data(), Hdf5Helpers::getNativeType<typename Derived::Scalar>() );

        // Write attribute row or colmajor for matrices
        if(dims[1]!=1){
            saveAttribute(s, static_cast<char>(Derived::Flags & Eigen::RowMajorBit), "rowMajor");
        }
    }


    /** AABB saving*/
    template<typename TFileGroup,
             typename T,
             SFINAE_ENABLE_IF((std::is_same<T,AABB3d>::value ||
                              std::is_same<T,AABB2d>::value) )
             >
    H5::Group saveData(const TFileGroup & fg, const T & aabb , std::string name="AABB"){
        H5::Group group = fg.createGroup(name);
        saveData(group,aabb.m_minPoint,"minPoint");
        saveData(group,aabb.m_maxPoint,"maxPoint");
        return group;
    }


}

#endif
