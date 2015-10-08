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

        template<typename TFileGroup, typename Scalar>
        inline H5::DataSet saveArithmeticArray(const TFileGroup & fg,
                                               Scalar * data,
                                               H5::DataSpace & dataSpace ,
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
        void saveMatrixOrArray_artihmScalar(const TFileGroup & fg, const T & m , std::string name){
            /* Derived = Matrix<Scalar,...> or similar */
            /* Scalar = arithmetic */
            hsize_t dims[2] = {static_cast<hsize_t>(m.rows()),
                               static_cast<hsize_t>(m.cols())};

            H5::DataSpace dataSpace(2, dims);

            H5::DataSet s = saveArithmeticArray(fg, m.derived().data(),dataSpace,name);

            // Hdf5 writes in row-major order, so if matrix is col-major (as default in eigen)
            // we mark an atttribute which tells if this matrix should be transformed to colmajor storage after read in the file
            if(dims[1]!=1){
                saveAttribute(s, !static_cast<char>(Derived::Flags & Eigen::RowMajorBit), "convertToColMajor");
            }
        }

        template<typename Derived, typename TFileGroup, typename T>
        void saveTensor_artihmScalar(const TFileGroup & fg, const T & m , std::string name){
            /* Derived = Tensor<Scalar,...> or similar */
            /* Scalar  = arithmetic */

            auto & d = m.derived().dimensions();
            std::array<hsize_t, T::NumIndices > dims;
            for( std::size_t i=0;  i < T::NumIndices;  ++i){
                dims(i) = static_cast<hsize_t>(d(i));
            }

            H5::DataSpace dataSpace(T::NumIndices, &dims[0]);

            H5::DataSet s = saveArithmeticArray(fg, m.derived().data(),dataSpace,name);

            // Hdf5 writes in row-major order, so if matrix is col-major (as default in eigen)
            // we mark an atttribute which tells if this matrix should be transformed to colmajor storage after read in the file

            saveAttribute(s, !static_cast<char>(Derived::Layout & Eigen::RowMajorBit), "convertToColMajor");

        }

        template<typename Derived, typename TFileGroup, typename T>
        void saveTensor_vectorScalar(const TFileGroup & fg, const T & m , std::string name){
            /* Derived = Tensor<Scalar,...> or similar */
            /* Scalar  = VectorStat<N> , fixed size vector */
            static const unsigned int RowsAtCompileTime = Derived::Scalar::RowsAtCompileTime;
            using Scalar = typename Derived::Scalar::Scalar;

            STATIC_ASSERTM(std::is_arithmetic<Scalar>::value, "Your scalar of the Vector::Scalar is not arithmetic!")

            auto & d = m.derived().dimensions();
            std::array<hsize_t, T::NumIndices + 1 > dims;
            for( std::size_t i=0;  i < T::NumIndices;  ++i){
                dims(i) = static_cast<hsize_t>(d(i));
            }
            d(T::NumIndices + 1) = RowsAtCompileTime;

            H5::DataSpace dataSpace(T::NumIndices, &dims[0]);

            H5::DataSet s = saveArithmeticArray(fg, m.derived().data(),dataSpace,name);

            // Hdf5 writes in row-major order, so if matrix is col-major (as default in eigen)
            // we mark an atttribute which tells if this matrix should be transformed to colmajor storage after read in the file
            saveAttribute(s, !static_cast<char>(Derived::Layout & Eigen::RowMajorBit), "convertToColMajor");

        }

    }

     /** Vector and Matrix Saving */
    template<typename TFileGroup,
             typename Derived,
             SFINAE_ENABLE_IF( std::is_arithmetic<typename Derived::Scalar>::value )
    >
    void saveData(const TFileGroup & fg, const MatrixBase<Derived> & m , std::string name){
        details::saveMatrixOrArray_artihmScalar<Derived>(fg,m,name);
    }
    template<typename TFileGroup,
             typename Derived,
             SFINAE_ENABLE_IF( std::is_arithmetic<typename Derived::Scalar>::value )
    >
    void saveData(const TFileGroup & fg, const ArrayBase<Derived> & m , std::string name){
        details::saveMatrixOrArray_artihmScalar<Derived>(fg,m,name);
    }

    template<typename TFileGroup,
             typename Derived,
             SFINAE_ENABLE_IF( std::is_arithmetic<typename Derived::Scalar>::value )
    >
    void saveData(const TFileGroup & fg, const TensorBase<Derived> & m , std::string name){
        details::saveTensor_artihmScalar<Derived>(fg,m,name);
    }

    template<typename TFileGroup,
             typename Derived,
             SFINAE_ENABLE_IF( !std::is_arithmetic<typename Derived::Scalar>::value &&
                                Derived::Scalar::IsVectorAtCompileTime
                              )
    >
    void saveData(const TFileGroup & fg, const TensorBase<Derived> & m , std::string name){
        details::saveTensor_vectorScalar<Derived>(fg,m,name);
    }


    template<typename TFileGroup,
             typename Derived,
             SFINAE_ENABLE_IF( !std::is_arithmetic<typename Derived::Scalar>::value )
    >
    void saveData(const TFileGroup & fg, const MatrixBase<Derived> & m , std::string name){

       ERRORMSG("NOT IMPLEMENTED")
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
