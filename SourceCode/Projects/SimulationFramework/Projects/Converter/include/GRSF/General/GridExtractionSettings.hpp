#ifndef GRSF_General_GridExtractionSettings_hpp
#define GRSF_General_GridExtractionSettings_hpp

#include <string>
#include <vector>
#include <utility>
#include <type_traits>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include <boost/variant.hpp>

#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"

#include "GRSF/Common/HDF5Helpers.hpp"

namespace Extractors{

    DEFINE_LAYOUT_CONFIG_TYPES

    namespace details{


        /** Defines some tensor types and tensor maps*/
        template<unsigned int NTensorIndices, /* 3d grid has 3 indices*/
                 typename TensorScalar
                 >
        class TensorStorage{
        public:
            DEFINE_MATRIX_STORAGEOPTIONS
            using TensorType = typename MyMatrix::TensorDyn<TensorScalar,NTensorIndices,MatrixRowMajorOption>;
            using TensorMapType = TensorMap<TensorType>;

            /** Make empty map with dimension zero, mapped to nothing!! */
            TensorStorage(std::string name)
                : TensorStorage(name, std::make_index_sequence<NTensorIndices>{}) {}

            template<typename IndexType>
            std::size_t resizeTensor( const IndexType & dimensions ){
                EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(IndexType,NTensorIndices);
                resizeTensor(dimensions, std::make_index_sequence<NTensorIndices>{} );
                return m_tensor.size() * sizeof(TensorScalar);
            }

            template<typename IndexType>
            TensorScalar & getElement( const IndexType & index ){
                EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(IndexType,NTensorIndices);
                return getElement(index, std::make_index_sequence<NTensorIndices>{} );
            }

            std::string m_dataName;         /// Data extraction name
            TensorType m_tensor;
        private:

            template<typename IndexType, std::size_t... Is>
            void resizeTensor(const IndexType & dimensions, std::index_sequence<Is...> )
            {
                std::cout << "REsize" << dimensions << std::endl;
                m_tensor.resize( dimensions(Is)... );
            }

            template<typename IndexType, std::size_t... Is>
            TensorScalar & getElement(const IndexType & index, std::index_sequence<Is...> )
            {
                return m_tensor( index(Is)... );
            }

            template<std::size_t... Is>
            TensorStorage(std::string name, std::index_sequence<Is...>)
                : m_dataName(name), m_tensor( getZero<Is>()... ) {}

            template<std::size_t I> constexpr int getZero(){ return 0;}
        };

        #define DEFINE_TENSORMAPCOMP_TYPE( _class_ ) \
            using TensorType = typename _class_::TensorType;\
            using TensorMapType = typename _class_::TensorMapType;


        template<unsigned int _DimIn,
                 unsigned int _DimOut,
                 typename TScalar = PREC>
        class ProjectiveTransformComponent{
        public:
            static const unsigned int DimIn = _DimIn;
            static const unsigned int DimOut = _DimOut;

            using ProjIndexType  = typename MyMatrix::ArrayStat<std::size_t,DimOut>;
            using ProjMatrixType = typename MyMatrix::MatrixStatStat<TScalar,DimOut,DimIn>;

            bool m_useProjectionMatrix = false;

            /** Method 1*/
            ProjIndexType m_indices;          ///< Two indices which components to take!
            /** Method 2*/
            ProjMatrixType m_P;               ///< Projection operator

        };

        template<unsigned int CellDimIn = 3,
                 unsigned int CellDimOut = 3,
                 typename TScalar = PREC,
                 unsigned int nTensorIndices = 3, /* 3d grid has 3 indices*/
                 typename TCellData   = typename MyMatrix::VectorStat<TScalar,CellDimOut>
                >
        class ExtractorProjection : public ProjectiveTransformComponent<CellDimIn,CellDimOut,TScalar> ,
                                    public TensorStorage<nTensorIndices,TCellData>
        {
        public:

            using BaseStorage = TensorStorage<nTensorIndices,TCellData>;
            DEFINE_TENSORMAPCOMP_TYPE(BaseStorage)

            static const unsigned int CellDataDimension = CellDimOut;
            using Scalar = TScalar;
            using CellDataType = TCellData;

            ExtractorProjection(std::string name): TensorStorage<nTensorIndices,TCellData>(name){}

            template<typename IndexType>
            std::size_t resizeBuffer(const IndexType & gridDimension){
                return this->resizeTensor(gridDimension);
            }

            //constexpr std::size_t getCellDataBytes(){ return CellDataDimension  * sizeof(Scalar);}
        };

        template<unsigned int CellDimOut = 3,
                 typename TScalar = PREC,
                 unsigned int nTensorIndices = 3, /* 3d grid has 3 indices*/
                 typename TCellData = typename MyMatrix::VectorStat<TScalar,CellDimOut>
                >
        class ExtractorNormal : public TensorStorage<nTensorIndices,TCellData>
        {
        public:

            using Scalar = TScalar;
            using DataVectorType = VectorStat<CellDimOut>;
            static const unsigned int Dimension = CellDimOut;

            using BaseStorage = TensorStorage<nTensorIndices,TCellData>;
            DEFINE_TENSORMAPCOMP_TYPE(BaseStorage)

            ExtractorNormal(std::string name): TensorStorage<nTensorIndices,TCellData>(name){}

            //constexpr std::size_t getCellDataBytes(){ return Dimension * sizeof(Scalar); /* size of TCellData */}

            template<typename IndexType>
            std::size_t resizeBuffer(const IndexType & gridDimension){
                return this->resizeTensor(gridDimension);
            }



        public:
            /** TensorMap does not provide setData() functionality and ptr needs to be set in cosntructor,
            *   Therefore we make a unique_ptr to construct it while resizing the buffer
            */
        };

    }

    /** A 3D Grid has 3 Tensor indices, a 2D has 2 indices */
    template<unsigned int nTensorIndices = 3>
    class ExtractorTransVelocityProj1D : public details::ExtractorProjection<3,1,PREC,nTensorIndices>{
    public:

        using Base = details::ExtractorProjection<3,1,PREC,nTensorIndices>;

        DEFINE_TENSORMAPCOMP_TYPE(Base)

        ExtractorTransVelocityProj1D(std::string name): Base(name){}

        bool m_transformToGridCoordinates = true;

        template<typename CellDataType, typename IndexType>
        void writeData(CellDataType & cellData, const IndexType & index)
        {
//            decltype(typename TensorType::Dimensions) d(1,2,3);
//            TensorMapType a(dataBufferPtr,d);
        }
        template<typename FileOrGroup>
        void writeHDF5(const FileOrGroup & fOrG){
            Hdf5Helpers::saveData(fOrG,this->m_tensor,this->m_dataName);
        }
    };

    template<unsigned int nTensorIndices = 3>
    class ExtractorTransVelocityProj2D : public details::ExtractorProjection<3,2,PREC,nTensorIndices>{
    public:
        using Base = details::ExtractorProjection<3,2,PREC,nTensorIndices>;

        DEFINE_TENSORMAPCOMP_TYPE(Base)

        ExtractorTransVelocityProj2D(std::string name): Base(name){}
        bool m_transformToGridCoordinates = true;

        template<typename CellDataType, typename IndexType>
        void writeData(CellDataType & cellData, const IndexType & index)
        {
            //this->getElement(index);
        }
        template<typename FileOrGroup>
        void writeHDF5(const FileOrGroup & fOrG){
            Hdf5Helpers::saveData(fOrG,this->m_tensor,this->m_dataName);
        }
    };
    template<unsigned int nTensorIndices = 3>
    class ExtractorTransVelocity : public details::ExtractorNormal<3,PREC,nTensorIndices>{
    public:
        using Base = details::ExtractorNormal<3,PREC,nTensorIndices>;
        DEFINE_TENSORMAPCOMP_TYPE(Base)

        ExtractorTransVelocity(std::string name): Base(name){}

        bool m_transformToGridCoordinates = true;

        template<typename CellDataType, typename IndexType>
        void writeData(CellDataType & cellData, const IndexType & index)
        {

        }
        template<typename FileOrGroup>
        void writeHDF5(const FileOrGroup & fOrG){
            Hdf5Helpers::saveData(fOrG, this->m_tensor ,this->m_dataName);
        }
    };

}



class GridExtractionSettings{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DEFINE_MATRIX_TYPES

    using Array3UInt = typename MyMatrix::Array3<std::size_t>;

    std::string m_fileName;

    /** 3D Grid */
    AABB3d m_aabb;
    Matrix33 m_R_KI;
    Array3UInt m_dimension;

    /** Data Extractors */
    using ExtractorTransVelType       = Extractors::ExtractorTransVelocity<3>;
    using ExtractorTransVelProj1DType = Extractors::ExtractorTransVelocityProj1D<3>;
    using ExtractorTransVelProj2DType = Extractors::ExtractorTransVelocityProj2D<3>;

    std::vector<ExtractorTransVelType>       m_transVelExtractor; /// only one makes sense!
    std::vector<ExtractorTransVelProj1DType> m_transVelProj1DExtractors;
    std::vector<ExtractorTransVelProj2DType> m_transVelProj2DExtractors;

    /** Returns the total bytes which is needed for all extractors and initializes all buffers */
    std::size_t resizeBuffer(){
        std::size_t totalBytes = 0;
        for(auto & e : m_transVelExtractor){
            totalBytes += e.resizeBuffer(m_dimension);
        }
        for(auto & e : m_transVelProj2DExtractors){
           totalBytes += e.resizeBuffer(m_dimension);
        }
        for(auto & e : m_transVelProj1DExtractors){
            totalBytes += e.resizeBuffer(m_dimension);
        }
        return totalBytes;
    }

    /** Extractor Data Writer Visitor */
    template<typename TGrid>
    class DataWriterVisitor{
        public:
        using GridType = TGrid;
        using GridSettingsType = GridExtractionSettings;

        DataWriterVisitor(GridType * g, GridSettingsType * s): m_grid(g), m_settings(s){}
        DataWriterVisitor(DataWriterVisitor&&d) = default;

        template<typename CellDataType, typename IndexType>
        void operator()(CellDataType & cellData, const IndexType & index)
        {
            for(auto & e : m_settings->m_transVelExtractor){
                e.writeData(cellData,index);
            }
            for(auto & e : m_settings->m_transVelProj1DExtractors){
                e.writeData(cellData,index);
            }
            for(auto & e : m_settings->m_transVelProj2DExtractors){
                e.writeData(cellData,index);
            }
        }

        private:
        GridType * m_grid;
        GridSettingsType * m_settings;
    };

    template<typename FileOrGroup>
    void writeToHDF5(FileOrGroup & fOrG){
        for(auto & e : m_transVelExtractor){
            e.writeHDF5(fOrG);
        }
        for(auto & e : m_transVelProj1DExtractors){
            e.writeHDF5(fOrG);
        }
        for(auto & e : m_transVelProj2DExtractors){
            e.writeHDF5(fOrG);
        }
    }


    template<typename TGrid>
    auto createDataWriterVisitor(TGrid * g) -> DataWriterVisitor<TGrid>
    {
        return DataWriterVisitor<TGrid>{g,this};
    }

    std::size_t extractorCount(){ return m_transVelExtractor.size() + m_transVelProj1DExtractors.size() + m_transVelProj2DExtractors.size();}

};


#endif
