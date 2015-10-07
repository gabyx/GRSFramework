#ifndef GRSF_General_GridExtractionSettings_hpp
#define GRSF_General_GridExtractionSettings_hpp

#include <string>
#include <vector>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include <boost/variant.hpp>

#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"



namespace Extractors{

    DEFINE_LAYOUT_CONFIG_TYPES

    namespace details{


        class ExtractorBase{
        public:

            ExtractorBase(std::string name): m_dataName(name){}

            void resizeBuffer(std::size_t bytes){
                m_dataBuffer.resize( bytes );
            }
            void clearBuffer(){
                m_dataBuffer.clear();
            }

            std::vector<char> m_dataBuffer; ///< Data buffer to write all data
            std::string m_dataName;         /// Data extraction name
        };


        template<unsigned int _DimIn,
                 unsigned int _DimOut,
                 typename TScalar = PREC>
        class ProjectiveTransform{
        public:
            static const unsigned int DimIn = _DimIn;
            static const unsigned int DimOut = _DimOut;

            using ProjIndexType  = typename MyMatrix<std::size_t>::template ArrayStat<DimOut>;
            using ProjMatrixType = typename MyMatrix<TScalar>::template MatrixStatStat<DimOut,DimIn>;

            bool m_useProjectionMatrix = false;

            /** Method 1*/
            ProjIndexType m_indices;          ///< Two indices which components to take!
            /** Method 2*/
            ProjMatrixType m_P;               ///< Projection operator

        };

        template<unsigned int DimOut, unsigned int DimIn = 3, typename TScalar = PREC>
        class ExtractorProjection : public ProjectiveTransform<DimIn,DimOut,TScalar> , public ExtractorBase{
        public:

            ExtractorProjection(std::string name): ExtractorBase(name){}

            using Scalar = TScalar;
            static const unsigned int Dimension = DimOut;

            template<typename T>
            std::size_t resizeBuffer(const T & gridDimension){
                std::size_t bytes = getBytes() * gridDimension.prod();
                ExtractorBase::resizeBuffer(bytes);
                return bytes;
            }

            constexpr std::size_t getBytes(){ return Dimension  * sizeof(PREC);}
        };

        template<unsigned int DimOut = 3, typename TScalar = PREC>
        class ExtractorNormal : public ExtractorBase{
        public:

            ExtractorNormal(std::string name): ExtractorBase(name){}

            using Scalar = TScalar;
            static const unsigned int Dimension = DimOut;

            template<typename T>
            std::size_t resizeBuffer(const T & gridDimension){
                std::size_t bytes = getBytes() * gridDimension.prod();
                ExtractorBase::resizeBuffer(bytes);
                return bytes;
            }

            constexpr std::size_t getBytes(){ return Dimension * sizeof(PREC);}
        };

    }



    class ExtractorTransVelocityProj1D : public details::ExtractorProjection<1,3,PREC>{
    public:
        using Base = details::ExtractorProjection<1,3,PREC>;

        ExtractorTransVelocityProj1D(std::string name): Base(name){}

        bool m_transformToGridCoordinates = true;
    };
    class ExtractorTransVelocityProj2D : public details::ExtractorProjection<2,3,PREC>{
    public:
        using Base = details::ExtractorProjection<2,3,PREC>;

        ExtractorTransVelocityProj2D(std::string name): Base(name){}
        bool m_transformToGridCoordinates = true;
    };
    class ExtractorTransVelocity : public details::ExtractorNormal<3,PREC>{
    public:
        using Base =  details::ExtractorNormal<3,PREC>;

        ExtractorTransVelocity(std::string name = "velocity"): Base(name){}

        bool m_transformToGridCoordinates = true;
    };


}



class GridExtractionSettings{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DEFINE_MATRIX_TYPES

    using Array3UInt = typename MyMatrix<std::size_t>::Array3;

    std::string m_fileName;

    AABB3d m_aabb;
    Matrix33 m_R_KI;
    Array3UInt m_dimension;

    using ExtractorTransVelType       = Extractors::ExtractorTransVelocity;
    using ExtractorTransVelProj1DType = Extractors::ExtractorTransVelocityProj1D;
    using ExtractorTransVelProj2DType = Extractors::ExtractorTransVelocityProj2D;

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

    std::size_t extractorCount(){ return m_transVelExtractor.size() + m_transVelProj1DExtractors.size() + m_transVelProj2DExtractors.size();}

};


#endif
