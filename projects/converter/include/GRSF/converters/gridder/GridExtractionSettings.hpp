// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_gridder_GridExtractionSettings_hpp
#define GRSF_converters_gridder_GridExtractionSettings_hpp

#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include <boost/variant.hpp>

#include "GRSF/dynamics/collision/geometry/AABB.hpp"

#include "GRSF/common/HDF5Helpers.hpp"

namespace Extractors
{
DEFINE_LAYOUT_CONFIG_TYPES

namespace details
{
/** Defines some tensor types and tensor maps*/
template <unsigned int NTensorIndices, /* 3d grid has 3 indices*/
          typename TensorScalar>
class TensorStorage
{
public:
    DEFINE_MATRIX_STORAGEOPTIONS
    using TensorType    = typename MyMatrix::TensorDyn<TensorScalar, NTensorIndices, MatrixRowMajorOption>;
    using TensorMapType = TensorMap<TensorType>;

    /** Make empty map with dimension zero, mapped to nothing!! */
    TensorStorage(std::string name) : TensorStorage(name, std::make_index_sequence<NTensorIndices>{})
    {
    }

    template <typename IndexType>
    std::size_t resizeTensor(const IndexType& dimensions)
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(IndexType, NTensorIndices);
        resizeTensor(dimensions, std::make_index_sequence<NTensorIndices>{});
        return m_tensor.size() * sizeof(TensorScalar);
    }

    template <typename IndexType>
    TensorScalar& getElement(const IndexType& index)
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(IndexType, NTensorIndices);
        return getElement(index, std::make_index_sequence<NTensorIndices>{});
    }

    std::string m_dataName;  /// Data extraction name
    TensorType m_tensor;

private:
    template <typename IndexType, std::size_t... Is>
    void resizeTensor(const IndexType& dimensions, std::index_sequence<Is...>)
    {
        m_tensor.resize(dimensions(Is)...);
    }

    template <typename IndexType, std::size_t... Is>
    TensorScalar& getElement(const IndexType& index, std::index_sequence<Is...>)
    {
        return m_tensor(index(Is)...);
    }

    template <std::size_t... Is>
    TensorStorage(std::string name, std::index_sequence<Is...>) : m_dataName(name), m_tensor(getZero<Is>()...)
    {
    }

    template <std::size_t I>
    constexpr int getZero()
    {
        return 0;
    }
};

#define DEFINE_TENSORSTORAGE_TYPE(_class_)              \
    using TensorType    = typename _class_::TensorType; \
    using TensorMapType = typename _class_::TensorMapType;

template <unsigned int _DimIn, unsigned int _DimOut, typename TScalar = PREC>
class ProjectiveTransformComponent
{
public:
    static const unsigned int DimIn  = _DimIn;
    static const unsigned int DimOut = _DimOut;

    using ProjIndexType  = typename MyMatrix::ArrayStat<std::size_t, DimOut>;
    using ProjMatrixType = typename MyMatrix::MatrixStatStat<TScalar, DimOut, DimIn>;

    bool m_useProjectionMatrix = false;

    /** Method 1*/
    ProjIndexType m_projIndices;  ///< Two indices which components to take!
    /** Method 2*/
    ProjMatrixType m_P;  ///< Projection operator
};

template <unsigned int CellDimIn      = 3,
          unsigned int CellDimOut     = 3,
          typename TScalar            = PREC,
          unsigned int nTensorIndices = 3, /* 3d grid has 3 indices*/
          typename TCellData          = typename MyMatrix::VectorStat<TScalar, CellDimOut>>
class ExtractorProjection : public ProjectiveTransformComponent<CellDimIn, CellDimOut, TScalar>,
                            public TensorStorage<nTensorIndices, TCellData>
{
public:
    using BaseStorage = TensorStorage<nTensorIndices, TCellData>;
    DEFINE_TENSORSTORAGE_TYPE(BaseStorage)

    static const unsigned int CellDataDimension = CellDimOut;
    using Scalar                                = TScalar;
    using CellDataType                          = TCellData;

    ExtractorProjection(std::string name) : TensorStorage<nTensorIndices, TCellData>(name)
    {
    }

    template <typename IndexType>
    std::size_t resizeBuffer(const IndexType& gridDimension)
    {
        return this->resizeTensor(gridDimension);
    }

    // constexpr std::size_t getCellDataBytes(){ return CellDataDimension  * sizeof(Scalar);}
};

template <unsigned int CellDimOut     = 3,
          typename TScalar            = PREC,
          unsigned int nTensorIndices = 3, /* 3d grid has 3 indices*/
          typename TCellData          = typename MyMatrix::VectorStat<TScalar, CellDimOut>>
class ExtractorNormal : public TensorStorage<nTensorIndices, TCellData>
{
public:
    using Scalar                        = TScalar;
    using DataVectorType                = VectorStat<CellDimOut>;
    static const unsigned int Dimension = CellDimOut;

    using BaseStorage = TensorStorage<nTensorIndices, TCellData>;
    DEFINE_TENSORSTORAGE_TYPE(BaseStorage)

    ExtractorNormal(std::string name) : TensorStorage<nTensorIndices, TCellData>(name)
    {
    }

    // constexpr std::size_t getCellDataBytes(){ return Dimension * sizeof(Scalar); /* size of TCellData */}

    template <typename IndexType>
    std::size_t resizeBuffer(const IndexType& gridDimension)
    {
        return this->resizeTensor(gridDimension);
    }

public:
    /** TensorMap does not provide setData() functionality and ptr needs to be set in cosntructor,
            *   Therefore we make a unique_ptr to construct it while resizing the buffer
            */
};
}

/** A 3D Grid has 3 Tensor indices, a 2D has 2 indices */
template <unsigned int nTensorIndices = 3>
class ExtractorTransVelocityProj1D : public details::ExtractorProjection<3, 1, PREC, nTensorIndices>
{
public:
    using Base = details::ExtractorProjection<3, 1, PREC, nTensorIndices>;

    DEFINE_TENSORSTORAGE_TYPE(Base)

    using Base::m_P;
    using Base::m_projIndices;
    using Base::m_useProjectionMatrix;

    ExtractorTransVelocityProj1D(std::string name) : Base(name)
    {
    }

    bool m_transformToGridCoordinates = true;

    template <typename TGrid, typename CellDataType, typename IndexType>
    inline void writeCellData(TGrid* g, CellDataType& cellData, const IndexType& index)
    {
        static Vector3 temp;
        if (cellData.m_rigidBodyState)
        {
            if (m_transformToGridCoordinates)
            {
                temp = g->getTransformKI() * cellData.m_rigidBodyState->getVelocityTrans();
            }
            else
            {
                temp = cellData.m_rigidBodyState->getVelocityTrans();
            }
            if (m_useProjectionMatrix)
            {
                this->getElement(index) = m_P * temp;
            }
            else
            {
                this->getElement(index)(0) = temp(m_projIndices(0));
            }
        }
        else
        {
            this->getElement(index)(0) = 0;
        }
    }

    template <typename TGrid, typename Iterator>
    inline void writeAllData(TGrid* g, Iterator begin, Iterator end)
    {
    }

    template <typename FileOrGroup>
    inline void writeHDF5(const FileOrGroup& fOrG)
    {
        Hdf5Helpers::saveData(fOrG, this->m_tensor, this->m_dataName);
    }
};

template <unsigned int nTensorIndices = 3>
class ExtractorTransVelocityProj2D : public details::ExtractorProjection<3, 2, PREC, nTensorIndices>
{
public:
    using Base = details::ExtractorProjection<3, 2, PREC, nTensorIndices>;

    DEFINE_TENSORSTORAGE_TYPE(Base)

    using Base::m_P;
    using Base::m_projIndices;
    using Base::m_useProjectionMatrix;

    ExtractorTransVelocityProj2D(std::string name) : Base(name)
    {
    }
    bool m_transformToGridCoordinates = true;

    template <typename TGrid, typename CellDataType, typename IndexType>
    inline void writeCellData(TGrid* g, CellDataType& cellData, const IndexType& index)
    {
        static Vector3 temp;
        if (cellData.m_rigidBodyState)
        {
            if (m_transformToGridCoordinates)
            {
                temp = g->getTransformKI() * cellData.m_rigidBodyState->getVelocityTrans();
            }
            else
            {
                temp = cellData.m_rigidBodyState->getVelocityTrans();
            }
            if (m_useProjectionMatrix)
            {
                this->getElement(index) = m_P * temp;
            }
            else
            {
                auto& d = this->getElement(index);
                d(0)    = temp(m_projIndices(0));
                d(1)    = temp(m_projIndices(1));
            }
        }
        else
        {
            this->getElement(index) = Vector2::Zero();
        }
    }

    template <typename TGrid, typename Iterator>
    inline void writeAllData(TGrid* g, Iterator begin, Iterator end)
    {
    }

    template <typename FileOrGroup>
    inline void writeHDF5(const FileOrGroup& fOrG)
    {
        Hdf5Helpers::saveData(fOrG, this->m_tensor, this->m_dataName);
    }
};
template <unsigned int nTensorIndices = 3>
class ExtractorTransVelocity : public details::ExtractorNormal<3, PREC, nTensorIndices>
{
public:
    using Base = details::ExtractorNormal<3, PREC, nTensorIndices>;
    DEFINE_TENSORSTORAGE_TYPE(Base)

    ExtractorTransVelocity(std::string name) : Base(name)
    {
    }

    bool m_transformToGridCoordinates = true;

    template <typename TGrid, typename CellDataType, typename IndexType>
    inline void writeCellData(TGrid* g, CellDataType& cellData, const IndexType& index)
    {
        if (cellData.m_rigidBodyState)
        {
            if (m_transformToGridCoordinates)
            {
                this->getElement(index) = g->getTransformKI() * cellData.m_rigidBodyState->getVelocityTrans();
            }
            else
            {
                this->getElement(index) = cellData.m_rigidBodyState->getVelocityTrans();
            }
        }
        else
        {
            this->getElement(index) = Vector3::Zero();
        }
    }

    template <typename TGrid, typename Iterator>
    inline void writeAllData(TGrid* g, Iterator begin, Iterator end)
    {
    }

    template <typename FileOrGroup>
    inline void writeHDF5(const FileOrGroup& fOrG)
    {
        Hdf5Helpers::saveData(fOrG, this->m_tensor, this->m_dataName);
    }
};

template <unsigned int nTensorIndices = 3>
class ExtractorBodyMask : public details::ExtractorNormal<1, unsigned char, nTensorIndices>
{
public:
    using Base = details::ExtractorNormal<1, unsigned char, nTensorIndices>;
    DEFINE_TENSORSTORAGE_TYPE(Base)

    ExtractorBodyMask(std::string name) : Base(name)
    {
    }

    bool m_transformToGridCoordinates = true;

    template <typename TGrid, typename CellDataType, typename IndexType>
    inline void writeCellData(TGrid* g, CellDataType& cellData, const IndexType& index)
    {
        if (cellData.m_rigidBodyState)
        {
            this->getElement(index)(0) = 1;
        }
        else
        {
            this->getElement(index)(0) = 0;
        }
    }

    template <typename TGrid, typename Iterator>
    inline void writeAllData(TGrid* g, Iterator begin, Iterator end)
    {
    }

    template <typename FileOrGroup>
    inline void writeHDF5(const FileOrGroup& fOrG)
    {
        Hdf5Helpers::saveData(fOrG, this->m_tensor, this->m_dataName);
    }
};
}

class GridExtractionSettings
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DEFINE_MATRIX_TYPES

    using Array3UInt = typename MyMatrix::Array3<std::size_t>;

    std::string m_fileName;

    /** Bounds (only used for output, not for computation!)*/
    Vector3 m_minPointOrig;
    Vector3 m_maxPointOrig;

    /** 3D Grid */
    AABB3d m_aabb;  ///< Coordinates in K frame which is located at Origin
    Matrix33 m_R_KI;
    Array3UInt m_dimension;

    /** Data Extractors */
    using ExtractorBodyMaskType       = Extractors::ExtractorBodyMask<3>;
    using ExtractorTransVelType       = Extractors::ExtractorTransVelocity<3>;
    using ExtractorTransVelProj1DType = Extractors::ExtractorTransVelocityProj1D<3>;
    using ExtractorTransVelProj2DType = Extractors::ExtractorTransVelocityProj2D<3>;

    std::vector<ExtractorTransVelType> m_transVelExtractor;  /// only one makes sense!
    std::vector<ExtractorTransVelProj1DType> m_transVelProj1DExtractors;
    std::vector<ExtractorTransVelProj2DType> m_transVelProj2DExtractors;
    std::vector<ExtractorBodyMaskType> m_bodyMaskExtractors;

    /** Returns the total bytes which is needed for all extractors and initializes all buffers */
    std::size_t resizeBuffer()
    {
        std::size_t totalBytes = 0;
        for (auto& e : m_transVelExtractor)
        {
            totalBytes += e.resizeBuffer(m_dimension);
        }
        for (auto& e : m_transVelProj2DExtractors)
        {
            totalBytes += e.resizeBuffer(m_dimension);
        }
        for (auto& e : m_transVelProj1DExtractors)
        {
            totalBytes += e.resizeBuffer(m_dimension);
        }
        for (auto& e : m_bodyMaskExtractors)
        {
            totalBytes += e.resizeBuffer(m_dimension);
        }
        return totalBytes;
    }

    /** Extractor Data Writer Visitor */
    template <typename TGrid>
    class DataWriterVisitor
    {
    public:
        using GridType            = TGrid;
        using GridExtSettingsType = GridExtractionSettings;

        DataWriterVisitor(GridType* g, GridExtSettingsType* s) : m_grid(g), m_settings(s)
        {
        }
        DataWriterVisitor(DataWriterVisitor&& d) = default;

        /* Grid Visitor */
        template <typename CellDataType, typename IndexType>
        void operator()(CellDataType& cellData, const IndexType& index)
        {
            for (auto& e : m_settings->m_transVelExtractor)
            {
                e.writeCellData(m_grid, cellData, index);
            }
            for (auto& e : m_settings->m_transVelProj2DExtractors)
            {
                e.writeCellData(m_grid, cellData, index);
            }
            for (auto& e : m_settings->m_transVelProj1DExtractors)
            {
                e.writeCellData(m_grid, cellData, index);
            }
            for (auto& e : m_settings->m_bodyMaskExtractors)
            {
                e.writeCellData(m_grid, cellData, index);
            }
        }

        /* Write all data for iterator begin to the end */
        template <typename CellDataIt>
        void writeAllData(CellDataIt begin, CellDataIt end)
        {
            for (auto& e : m_settings->m_transVelExtractor)
            {
                e.writeAllData(m_grid, begin, end);
            }
            for (auto& e : m_settings->m_transVelProj2DExtractors)
            {
                e.writeCellData(m_grid, begin, end);
            }
            for (auto& e : m_settings->m_transVelProj1DExtractors)
            {
                e.writeCellData(m_grid, begin, end);
            }
            for (auto& e : m_settings->m_bodyMaskExtractors)
            {
                e.writeCellData(m_grid, begin, end);
            }
        }

    private:
        GridType* m_grid;
        GridExtSettingsType* m_settings;
    };

    template <typename TGrid>
    auto createDataWriterVisitor(TGrid* g) -> DataWriterVisitor<TGrid>
    {
        return DataWriterVisitor<TGrid>{g, this};
    }

    template <typename FileOrGroup>
    void writeToHDF5(FileOrGroup& fOrG)
    {
        for (auto& e : m_transVelExtractor)
        {
            e.writeHDF5(fOrG);
        }
        for (auto& e : m_transVelProj2DExtractors)
        {
            e.writeHDF5(fOrG);
        }
        for (auto& e : m_transVelProj1DExtractors)
        {
            e.writeHDF5(fOrG);
        }
        for (auto& e : m_bodyMaskExtractors)
        {
            e.writeHDF5(fOrG);
        }
    }

    std::size_t extractorCount()
    {
        return m_transVelExtractor.size() + m_transVelProj1DExtractors.size() + m_transVelProj2DExtractors.size() +
               m_bodyMaskExtractors.size();
    }
};

#endif
