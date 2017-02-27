// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_CartesianGrid_hpp
#define GRSF_dynamics_general_CartesianGrid_hpp

#include "GRSF/common/Asserts.hpp"
#include "GRSF/common/StaticAssert.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include <set>
#include "GRSF/dynamics/collision/geometry/AABB.hpp"

#include "GRSF/common/SfinaeMacros.hpp"

struct NoCellData
{
};

template <typename TCellData = NoCellData, typename TSize = std::size_t, typename TLongInt = long int>
class CartesianGrid
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DEFINE_MATRIX_TYPES

    using SizeType = TSize;  ///< The type of the indices
    using LongInt =
        TLongInt;  ///< used for index computations (negative numbers needed), int32 is not faster than int64!
    static const unsigned int Dimension = 3;
    using CellDataType                  = TCellData;

    using IndexType     = typename MyMatrix::Array3<SizeType>;
    using IndexLongType = typename MyMatrix::Array3<TLongInt>;

    using CellDataListType = std::vector<TCellData>;

    CartesianGrid(const AABB3d& aabb, const IndexType& dim, const Matrix33& A_KI = Matrix33::Identity())
        : m_aabb(aabb), m_dim(dim), m_A_KI(A_KI)
    {
        GRSF_ASSERTMSG(m_dim.prod() != 0, "Dimension zero: " << dim)
        GRSF_ASSERTMSG(aabb.isEmpty() == false, "CartesianGrid, wrongly initialized: maxPoint < minPoint");

        m_dxyz    = m_aabb.extent() / dim.template cast<PREC>();
        m_dxyzInv = m_dxyz.inverse();

        if (!std::is_same<TCellData, NoCellData>::value)
        {
            m_cellData.resize(m_dim.prod());
        }
    }

    ~CartesianGrid()
    {
    }

    inline IndexType getDimensions()
    {
        return m_dim;
    }
    inline Array3 getDx()
    {
        return m_dxyz;
    }
    inline Array3 getDxInv()
    {
        return m_dxyzInv;
    }
    inline const AABB3d& getAABB()
    {
        return m_aabb;
    }
    inline const Matrix33& getTransformKI()
    {
        return m_A_KI;
    }

    /** Get cell index, points needs to be in same frame as aabb in this class */
    template <bool transformInputToKSystem, typename Derived>
    bool getCellIndex(const MatrixBase<Derived>& point, IndexType& index) const
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

        if (transformInputToKSystem)
        {
            m_temp = (((m_A_KI * point - m_aabb.m_minPoint).array()) * m_dxyzInv).template cast<LongInt>();
        }
        else
        {
            m_temp = (((point - m_aabb.m_minPoint).array()) * m_dxyzInv).template cast<LongInt>();
        }
        // If inside grid
        if (!((m_temp < 0).any() || (m_temp >= m_dim.template cast<LongInt>()).any()))
        {
            index = m_temp.template cast<SizeType>();
            return true;
        };
        return false;
    }

    /** If transformInputToKSystem == true , the input point is assumed to be
    * in I system and is transformed to the K system. other wise the point is assumed to be in the system of the grid
    */
    template <bool transformInputToKSystem, typename Derived>
    IndexType getCellIndexClosest(const MatrixBase<Derived>& point) const
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

        // calculate index normally and then project it into the feasible grid.
        if (transformInputToKSystem)
        {
            m_temp = (((m_A_KI * point - m_aabb.m_minPoint).array()) * m_dxyzInv).template cast<LongInt>();
        }
        else
        {
            m_temp = (((point - m_aabb.m_minPoint).array()) * m_dxyzInv).template cast<LongInt>();
        }

        // prox  index to feasible range (cartesian prox)
        //        m_temp(0) = std::max(   std::min( LongInt(m_dim(0)-1), m_temp(0)),  0LL   );
        //        m_temp(1) = std::max(   std::min( LongInt(m_dim(1)-1), m_temp(1)),  0LL   );
        //        m_temp(2) = std::max(   std::min( LongInt(m_dim(2)-1), m_temp(2)),  0LL   );
        // eigen is as fast
        m_temp = m_zero.max((m_dim.template cast<LongInt>() - 1).min(m_temp));

        return m_temp.template cast<TSize>();
    };

    /** Get cell data  */
    template <bool                         transformInputToKSystem, typename Derived>
    typename CellDataListType::value_type* getCellData(const MatrixBase<Derived>& point)
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

        if (!std::is_same<TCellData, NoCellData>::value)
        {
            IndexType index;
            if (getCellIndex<transformInputToKSystem>(point, index))
            {
                return &m_cellData[getLinearIndex(index)];
            }
        }
        return nullptr;
    }

    /** Get cell data  */
    template <bool                         transformInputToKSystem, typename Derived>
    typename CellDataListType::value_type* getCellData(const MatrixBase<Derived>& point, IndexType& index)
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        if (!std::is_same<TCellData, NoCellData>::value)
        {
            if (getCellIndex<transformInputToKSystem>(point, index))
            {
                return &m_cellData[getLinearIndex(index)];
            }
        }
        return nullptr;
    }

    /** Get cell data closest  */
    template <bool transformInputToKSystem, typename Derived>
    TCellData* getCellDataClosest(const MatrixBase<Derived>& point) const
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

        if (!std::is_same<TCellData, NoCellData>::value)
        {
            IndexType index = getCellIndexClosest<transformInputToKSystem>(point);
            return &m_cellData[getLinearIndex(index)];
        }
        return nullptr;
    }

    template <typename Derived, bool indexCheck = true, SFINAE_ENABLE_IF(indexCheck)>
    TCellData* getCellData(const ArrayBase<Derived>& index) const
    {
        GRSF_STATIC_ASSERT((std::is_same<typename Derived::Scalar, SizeType>::value));

        if (!std::is_same<TCellData, NoCellData>::value &&
            (!indexCheck ||
             !((index < 0).any() ||
               (index >= m_dim).any())) /* a decent compiler will optimize out if !indexCheck = true */
            )
        {
            return &m_cellData[getLinearIndex(index)];
        }
        return nullptr;
    }

    template <typename Derived, bool indexCheck, SFINAE_ENABLE_IF(!indexCheck)>
    TCellData* getCellData(const ArrayBase<Derived>& index) const
    {
        if (!std::is_same<TCellData, NoCellData>::value)
        {
            return &m_cellData[getLinearIndex(index)];
        }
        return nullptr;
    }

    /** Get cell point in K frame of grid,
    *   P = 0: Bottom
    *   P = 1: Center
    *   P = 2: Top
    *   index tuple must be valid, and is not checked!
    */
    template <unsigned int P = 0, typename Derived>
    Vector3 getCellPoint(const ArrayBase<Derived>& index) const
    {
        GRSF_STATIC_ASSERT(P <= 2)
        if (P == 0)
        {
            return m_aabb.m_minPoint.array() + m_dxyz * index.template cast<PREC>();
        }
        else if (P == 1)
        {
            return m_aabb.m_minPoint.array() + 0.5 * m_dxyz + m_dxyz * index.template cast<PREC>();
        }
        else
        {
            return m_aabb.m_minPoint.array() + m_dxyz + m_dxyz * index.template cast<PREC>();
        }
    }

    template <typename TVisitor>
    void applyVisitor(TVisitor&& v)
    {
        IndexType idx;

        for (SizeType x = 0; x < m_dim(0); ++x)
        {
            idx(0) = x;
            for (SizeType y = 0; y < m_dim(1); ++y)
            {
                idx(1) = y;
                for (SizeType z = 0; z < m_dim(2); ++z)
                {
                    idx(2) = z;
                    v(m_cellData[getLinearIndex(x, y, z)], idx);
                }
            }
        }
    }

    /** Iterator Facade */

    /** Iterator which iterates ovel all entries, providing an index for each position*/
    template <typename IteratorType>
    class iterator : public std::iterator_traits<IteratorType>
    {
        public:
        using iterator_traits = std::iterator_traits<IteratorType>;

        explicit iterator(CartesianGrid* grid, IteratorType it) : m_grid(grid), m_it(it)
        {
            m_indices.setZero();  // always set to zero, even if it = begin() or it = end()
            // decrementing end() works (since decrementing indices works properly)
        }

        /** pre-increment ++it */
        iterator& operator++()
        {
            incrementIndex();
            ++m_it;
            return *this;
        }
        /** post-increment it++ */
        iterator operator++(int)
        {
            iterator it(*this);
            incrementIndex();
            operator++();
            return it;
        }

        /** pre-decrement --it */
        iterator& operator--()
        {
            decrementIndex();
            --m_it;
            return *this;
        }
        /** post-decrement it-- */
        iterator operator--(int)
        {
            iterator it(*this);
            decrementIndex();
            operator--();
            return it;
        }

        bool operator==(const iterator& rhs)
        {
            return m_it == rhs.m_it;
        }
        bool operator!=(const iterator& rhs)
        {
            return m_it != rhs.m_it;
        }

        typename iterator_traits::difference_type operator-(const iterator& rhs)
        {
            return m_it - rhs.m_it;
        }
        iterator& operator+=(typename iterator_traits::difference_type d)
        {
            m_it += d;
            return *this;
        }

        iterator& operator=(const iterator& rhs) = default;
        iterator(const iterator& r)              = default;

        typename iterator_traits::reference_type operator*()
        {
            return *m_it;
        }

        const IndexType& getIndices()
        {
            return m_indices;
        }

        private:
        void incrementIndex()
        {
            unsigned int i = 0;
            do
            {
                (++m_indices(i)) %=
                    m_grid->m_dim(i);  // (1,2,3) + 1 ==> (0,2,3)  (if maximum (2,2,4)) while loop goes further
            } while (m_indices(i++) == 0 && i < CartesianGrid::Dimension);
        }

        void decrementIndex()
        {
            unsigned int i = 0;
            // move to first non-zero entry, making all zero entries dim-1
            while (m_indices(i) == 0 && i < CartesianGrid::Dimension)
            {
                m_indices(i) = m_grid->m_dim(i) - 1;
                ++i;
            }
            // subtract if not at the end
            if (i < CartesianGrid::Dimension)
            {
                --m_indices(i);
            }
        }

        CartesianGrid* m_grid;
        IndexType      m_indices;
        IteratorType   m_it;
    };

    using IteratorType      = iterator<typename CellDataListType::iterator>;
    using ConstIteratorType = iterator<typename CellDataListType::const_iterator>;

    IteratorType begin()
    {
        return IteratorType(this, m_cellData->begin());
    }

    IteratorType end()
    {
        return IteratorType(this, m_cellData->end());
    }

    IteratorType cbegin()
    {
        return ConstIteratorType(this, m_cellData->cbegin());
    }

    IteratorType cend()
    {
        return ConstIteratorType(this, m_cellData->cend());
    }

    protected:
    template <typename T1, typename T2, typename T3>
    inline SizeType getLinearIndex(T1 x, T2 y, T3 z)
    {
        return x + m_dim(0) * (y + m_dim(1) * z);  // Colmajor Storage order
    }

    template <typename Derived>
    inline SizeType getLinearIndex(const ArrayBase<Derived>& index)
    {
        return index(0) + m_dim(0) * (index(1) + m_dim(1) * index(2));
    }

    CellDataListType m_cellData;

    Array3    m_dxyzInv;
    Array3    m_dxyz;
    IndexType m_dim;
    AABB3d    m_aabb;  ///< AABB in K coordinate system
    Matrix33  m_A_KI;  ///< an optional transformation matrix, default to identity! (from I system to the grid system K)
                       ///( rotation R_IK, from K system to I system)

    static char m_nbIndicesOff[26 * 3];

    private:
    // temporary
    mutable IndexLongType m_temp;

    static IndexLongType m_zero;
};

template <typename TCellData, typename TSize, typename TLongInt>
typename CartesianGrid<TCellData, TSize, TLongInt>::IndexLongType CartesianGrid<TCellData, TSize, TLongInt>::m_zero =
    CartesianGrid<TCellData, TSize, TLongInt>::IndexLongType(0, 0, 0);

template <typename TCellData, typename TSize, typename TLongInt>
char CartesianGrid<TCellData, TSize, TLongInt>::m_nbIndicesOff[26 * 3] = {
    1,  0,  0, 1,  0,  1,  1,  0,  -1,

    -1, 0,  0, -1, 0,  1,  -1, 0,  -1,

    0,  0,  1, 0,  0,  -1,

    0,  1,  0, 0,  1,  1,  0,  1,  -1,

    0,  -1, 0, 0,  -1, 1,  0,  -1, -1,

    1,  1,  0, 1,  1,  1,  1,  1,  -1,

    -1, 1,  0, -1, 1,  1,  -1, 1,  -1,

    1,  -1, 0, 1,  -1, 1,  1,  -1, -1,

    -1, -1, 0, -1, -1, 1,  -1, -1, -1,
};

#endif
