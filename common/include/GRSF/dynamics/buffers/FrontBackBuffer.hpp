// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_buffers_FrontBackBuffer_hpp
#define GRSF_dynamics_buffers_FrontBackBuffer_hpp

#include <boost/type_traits.hpp>
#include <memory>

#include "GRSF/dynamics/general/MyMatrixTypeDefs.hpp"
/**
* @ingroup StatesAndBuffers
* @brief This is a class to store the front and back buffer pointers to the DynamicsState. This class is used in the
* timestepper.
*/

struct FrontBackBufferPtrType
{
    struct SharedPtr
    {
    };
    struct NormalPtr
    {
    };
    struct NoPtr
    {
    };
};

struct FrontBackBufferMode
{
    struct BackConst
    {
    };
    struct NoConst
    {
    };
};

template <typename TBufferType, typename TBufferPtrType, typename TBufferMode = FrontBackBufferMode::NoConst>
class FrontBackBuffer;

// Specialization for shared ptr!
template <typename TBufferType>
class FrontBackBuffer<TBufferType, FrontBackBufferPtrType::SharedPtr, typename FrontBackBufferMode::BackConst>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrontBackBuffer(){};
    FrontBackBuffer(std::shared_ptr<TBufferType> pfront, std::shared_ptr<const TBufferType> pback)
        : m_pFront(pfront), m_pBack(pback)
    {
    }

    ~FrontBackBuffer()
    {
        m_pFront.reset();
        m_pBack.reset();
    };

    std::shared_ptr<TBufferType> m_pFront;       ///< The front buffer which is readable and writable.
    std::shared_ptr<const TBufferType> m_pBack;  ///< The back buffer which is only readable.
};

// Specialization for normal ptr, objects do not get deleted!
template <typename TBufferType>
class FrontBackBuffer<TBufferType, FrontBackBufferPtrType::NormalPtr, FrontBackBufferMode::BackConst>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FrontBackBuffer() : m_pFront(nullptr), m_pBack(nullptr)
    {
    }
    FrontBackBuffer(TBufferType* pfront, const TBufferType* pback) : m_pFront(pfront), m_pBack(pback){};
    ~FrontBackBuffer(){
        /** NO OBJECT DELETION! */
    };
    TBufferType* m_pFront;       ///< The front buffer which is readable and writable.
    const TBufferType* m_pBack;  ///< The back buffer which is only readable.
};

template <typename TBufferType>
class FrontBackBuffer<TBufferType, FrontBackBufferPtrType::NoPtr, FrontBackBufferMode::NoConst>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FrontBackBuffer(){};
    ~FrontBackBuffer(){};
    TBufferType m_front;  ///< The front buffer which is readable and writable.
    TBufferType m_back;   ///< The back buffer which is only readable and writable.
};

#endif
