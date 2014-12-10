#ifndef GMSF_Dynamics_Buffers_FrontBackBuffer_hpp
#define GMSF_Dynamics_Buffers_FrontBackBuffer_hpp

#include <memory>
#include <boost/type_traits.hpp>

/**
* @ingroup StatesAndBuffers
* @brief This is a class to store the front and back buffer pointers to the DynamicsState. This class is used in the timestepper.
*/

struct FrontBackBufferPtrType {
    struct SharedPtr {};
    struct NormalPtr {};
    struct NoPtr {};
};

struct FrontBackBufferMode {
    struct BackConst {};
    struct NoConst {};
};

template< typename TBufferType, typename TBufferPtrType, typename TBufferMode = FrontBackBufferMode::NoConst> class FrontBackBuffer;


// Specialization for shared ptr!
template< typename TBufferType >
class FrontBackBuffer< TBufferType, FrontBackBufferPtrType::SharedPtr, typename FrontBackBufferMode::BackConst > {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrontBackBuffer() {};
    FrontBackBuffer(std::shared_ptr<TBufferType > pfront, std::shared_ptr<const TBufferType > pback):
        m_pFront(pfront), m_pBack(pback) {
    }

    ~FrontBackBuffer() {m_pFront.reset(); m_pBack.reset();};

    std::shared_ptr<TBufferType >       m_pFront;       ///< The front buffer which is readable and writable.
    std::shared_ptr<const TBufferType > m_pBack;        ///< The back buffer which is only readable.
};


// Specialization for normal ptr, objects do not get deleted!
template< typename TBufferType >
class FrontBackBuffer< TBufferType, FrontBackBufferPtrType::NormalPtr ,  FrontBackBufferMode::BackConst> {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FrontBackBuffer(): m_pFront(nullptr), m_pBack(nullptr) {}
    FrontBackBuffer(TBufferType * pfront, const TBufferType * pback): m_pFront(pfront), m_pBack(pback) {};
    ~FrontBackBuffer() {
        /** NO OBJECT DELETION! */
    };
    TBufferType * m_pFront;       ///< The front buffer which is readable and writable.
    const TBufferType * m_pBack;        ///< The back buffer which is only readable.
};


template< typename TBufferType >
class FrontBackBuffer< TBufferType, FrontBackBufferPtrType::NoPtr , FrontBackBufferMode::NoConst> {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FrontBackBuffer() {};
    ~FrontBackBuffer() {};
    TBufferType  m_front;       ///< The front buffer which is readable and writable.
    TBufferType  m_back;        ///< The back buffer which is only readable and writable.

};


#endif
