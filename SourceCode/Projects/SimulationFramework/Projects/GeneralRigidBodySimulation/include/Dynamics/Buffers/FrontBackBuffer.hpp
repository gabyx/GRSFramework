#ifndef FrontBackBuffer_hpp
#define FrontBackBuffer_hpp

#include <boost/shared_ptr.hpp>
#include "TypeDefs.hpp"

#include "DynamicsState.hpp"
#include "LogDefines.hpp"

/**
* @ingroup StatesAndBuffers
* @brief This is a class to store the front and back buffer pointers to the DynamicsState. This class is used in the timestepper.
*/
template< typename TLayoutConfig >
class FrontBackBuffer{

public:
  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrontBackBuffer()
  {
  
  };

  FrontBackBuffer(
    boost::shared_ptr<DynamicsState<TLayoutConfig> > pfront,
    boost::shared_ptr<const DynamicsState<TLayoutConfig> > pback)
  {
    m_pFront = pfront;
    m_pBack = pback;
  };

  ~FrontBackBuffer()
  {
    //DECONSTRUCTOR_MESSAGE
    m_pFront.reset();
    m_pBack.reset();
  };

  boost::shared_ptr<DynamicsState<TLayoutConfig> > m_pFront;       ///< The front buffer which is readable and writable.
  boost::shared_ptr<const DynamicsState<TLayoutConfig> > m_pBack;  ///< The back buffer which is only readable.

};


#endif