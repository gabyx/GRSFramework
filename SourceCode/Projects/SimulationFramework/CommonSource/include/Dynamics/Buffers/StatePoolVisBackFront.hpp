#ifndef StatePoolVisBackFront_hpp
#define StatePoolVisBackFront_hpp


#include "LogDefines.hpp"

#include "StatePool.hpp"
#include "FrontBackBuffer.hpp"

#include "FileManager.hpp"


/**
* @ingroup StatesAndBuffers
* @brief This is the StatePoolVisBackFront class which is a spcialisation of the StatePool class.
* It provides exactly 3 states and three indices. The third index gives the state for the visualization thread, the second index
* gives the state for the back buffer and the first index is the state corresponding to the front buffer.
* @{
*/
class StatePoolVisBackFront : public StatePool{
public:

  DECLERATIONS_STATEPOOL

  DEFINE_LAYOUT_CONFIG_TYPES

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StatePoolVisBackFront(const unsigned int nSimBodies);
  ~StatePoolVisBackFront();

  /** @name Only accessed by Simulation Thread.
  * @{
  */
  typedef FrontBackBuffer<DynamicsState, FrontBackBufferPtrType::SharedPtr, FrontBackBufferMode::BackConst> FrontBackBufferType;

  FrontBackBufferType getFrontBackBuffer();
  FrontBackBufferType swapFrontBackBuffer();
  /** @} */

  /** @name Only accessed by Visualization Thread.
  * @{
  */
  boost::shared_ptr<const DynamicsState > updateVisBuffer(bool & out_changed);
  boost::shared_ptr<const DynamicsState > updateVisBuffer();
  /** @} */

  /** @name Only accessed by if only Visualization Thread runs.
  * @{
  */
  void initializeStatePool(const DynamicsState & state_init);
  void initializeStatePool(const std::vector<DynamicsState > & state_initList); ///< Used to initialize from a list of DynamicStates which count up to the number of simulated bodies.
  void resetStatePool();
  /** @} */

  VectorUBody	getuInit(const unsigned idxObject);
  void						setuInit(const VectorUBody & u, const unsigned idxObject);
  VectorQBody	getqInit(const unsigned idxObject);
  void						setqInit(const VectorQBody & q, const unsigned idxObject);

protected:
  std::ofstream m_logfile;

  const unsigned int m_nDofu, m_nDofq; // These are the global dimensions of q and u
  const unsigned int m_nDofuBody, m_nDofqBody, m_nSimBodies; // These are the dimensions for one Obj

  boost::mutex	m_mutexStateInit; ///< Mutex for the initial state.
  DynamicsState m_state_init; ///< The initial state for the system.
};
/** @} */


#endif
