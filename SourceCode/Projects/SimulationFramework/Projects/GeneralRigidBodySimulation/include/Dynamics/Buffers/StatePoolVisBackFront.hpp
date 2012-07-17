#ifndef StatePoolVisBackFront_hpp
#define StatePoolVisBackFront_hpp

#include "StatePool.hpp"
#include "FrontBackBuffer.hpp"
#include "LogDefines.hpp"


/**
* @ingroup StatesAndBuffers
* @brief This is the StatePoolVisBackFront class which is a spcialisation of the StatePool class.
* It provides exactly 3 states and three indices. The third index gives the state for the visualization thread, the second index
* gives the state for the back buffer and the first index is the state corresponding to the front buffer.
* @{
*/
template< typename TLayoutConfig >
class StatePoolVisBackFront : public StatePool<TLayoutConfig>{
public:

  DECLERATIONS_STATEPOOL

  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StatePoolVisBackFront(const unsigned int nSimBodies);
  ~StatePoolVisBackFront();

  /** @name Only accessed by Simulation Thread.
  * @{
  */
  typedef FrontBackBuffer<DynamicsState<TLayoutConfig>, FrontBackBufferPtrType::SharedPtr, FrontBackBufferMode::BackConst> FrontBackBufferType;

  FrontBackBufferType getFrontBackBuffer();
  FrontBackBufferType swapFrontBackBuffer();
  /** @} */

  /** @name Only accessed by Visualization Thread.
  * @{
  */
  boost::shared_ptr<const DynamicsState<TLayoutConfig> > updateVisBuffer(bool & out_changed);
  boost::shared_ptr<const DynamicsState<TLayoutConfig> > updateVisBuffer();
  /** @} */

  /** @name Only accessed by if only Visualization Thread runs.
  * @{
  */
  void initializeStatePool(const DynamicsState<TLayoutConfig> & state_init);
  void initializeStatePool(const std::vector<DynamicsState<TLayoutConfig> > & state_initList); ///< Used to initialize from a list of DynamicStates which count up to the number of simulated bodies.
  void resetStatePool();
  /** @} */

  VectorUObj	getuInit(const unsigned idxObject);
  void						setuInit(const VectorUObj & u, const unsigned idxObject);
  VectorQObj	getqInit(const unsigned idxObject);
  void						setqInit(const VectorQObj & q, const unsigned idxObject);

protected:
  std::ofstream m_logfile;

  const unsigned int m_nDofu, m_nDofq; // These are the global dimensions of q and u
  const unsigned int m_nDofuObj, m_nDofqObj, m_nSimBodies; // These are the dimensions for one Obj

  boost::mutex	m_mutexStateInit; ///< Mutex for the initial state.
  DynamicsState<TLayoutConfig> m_state_init; ///< The initial state for the system.
};
/** @} */


template<typename TLayoutConfig>
StatePoolVisBackFront<TLayoutConfig>::StatePoolVisBackFront(const unsigned int nSimBodies):
StatePool<TLayoutConfig>(3),
m_state_init(nSimBodies),
m_nSimBodies(nSimBodies),
m_nDofqObj(NDOFqObj),
m_nDofuObj(NDOFuObj),
m_nDofq(m_nSimBodies * m_nDofqObj),
m_nDofu(m_nSimBodies * m_nDofuObj)
{

  // Add the 3 state pools, if m_state_pointer is deleted, all elements inside are deleted because of shared_ptr
  m_pool.push_back(
    boost::shared_ptr<DynamicsState<TLayoutConfig> >(new DynamicsState<TLayoutConfig>(nSimBodies))
    );
  m_pool.push_back(
    boost::shared_ptr<DynamicsState<TLayoutConfig> >(new DynamicsState<TLayoutConfig>(nSimBodies))
    );

  m_pool.push_back(
    boost::shared_ptr<DynamicsState<TLayoutConfig> >(new DynamicsState<TLayoutConfig>(nSimBodies))
    );

  m_idx[0] = 1; //front
  m_idx[1] = 0; //back
  m_idx[2] = 0; //vis

  initializeStatePool( DynamicsState<TLayoutConfig>(nSimBodies) );


  // Init Log
  boost::filesystem::path filePath = GLOBAL_LOG_FOLDER_DIRECTORY;
    if(!boost::filesystem::exists(filePath)){
            boost::filesystem::create_directories(filePath);
    }

  filePath /= "StatePoolVisSimLog.log";
  m_logfile.open(filePath.string());
  m_logfile.clear();
  m_logfile << "This is the State pool log file: each line describes the actual mode in which the state pool is\n";
}


template<typename TLayoutConfig>
void StatePoolVisBackFront<TLayoutConfig>::initializeStatePool(const DynamicsState<TLayoutConfig> & state_init){

  boost::mutex::scoped_lock l1(m_mutexStateInit);
  boost::mutex::scoped_lock l2(m_change_pointer_mutex);

  m_state_init = state_init; // Assignement operator
  // Fill in the initial values

  *(m_pool[0]) = m_state_init; // Assignment operator

#if LogToFileStatePool == 1
  m_logfile << "front: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t vis: \t"<<(unsigned int)m_idx[2]<< endl;
#endif
}

template<typename TLayoutConfig>
void StatePoolVisBackFront<TLayoutConfig>::initializeStatePool(const std::vector<DynamicsState<TLayoutConfig> > & state_initList){

  boost::mutex::scoped_lock l1(m_mutexStateInit);
  boost::mutex::scoped_lock l2(m_change_pointer_mutex);

  unsigned int index = 0;
  for(int i=0; i< state_initList.size(); i++){

     for(int k = 0; k < state_initList[i].m_SimBodyStates.size(); k++){
       m_state_init.m_SimBodyStates[index + k] = state_initList[i].m_SimBodyStates[k];
       //cout <<"State1: q:" << m_state_init.m_SimBodyStates[index + k].m_q.transpose()  <<endl<<" u: "<< m_state_init.m_SimBodyStates[index + k].m_u.transpose() <<endl;
     }

     index += (unsigned int) state_initList[i].m_SimBodyStates.size();
  }

  // Fill in the initial values

  *(m_pool[0]) = m_state_init; // Assignment operator

#if LogToFileStatePool == 1
  m_logfile << "front: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t vis: \t"<<(unsigned int)m_idx[2]<< endl;
#endif
}

template<typename TLayoutConfig>
StatePoolVisBackFront<TLayoutConfig>::~StatePoolVisBackFront()
{
  DECONSTRUCTOR_MESSAGE
  m_logfile.close();
}

// ==========================
template<typename TLayoutConfig>
typename TLayoutConfig::VectorQObj StatePoolVisBackFront<TLayoutConfig>::getqInit(const unsigned idxObject)
{
  static typename TLayoutConfig::VectorQObj  q;
  m_mutexStateInit.lock();
  q = m_state_init.m_SimBodyStates[idxObject].m_q;
  m_mutexStateInit.unlock();
  return q;
}

template<typename TLayoutConfig>
void StatePoolVisBackFront<TLayoutConfig>::setqInit(const VectorQObj & q ,const unsigned idxObject)
{
  m_mutexStateInit.lock();
  m_state_init.m_SimBodyStates[idxObject].m_q = q;
  m_mutexStateInit.unlock();
  return;
}


template<typename TLayoutConfig>
typename TLayoutConfig::VectorUObj StatePoolVisBackFront<TLayoutConfig>::getuInit(const unsigned idxObject)
{
  typename TLayoutConfig::VectorUObj u;
  m_mutexStateInit.lock();
  u = m_state_init.m_SimBodyStates[idxObject].m_u;
  m_mutexStateInit.unlock();
  return u;
}

template<typename TLayoutConfig>
void StatePoolVisBackFront<TLayoutConfig>::setuInit(const VectorUObj & u, const unsigned idxObject)
{
  m_mutexStateInit.lock();
  m_state_init.m_SimBodyStates[idxObject].m_u = u;
  m_mutexStateInit.unlock();
  return;
}


template<typename TLayoutConfig>
void StatePoolVisBackFront<TLayoutConfig>::resetStatePool()
{
  boost::mutex::scoped_lock l(m_change_pointer_mutex);

  //initialize state buffer pointers
  m_idx[0] = 1; //front
  m_idx[1] = 0; //back
  m_idx[2] = 0; //vis

  // Fill in the initial values
  *m_pool[0] = m_state_init;
  *m_pool[1] = m_state_init;

  return;
}



// ONLY USED IN SIM THREAD
template<typename TLayoutConfig>
typename StatePoolVisBackFront<TLayoutConfig>::FrontBackBufferType
StatePoolVisBackFront<TLayoutConfig>::getFrontBackBuffer() {
  return FrontBackBufferType(m_pool[m_idx[0]], m_pool[m_idx[1]]);
}

// ONLY USED IN SIM THREAD
template<typename TLayoutConfig>
typename StatePoolVisBackFront<TLayoutConfig>::FrontBackBufferType
StatePoolVisBackFront<TLayoutConfig>::swapFrontBackBuffer()
{
  boost::mutex::scoped_lock l(m_change_pointer_mutex);
  if(m_idx[1] != m_idx[2]) {
    std::swap(m_idx[0], m_idx[1]);
#if LogToFileStatePool == 1
    m_logfile << "swapFrontBackBuffer()"<<endl;
    m_logfile << "front: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t vis: \t"<<(unsigned int)m_idx[2]<< endl;
#endif
    return FrontBackBufferType(m_pool[m_idx[0]], m_pool[m_idx[1]]);
  }
  int new_front = 3 - m_idx[0] - m_idx[1]; //select the buffer which is currently unused
  m_idx[1] = m_idx[0];
  m_idx[0] = new_front;

#if LogToFileStatePool == 1
  m_logfile << "swapFrontBackBuffer()"<<endl;
  m_logfile << "front: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t vis: \t"<<(unsigned int)m_idx[2]<< endl;
#endif

  return FrontBackBufferType(m_pool[m_idx[0]], m_pool[m_idx[1]]);
}

// ONLY USED IN VISUALIZATION THREAD
template<typename TLayoutConfig>
boost::shared_ptr<const DynamicsState<TLayoutConfig> >
StatePoolVisBackFront<TLayoutConfig>::updateVisBuffer(bool & out_changed)
{
  boost::mutex::scoped_lock l(m_change_pointer_mutex);
  if(m_idx[2] == m_idx[1]){
    out_changed = false;
    return m_pool[m_idx[2]];
  }

  m_idx[2] = m_idx[1];
  out_changed = true;

#if LogToFileStatePool == 1
  m_logfile << "updateVisBuffer()"<<endl;
  m_logfile << "front: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t vis: \t"<<(unsigned int)m_idx[2]<< endl;
#endif

  return m_pool[m_idx[2]];
}

template<typename TLayoutConfig>
boost::shared_ptr<const DynamicsState<TLayoutConfig> >
StatePoolVisBackFront<TLayoutConfig>::updateVisBuffer()
{
  bool changed;
  return updateVisBuffer(changed);
}

//=========================================================


#endif
