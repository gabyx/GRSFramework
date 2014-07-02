#ifndef StateRingPoolVisBackFront_hpp
#define StateRingPoolVisBackFront_hpp

#include <boost/filesystem.hpp>

#include "AssertionDebug.hpp"
#include "FileManager.hpp"

#include "StatePool.hpp"
#include "FrontBackBuffer.hpp"

#include "LogDefines.hpp"


/**
* @ingroup StatesAndBuffers
* @brief This is the StateRingPoolVisBackFront class which is a spcialisation of the StatePool class.
* It provides a ring buffer with several states and three indices. The first index gives the state for the visualization thread, the second index
* gives the state for the back buffer and the third index is the state corresponding to the front buffer.
* @{
*/

class StateRingPoolVisBackFront : public StatePool<DynamicsState> {
public:

    DECLERATIONS_STATEPOOL

    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DynamicsState StateType;

    template<typename RigidBodyIterator>
    StateRingPoolVisBackFront(RigidBodyIterator itBegin, RigidBodyIterator itEnd);

    ~StateRingPoolVisBackFront();

    /** @name Only accessed by Simulation Thread.
    * @{
    */
    StateType * getSimBuffer();
    StateType * advanceSimBuffer(bool & out_changed);
    /** @} */

    /** @name Only accessed by Loader Thread.
    * @{
    */
    StateType * getLoadBuffer();
    StateType * advanceLoadBuffer(bool & out_changed);
    /** @} */

    /** @name Only accessed by Visualization Thread.
    * @{
    */
    const StateType * getVisBuffer();
    const StateType * updateVisBuffer(bool & out_changed);
    const StateType * updateVisBuffer();
    /** @} */

    /** @name Only accessed by if only Visualization Thread runs.
    * @{
    */
    template<typename RigidBodyStateContainerType>
    void resetStateRingPool(const RigidBodyStateContainerType & state_init);
    /** @} */
protected:

    boost::mutex	m_mutexStateInit; ///< Mutex for the initial state.
    std::ofstream m_logfile;
#define POOL_SIZE 50 ///< The ring pool size, must not exceed 256 and be lower than 3!, because of the integer for the index in the StatePool class.
};
/** @} */



template<typename RigidBodyIterator>
StateRingPoolVisBackFront::StateRingPoolVisBackFront(RigidBodyIterator itBegin, RigidBodyIterator itEnd):
    StatePool(3)
{

    // Add the 3 state pools, if m_state_pointer is deleted, all elements inside are deleted because of shared_ptr
    m_pool.assign(POOL_SIZE, DynamicsState());

    m_pool[0].initSimStates<true>(itBegin,itEnd);
    for(int i=1;i<POOL_SIZE;i++){
        m_pool[i] = m_pool[0];
    }

    m_idx[0] = 0; // vis
    m_idx[1] = 0; // back
    m_idx[2] = 1; // front

    // Init Log
    boost::filesystem::path filePath = FileManager::getSingletonPtr()->getLocalDirectoryPath();
    filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
    if(!boost::filesystem::exists(filePath)) {
        boost::filesystem::create_directories(filePath);
    }
    filePath /= "StateRingPoolVisSimLoadLog.log";
    m_logfile.open(filePath.string().c_str());
    m_logfile.clear();
    m_logfile << "This is the State Ring Pool log file: each line describes the actual mode in which the state pool is\n";
}


template<typename RigidBodyStateContainerType>
void StateRingPoolVisBackFront::resetStateRingPool(const RigidBodyStateContainerType & state_init) {

    boost::mutex::scoped_lock l2(m_change_pointer_mutex);

    //initialize state buffer pointers
    m_idx[0] = 0; // vis
    m_idx[1] = 0; // back
    m_idx[2] = 1; // front

    StateType & state = m_pool[0];
    state.reset();

    InitialConditionBodies::applyBodyStatesTo(state_init,state);

    //(m_pool[0]) = m_state_init; // Assignment operator
    m_pool[1] = state; // Assignment operator

#if LogToFileStateRingPool == 1
    m_logfile << "resetStateRingPool()"<<endl;
    m_logfile << "vis: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t front: \t"<<(unsigned int)m_idx[2]<< endl;
#endif
}



StateRingPoolVisBackFront::~StateRingPoolVisBackFront() {
    DECONSTRUCTOR_MESSAGE
    m_logfile.close();
}

// ==========================

//typename StateRingPoolVisBackFront::VectorQBody StateRingPoolVisBackFront::getqInit(const unsigned idxObject) {
//    static typename StateRingPoolVisBackFront::VectorQBody  q;
//    m_mutexStateInit.lock();
//    q = m_state_init.m_SimBodyStates[idxObject].m_q;
//    m_mutexStateInit.unlock();
//    return q;
//}
//
//
//void StateRingPoolVisBackFront::setqInit(const VectorQBody & q ,const unsigned idxObject) {
//    m_mutexStateInit.lock();
//    m_state_init.m_SimBodyStates[idxObject].m_q = q;
//    m_mutexStateInit.unlock();
//    return;
//}


//
//typename StateRingPoolVisBackFront::VectorUBody StateRingPoolVisBackFront::getuInit(const unsigned idxObject) {
//    typename StateRingPoolVisBackFront::VectorUBody u;
//    m_mutexStateInit.lock();
//    u = m_state_init.m_SimBodyStates[idxObject].m_u;
//    m_mutexStateInit.unlock();
//    return u;
//}
//
//
//void StateRingPoolVisBackFront::setuInit(const VectorUBody & u, const unsigned idxObject) {
//    m_mutexStateInit.lock();
//    m_state_init.m_SimBodyStates[idxObject].m_u = u;
//    m_mutexStateInit.unlock();
//    return;
//}



//void StateRingPoolVisBackFront::resetStateRingPool()
//{
//  boost::mutex::scoped_lock l1(m_mutexStateInit);
//  boost::mutex::scoped_lock l(m_change_pointer_mutex);
//
//  //initialize state buffer pointers
//  m_idx[0] = 0; // vis
//  m_idx[1] = 0; // back
//  m_idx[2] = 1; // front
//
//
//  // Fill in the initial values
//  //for(int i=0;i< POOL_SIZE;i++){
//  *m_pool[0] = m_state_init;
//  *m_pool[1] = m_state_init;
//  //}
//
//#if LogToFileStateRingPool == 1
//    m_logfile << "resetStateRingPool()"<<endl;
//  m_logfile << "vis: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t front: \t"<<(unsigned int)m_idx[2]<< endl;
//#endif
//
//  return;
//}


// ONLY USED IN SIM THREAD

StateRingPoolVisBackFront::StateType *
StateRingPoolVisBackFront::getSimBuffer() {
    //cout << " idx: " << (unsigned int)m_idx[1] << endl;
    return &m_pool[m_idx[1]];
}


// ONLY USED IN SIM THREAD

StateRingPoolVisBackFront::StateType *
StateRingPoolVisBackFront::advanceSimBuffer(bool & out_changed) {
    boost::mutex::scoped_lock l(m_change_pointer_mutex);
    // calculated next index!

    atomic_char next_index = (m_idx[1] + 1) % POOL_SIZE;
    // advance only if the next index is not the same as the load buffer!
    if( next_index == m_idx[2]) {
        out_changed = false;
        return &m_pool[m_idx[1]];
    }

    m_idx[1] = next_index;
    out_changed = true;
#if LogToFileStateRingPool == 1
    m_logfile << "advanceSimBuffer()"<<endl;
    m_logfile << "vis: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t front: \t"<<(unsigned int)m_idx[2]<< endl;
#endif
    return &m_pool[m_idx[1]];
}

// ONLY USED IN VISUALIZATION THREAD

const StateRingPoolVisBackFront::StateType *
StateRingPoolVisBackFront::updateVisBuffer(bool & out_changed) {
    boost::mutex::scoped_lock l(m_change_pointer_mutex);

    if(m_idx[0] == m_idx[1]) {
        out_changed = false;
        return &m_pool[m_idx[0]];
    }
    m_idx[0] = m_idx[1];
    out_changed = true;

#if LogToFileStateRingPool == 1
    m_logfile << "updateVisBuffer()"<<endl;
    m_logfile << "vis: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t front: \t"<<(unsigned int)m_idx[2]<< endl;
#endif

    return &m_pool[m_idx[0]];
}


const StateRingPoolVisBackFront::StateType *
StateRingPoolVisBackFront::updateVisBuffer() {
    bool out_changed;
    return updateVisBuffer(out_changed);
}


const StateRingPoolVisBackFront::StateType * StateRingPoolVisBackFront::getVisBuffer() {
    return &m_pool[m_idx[0]];
}




StateRingPoolVisBackFront::StateType * StateRingPoolVisBackFront::getLoadBuffer() {
    return &m_pool[m_idx[2]];
}


StateRingPoolVisBackFront::StateType * StateRingPoolVisBackFront::advanceLoadBuffer( bool & out_changed ) {
    boost::mutex::scoped_lock l(m_change_pointer_mutex);
    // calculated next index!

    atomic_char next_index = (m_idx[2] + 1) % POOL_SIZE;
    // advance only if the next index is not the same as the load buffer!
    if( next_index == m_idx[0]) {
        out_changed = false;
        return &m_pool[m_idx[2]];
    }

    m_idx[2] = next_index;
    out_changed = true;

#if LogToFileStateRingPool == 1
    m_logfile << "advanceLoadBuffer()"<<endl;
    m_logfile << "vis: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t front: \t"<<(unsigned int)m_idx[2]<< endl;
#endif
    return &m_pool[m_idx[2]];
}


//=========================================================

#endif
