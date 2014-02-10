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

class StateRingPoolVisBackFront : public StatePool {
public:

    DECLERATIONS_STATEPOOL

    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateRingPoolVisBackFront(const unsigned int nSimBodies);
    ~StateRingPoolVisBackFront();

    /** @name Only accessed by Simulation Thread.
    * @{
    */
    boost::shared_ptr<DynamicsState > getSimBuffer();
    boost::shared_ptr<DynamicsState > advanceSimBuffer(bool & out_changed);
    /** @} */

    /** @name Only accessed by Loader Thread.
    * @{
    */
    boost::shared_ptr<DynamicsState > getLoadBuffer();
    boost::shared_ptr<DynamicsState > advanceLoadBuffer(bool & out_changed);
    /** @} */

    /** @name Only accessed by Visualization Thread.
    * @{
    */
    boost::shared_ptr<const DynamicsState > getVisBuffer();
    boost::shared_ptr<const DynamicsState > updateVisBuffer(bool & out_changed);
    boost::shared_ptr<const DynamicsState > updateVisBuffer();
    /** @} */

    /** @name Only accessed by if only Visualization Thread runs.
    * @{
    */
    template<typename RigidBodyStateContainerType>
    void resetStateRingPool(const RigidBodyStateContainerType & state_init);
    /** @} */


//    VectorUBody	getuInit(const unsigned idxObject);
//    void				setuInit(const VectorUBody & u, const unsigned idxObject);
//    VectorQBody	getqInit(const unsigned idxObject);
//    void				setqInit(const VectorQBody & q, const unsigned idxObject);


protected:

    const unsigned int m_nDofu, m_nDofq; // These are the global dimensions of q and u
    const unsigned int m_nDofuBody, m_nDofqBody, m_nSimBodies; // These are the dimensions for one Obj

    boost::mutex	m_mutexStateInit; ///< Mutex for the initial state.

    std::ofstream m_logfile;
#define POOL_SIZE 50 ///< The ring pool size, must not exceed 256 and be lower than 3!, because of the integer for the index in the StatePool class.
};
/** @} */



StateRingPoolVisBackFront::StateRingPoolVisBackFront(const unsigned int nSimBodies):
    StatePool(3),
    m_nSimBodies(nSimBodies),
    m_nDofqBody(NDOFqBody),
    m_nDofuBody(NDOFuBody),
    m_nDofq(m_nSimBodies * m_nDofqBody),
    m_nDofu(m_nSimBodies * m_nDofuBody) {

    // Add the 3 state pools, if m_state_pointer is deleted, all elements inside are deleted because of shared_ptr
    for(int i = 0; i < POOL_SIZE; i++) {
        m_pool.push_back(
            boost::shared_ptr<DynamicsState >(new DynamicsState(nSimBodies))
        );
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

    DynamicsState & state = *m_pool[0];

    if( state_init.size() != state.m_SimBodyStates.size()) {
        ERRORMSG(" initializeStatePool:: state_init has size: " << state_init.size() << "instead of " << state.m_SimBodyStates.size());
    }
    // Fill in the initial values

    for(auto it = state_init.begin(); it!= state_init.end(); it++) {
        unsigned int bodyNr = RigidBodyId::getBodyNr(it->first);
        if( bodyNr > state.m_SimBodyStates.size()) {
            ERRORMSG("body nr: " << bodyNr << " out of bound for DynamicState!")
        }
        state.m_SimBodyStates[bodyNr] =  it->second;
    }

    //*(m_pool[0]) = m_state_init; // Assignment operator
    *(m_pool[1]) = state; // Assignment operator

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

boost::shared_ptr<DynamicsState >
StateRingPoolVisBackFront::getSimBuffer() {
    //cout << " idx: " << (unsigned int)m_idx[1] << endl;
    return m_pool[m_idx[1]];
}


// ONLY USED IN SIM THREAD

boost::shared_ptr< DynamicsState >
StateRingPoolVisBackFront::advanceSimBuffer(bool & out_changed) {
    boost::mutex::scoped_lock l(m_change_pointer_mutex);
    // calculated next index!

    atomic_char next_index = (m_idx[1] + 1) % POOL_SIZE;
    // advance only if the next index is not the same as the load buffer!
    if( next_index == m_idx[2]) {
        out_changed = false;
        return m_pool[m_idx[1]];
    }

    m_idx[1] = next_index;
    out_changed = true;
#if LogToFileStateRingPool == 1
    m_logfile << "advanceSimBuffer()"<<endl;
    m_logfile << "vis: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t front: \t"<<(unsigned int)m_idx[2]<< endl;
#endif
    return m_pool[m_idx[1]];
}

// ONLY USED IN VISUALIZATION THREAD

boost::shared_ptr<const DynamicsState >
StateRingPoolVisBackFront::updateVisBuffer(bool & out_changed) {
    boost::mutex::scoped_lock l(m_change_pointer_mutex);

    if(m_idx[0] == m_idx[1]) {
        out_changed = false;
        return m_pool[m_idx[0]];
    }
    m_idx[0] = m_idx[1];
    out_changed = true;

#if LogToFileStateRingPool == 1
    m_logfile << "updateVisBuffer()"<<endl;
    m_logfile << "vis: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t front: \t"<<(unsigned int)m_idx[2]<< endl;
#endif

    return m_pool[m_idx[0]];
}


boost::shared_ptr<const DynamicsState >
StateRingPoolVisBackFront::updateVisBuffer() {
    bool out_changed;
    return updateVisBuffer(out_changed);
}


boost::shared_ptr<const DynamicsState > StateRingPoolVisBackFront::getVisBuffer() {
    return m_pool[m_idx[0]];
}




boost::shared_ptr<DynamicsState > StateRingPoolVisBackFront::getLoadBuffer() {
    return m_pool[m_idx[2]];
}


boost::shared_ptr<DynamicsState > StateRingPoolVisBackFront::advanceLoadBuffer( bool & out_changed ) {
    boost::mutex::scoped_lock l(m_change_pointer_mutex);
    // calculated next index!

    atomic_char next_index = (m_idx[2] + 1) % POOL_SIZE;
    // advance only if the next index is not the same as the load buffer!
    if( next_index == m_idx[0]) {
        out_changed = false;
        return m_pool[m_idx[2]];
    }

    m_idx[2] = next_index;
    out_changed = true;

#if LogToFileStateRingPool == 1
    m_logfile << "advanceLoadBuffer()"<<endl;
    m_logfile << "vis: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t front: \t"<<(unsigned int)m_idx[2]<< endl;
#endif
    return m_pool[m_idx[2]];
}


//=========================================================

#endif
