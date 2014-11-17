#ifndef StatePoolVisBackFront_hpp
#define StatePoolVisBackFront_hpp


#include "LogDefines.hpp"

#include "StatePool.hpp"
#include "FrontBackBuffer.hpp"

#include "FileManager.hpp"

#include "InitialConditionBodies.hpp"

/**
* @ingroup StatesAndBuffers
* @brief This is the StatePoolVisBackFront class which is a spcialisation of the StatePool class.
* It provides exactly 3 states and three indices. The third index gives the state for the visualization thread, the second index
* gives the state for the back buffer and the first index is the state corresponding to the front buffer.
* @{
*/
class StatePoolVisBackFront : public StatePool<DynamicsState> {
public:

    DECLERATIONS_STATEPOOL
    DEFINE_LAYOUT_CONFIG_TYPES

    using StateType = DynamicsState;

    template<typename TRigidBodyIterator>
    StatePoolVisBackFront(TRigidBodyIterator beg, TRigidBodyIterator end);

    ~StatePoolVisBackFront();

    /** @name Only accessed by Simulation Thread.
    * @{
    */
    using FrontBackBufferType = FrontBackBuffer<StateType, FrontBackBufferPtrType::NormalPtr, FrontBackBufferMode::BackConst>;

    FrontBackBufferType getFrontBackBuffer();
    FrontBackBufferType swapFrontBackBuffer();
    /** @} */

    /** @name Only accessed by Visualization Thread.
    * @{
    */
    const StateType * updateVisBuffer(bool & out_changed);
    const StateType * updateVisBuffer();
    /** @} */

    /** @name Only accessed by if only Visualization Thread runs.
    * @{
    */
    template<typename RigidBodyStateContainerType>
    void resetStatePool(const RigidBodyStateContainerType & state_init);
    /** @} */


protected:
    std::ofstream m_logfile;
};
/** @} */


template<typename TRigidBodyIterator>
StatePoolVisBackFront::StatePoolVisBackFront(TRigidBodyIterator beg, TRigidBodyIterator end):
    StatePool(3)
{
    // Add the 3 state pools, managed by this class!
    m_pool.assign(3,StateType());

    m_pool[0].initSimStates<true>(beg,end);
    m_pool[1] = m_pool[0];
    m_pool[2] = m_pool[0];

//    for(auto & s : m_pool[0].m_SimBodyStates){
//        std::cout << RigidBodyId::getBodyIdString(s.m_id) << std::endl;
//    }

    m_idx[0] = 1; //front
    m_idx[1] = 0; //back
    m_idx[2] = 0; //vis

    // Init Log
    boost::filesystem::path filePath = FileManager::getSingleton().getLocalDirectoryPath();
    filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
    if(!boost::filesystem::exists(filePath)) {
        boost::filesystem::create_directories(filePath);
    }

    filePath /= "StatePoolVisSimLog.log";
    m_logfile.open(filePath.string().c_str());
    m_logfile.clear();
    m_logfile << "This is the State pool log file: each line describes the actual mode in which the state pool is\n";
}



template<typename RigidBodyStateContainerType>
void StatePoolVisBackFront::resetStatePool(const RigidBodyStateContainerType & state_init) {

    boost::mutex::scoped_lock l2(m_change_pointer_mutex);

    //initialize state buffer pointers
    m_idx[0] = 1; //front
    m_idx[1] = 0; //back
    m_idx[2] = 0; //vis

    m_pool[0].reset();
    m_pool[0].applyBodyStates<false>(state_init);

    // Fill in the initial values
    m_pool[1] = m_pool[0];

#if STATEPOOLLOG_TOFILE == 1
    m_logfile << "front: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t vis: \t"<<(unsigned int)m_idx[2]<< endl;
#endif
}


#endif
