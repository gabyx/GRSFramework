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
class StatePoolVisBackFront : public StatePool {
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
    std::shared_ptr<const DynamicsState > updateVisBuffer(bool & out_changed);
    std::shared_ptr<const DynamicsState > updateVisBuffer();
    /** @} */

    /** @name Only accessed by if only Visualization Thread runs.
    * @{
    */
    template<typename RigidBodyStateContainerType>
    void resetStatePool(const RigidBodyStateContainerType & state_init);
    /** @} */

//    VectorUBody	getuInit(const unsigned idxObject);
//    void						setuInit(const VectorUBody & u, const unsigned idxObject);
//    VectorQBody	getqInit(const unsigned idxObject);
//    void						setqInit(const VectorQBody & q, const unsigned idxObject);

protected:
    std::ofstream m_logfile;

    const unsigned int m_nSimBodies; // These are the dimensions for one Obj

};
/** @} */


template<typename RigidBodyStateContainerType>
void StatePoolVisBackFront::resetStatePool(const RigidBodyStateContainerType & state_init) {

    boost::mutex::scoped_lock l2(m_change_pointer_mutex);

    //initialize state buffer pointers
    m_idx[0] = 1; //front
    m_idx[1] = 0; //back
    m_idx[2] = 0; //vis

    DynamicsState & state = *m_pool[0];

    state.m_StateType = DynamicsState::NONE;
    state.m_t = 0;

    InitialConditionBodies::applyBodyStatesTo(state_init,state);

    // Fill in the initial values
    //*m_pool[0] = state;
    *m_pool[1] = state;

#if LogToFileStatePool == 1
    m_logfile << "front: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t vis: \t"<<(unsigned int)m_idx[2]<< endl;
#endif
}


#endif
