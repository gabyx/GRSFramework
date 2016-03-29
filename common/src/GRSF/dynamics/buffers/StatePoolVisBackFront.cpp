// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/dynamics/buffers/StatePoolVisBackFront.hpp"






StatePoolVisBackFront::~StatePoolVisBackFront() {
    DECONSTRUCTOR_MESSAGE
    m_logfile.close();
}

// ==========================

//typename StatePoolVisBackFront::VectorQBody StatePoolVisBackFront::getqInit(const unsigned idxObject)
//{
//  static typename LayoutConfigType::VectorQBody  q;
//  m_mutexStateInit.lock();
//  q = m_state_init.m_SimBodyStates[idxObject].m_q;
//  m_mutexStateInit.unlock();
//  return q;
//}
//
//
//void StatePoolVisBackFront::setqInit(const VectorQBody & q ,const unsigned idxObject)
//{
//  m_mutexStateInit.lock();
//  m_state_init.m_SimBodyStates[idxObject].m_q = q;
//  m_mutexStateInit.unlock();
//  return;
//}



//typename StatePoolVisBackFront::VectorUBody StatePoolVisBackFront::getuInit(const unsigned idxObject)
//{
//  typename LayoutConfigType::VectorUBody u;
//  m_mutexStateInit.lock();
//  u = m_state_init.m_SimBodyStates[idxObject].m_u;
//  m_mutexStateInit.unlock();
//  return u;
//}
//
//
//void StatePoolVisBackFront::setuInit(const VectorUBody & u, const unsigned idxObject)
//{
//  m_mutexStateInit.lock();
//  m_state_init.m_SimBodyStates[idxObject].m_u = u;
//  m_mutexStateInit.unlock();
//  return;
//}



//void StatePoolVisBackFront::resetStatePool()
//{
//  boost::mutex::scoped_lock l(m_change_pointer_mutex);
//
//  //initialize state buffer pointers
//  m_idx[0] = 1; //front
//  m_idx[1] = 0; //back
//  m_idx[2] = 0; //vis
//
//  // Fill in the initial values
//  *m_pool[0] = m_state_init;
//  *m_pool[1] = m_state_init;
//
//  return;
//}



// ONLY USED IN SIM THREAD

typename StatePoolVisBackFront::FrontBackBufferType
StatePoolVisBackFront::getFrontBackBuffer() {
    return FrontBackBufferType(&m_pool[m_idx[0]], &m_pool[m_idx[1]]);
}

// ONLY USED IN SIM THREAD

typename StatePoolVisBackFront::FrontBackBufferType
StatePoolVisBackFront::swapFrontBackBuffer() {
    boost::mutex::scoped_lock l(m_change_pointer_mutex);
    if(m_idx[1] != m_idx[2]) {
        std::swap(m_idx[0], m_idx[1]);
#if STATEPOOLLOG_TOFILE == 1
        m_logfile << "swapFrontBackBuffer()"<<endl;
        m_logfile << "front: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t vis: \t"<<(unsigned int)m_idx[2]<< endl;
#endif
        return FrontBackBufferType(&m_pool[m_idx[0]], &m_pool[m_idx[1]]);
    }
    int new_front = 3 - m_idx[0] - m_idx[1]; //select the buffer which is currently unused
    m_idx[1] = m_idx[0];
    m_idx[0] = new_front;

#if STATEPOOLLOG_TOFILE == 1
    m_logfile << "swapFrontBackBuffer()"<<endl;
    m_logfile << "front: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t vis: \t"<<(unsigned int)m_idx[2]<< endl;
#endif

    return FrontBackBufferType(&m_pool[m_idx[0]], &m_pool[m_idx[1]]);
}

// ONLY USED IN VISUALIZATION THREAD

const DynamicsState *
StatePoolVisBackFront::updateVisBuffer(bool & out_changed) {
    boost::mutex::scoped_lock l(m_change_pointer_mutex);
    if(m_idx[2] == m_idx[1]) {
        out_changed = false;
        return &m_pool[m_idx[2]];
    }

    m_idx[2] = m_idx[1];
    out_changed = true;

#if STATEPOOLLOG_TOFILE == 1
    m_logfile << "updateVisBuffer()"<<endl;
    m_logfile << "front: \t"<<(unsigned int)m_idx[0]<< "\t back: \t"<<(unsigned int)m_idx[1]<< "\t vis: \t"<<(unsigned int)m_idx[2]<< endl;
#endif

    return &m_pool[m_idx[2]];
}


const DynamicsState *
StatePoolVisBackFront::updateVisBuffer() {
    bool changed;
    return updateVisBuffer(changed);
}

//=========================================================

