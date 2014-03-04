#include "SimulationManagerBase.hpp"

#include <fstream>

#include "InputContext.hpp"
#include "LogDefines.hpp"



SimulationManagerBase::SimulationManagerBase():
    m_bSimThreadRunning(false),
    m_eSimThreadRunning(NONE),
    m_averageIterationTime(0.0),
    m_maxIterationTime(0.0),
    m_nCurrentContacts(0),
    m_bThreadToBeStopped(false),

    m_barrier_start(2) {
    m_lastTime = 0;
    m_pTimelineSimulation = boost::shared_ptr<boost::timer::cpu_timer>(new boost::timer::cpu_timer);
    m_pTimelineSimulation->stop();

    // Setup timeScale List;
    m_timeScaleList.push_back(0);
    for(int i=1; i<10; i++) {
        m_timeScaleList.push_back(i*0.01);
    }
    for(int i=1; i<=30; i++) {
        m_timeScaleList.push_back(i*0.1);
    }
    m_timeScaleListIdx = 19;
    m_timeScale = m_timeScaleList[m_timeScaleListIdx];


    m_bPauseEnabled = false;
};

SimulationManagerBase::~SimulationManagerBase() {
    DECONSTRUCTOR_MESSAGE
};

bool SimulationManagerBase::isSimThreadRunning() {
    boost::mutex::scoped_lock l(m_bSimThreadRunning_mutex);
    return m_bSimThreadRunning;
}

void SimulationManagerBase::setSimThreadRunning( bool value ) {
    boost::mutex::scoped_lock l(m_bSimThreadRunning_mutex);
    m_bSimThreadRunning = value;
}
bool  SimulationManagerBase::isSimThreadToBeStopped() {
    boost::mutex::scoped_lock l(m_bThreadToBeStopped_mutex);
    return m_bThreadToBeStopped;
};
void  SimulationManagerBase::setThreadToBeStopped(bool stop) {
    boost::mutex::scoped_lock l(m_bThreadToBeStopped_mutex);
    m_bThreadToBeStopped = stop;
};

double SimulationManagerBase::getTimelineSimulation() {
    double x;
    m_mutexTimelineSimulation.lock();
    x = ((double)m_pTimelineSimulation->elapsed().wall) * 1e-9 * m_timeScale + m_lastTime;
    m_mutexTimelineSimulation.unlock();
    return x;
};

void  SimulationManagerBase::resetTimelineSimulation() {
    m_mutexTimelineSimulation.lock();
    m_pTimelineSimulation->stop();
    m_pTimelineSimulation->start();
    m_lastTime = 0;
    m_bPauseEnabled = false;
    m_mutexTimelineSimulation.unlock();
};

void  SimulationManagerBase::stopTimelineSimulation() {
    m_mutexTimelineSimulation.lock();
    m_pTimelineSimulation->stop();
    m_lastTime = 0;
    m_bPauseEnabled = false;
    m_mutexTimelineSimulation.unlock();
};

void SimulationManagerBase::getIterationTime(double & averageIterationTime, double & maxIterationTime) {
    m_mutexIterationtime.lock();
    averageIterationTime = m_averageIterationTime;
    maxIterationTime =  m_maxIterationTime;
    m_mutexIterationtime.unlock();

};

void SimulationManagerBase::setIterationTime(double averageIterationTime, double maxIterationTime) {
    m_mutexIterationtime.lock();
    m_averageIterationTime = averageIterationTime;
    m_maxIterationTime = maxIterationTime;
    m_mutexIterationtime.unlock();
};

void SimulationManagerBase::addToTimeScale(double step) {

    m_mutexTimelineSimulation.lock();
    // Reset the Timer
    m_pTimelineSimulation->stop();
    m_lastTime = ((double)m_pTimelineSimulation->elapsed().wall) * 1e-9 * m_timeScale + m_lastTime;
    m_pTimelineSimulation->start();


    //Set mGlobalTimeFactor
    if(step>0) {
        if(m_timeScaleListIdx+1 < m_timeScaleList.size()) {
            m_timeScaleListIdx++;
        }
    } else {
        if(m_timeScaleListIdx-1 >= 0) {
            m_timeScaleListIdx--;
        }
    }

    m_timeScale = m_timeScaleList[m_timeScaleListIdx];
//    LOG(m_pSimluationLog, "---> TimeScale set to " << m_timeScale <<std::endl;);

    m_mutexTimelineSimulation.unlock();

}

void SimulationManagerBase::togglePauseSimulation() {

    m_mutexTimelineSimulation.lock();
    if(m_bPauseEnabled==false){

        m_bPauseEnabled = true;
        std::cout << "Paused Simulation!"<< std::endl;


        // Reset the Timer
        m_timeScale = m_timeScaleList[0];
        m_pTimelineSimulation->stop();
//      m_timeScale = 0;
    }
    else{
        m_bPauseEnabled = false;
        m_timeScale = m_timeScaleList[m_timeScaleListIdx];
        m_lastTime = ((double)m_pTimelineSimulation->elapsed().wall) * 1e-9 * m_timeScale + m_lastTime;
        m_pTimelineSimulation->start();

    }

    m_mutexTimelineSimulation.unlock();
}

bool SimulationManagerBase::isSimulationPaused() {
    return m_bPauseEnabled;
}

double SimulationManagerBase::getTimeScale() {
    boost::mutex::scoped_lock l(m_mutexTimelineSimulation);
    return m_timeScale;
}

void SimulationManagerBase::setNumberOfContacts( unsigned int nContacts ) {
    boost::mutex::scoped_lock l(m_nCurrentContacts_mutex);
    m_nCurrentContacts = nContacts;
}
unsigned int SimulationManagerBase::getNumberOfContacts( ) {
    boost::mutex::scoped_lock l(m_nCurrentContacts_mutex);
    return m_nCurrentContacts;
}

