#ifndef PlaybackManager_hpp
#define PlaybackManager_hpp


#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <OGRE/Ogre.h>
#include <OIS/OISEvents.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISMouse.h>

#include "AssertionDebug.hpp"

#include <boost/timer/timer.hpp>

class PlaybackManagerBase {
public:
    PlaybackManagerBase();

    ~PlaybackManagerBase();


    double getTimelineSimulation();
    void resetTimelineSimulation();

    enum ThreadRunning {NONE, REALTIME} m_eSimThreadRunning;

    double getTimeScale();
    bool isSimulationPaused();
    // Information to visualize by GUI
    void getIterationTime(double & averageIterationTime, double & maxIterationTime);
    void setIterationTime(double averageIterationTime,   double maxIterationTime);
    unsigned int getNumberOfContacts();
    void setNumberOfContacts(unsigned int & nContacts);

    virtual bool setup() = 0;
    virtual void updateScene(double timeSinceLastFrame) = 0;
    virtual double getSimulationTime() {
        return 0;
    };

    boost::thread*	m_pThread;
    virtual void startPlaybackThread()=0;
    virtual void stopPlaybackThread(bool force_stop)=0;


protected:

    virtual void initBeforeThreads() {};

    virtual void initSimThread() {};
    virtual void threadRunSimulation() {
        while(!isSimThreadToBeStopped()) {
            ASSERTMSG(false, "There is no Simulation thread implemented");
            std::cout << " Sim Thread running, but no implementation"<<std::endl;
        }
    };



    boost::mutex m_mutexIterationtime;
    double m_averageIterationTime;
    double m_maxIterationTime;

    boost::mutex m_nCurrentContacts_mutex;
    unsigned int m_nCurrentContacts;


    void setThreadToBeStopped(bool stop);   /**	\brief Sets the variable in the vis thread to make the sim thread cancel. */
    boost::mutex m_bThreadToBeStopped_mutex;
    bool m_bThreadToBeStopped;

    bool isSimThreadRunning();              /**	\brief Checks in the vis thread if the thread is still running. */
    boost::mutex m_bSimThreadRunning_mutex;
    bool m_bSimThreadRunning;

    // Used by sim thread
    bool isSimThreadToBeStopped();          /**	\brief Checks if the sim thread should be stopped. */
    void setSimThreadRunning(bool value);   /**	\brief Sets the variable which is used in the vis thread to check if the thread is still alive. */

    boost::mutex		m_mutexTimelineSimulation;
    boost::shared_ptr<boost::timer::cpu_timer>		m_pTimelineSimulation;
    double m_lastTime; // This is used to stop the timer, set m_lastTime, increase/decrease timeScale, startTimer again

    double	m_timeScale;
    std::vector<double> m_timeScaleList;
    int m_timeScaleListIdx;
    void addToTimeScale(double step);

    void togglePauseSimulation();
    bool m_bPauseEnabled;

    boost::barrier m_barrier_start; // waits for 2 threads! // if needed,

};
//=========================================================

#endif
