// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_states_simulationManager_PlaybackManagerBase_hpp
#define GRSF_states_simulationManager_PlaybackManagerBase_hpp

#include <OGRE/Ogre.h>
#include <OIS/OISEvents.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISMouse.h>
#include <boost/thread.hpp>
#include <memory>

#include "GRSF/common/Asserts.hpp"

#include "GRSF/common/CPUTimer.hpp"

class PlaybackManagerBase
{
public:
    PlaybackManagerBase();

    ~PlaybackManagerBase();

    double getTimelineSimulation();
    void   resetTimelineSimulation();

    enum ThreadRunning
    {
        NONE,
        REALTIME
    } m_eSimThreadRunning;

    double getTimeScale();
    bool   isSimulationPaused();
    // Information to visualize by GUI
    void getIterationTime(double& averageIterationTime, double& maxIterationTime);
    void setIterationTime(double averageIterationTime, double maxIterationTime);
    unsigned int getNumberOfContacts();
    void setNumberOfContacts(unsigned int& nContacts);

    virtual bool setup()                                = 0;
    virtual void updateScene(double timeSinceLastFrame) = 0;
    virtual double getSimulationTime()
    {
        return 0;
    };

    boost::thread* m_pThread;
    virtual void   startPlaybackThread()             = 0;
    virtual void stopPlaybackThread(bool force_stop) = 0;

protected:
    virtual void initBeforeThreads(){};

    virtual void initSimThread(){};
    virtual void threadRunSimulation()
    {
        while (!isSimThreadToBeStopped())
        {
            GRSF_ASSERTMSG(false, "There is no Simulation thread implemented");
            std::cout << " Sim Thread running, but no implementation" << std::endl;
        }
    };

    boost::mutex m_mutexIterationtime;
    double       m_averageIterationTime;
    double       m_maxIterationTime;

    boost::mutex m_nCurrentContacts_mutex;
    unsigned int m_nCurrentContacts;

    void setThreadToBeStopped(
        bool stop); /**	\brief Sets the variable in the vis thread to make the sim thread cancel. */
    boost::mutex m_bThreadToBeStopped_mutex;
    bool         m_bThreadToBeStopped;

    bool         isSimThreadRunning(); /**	\brief Checks in the vis thread if the thread is still running. */
    boost::mutex m_bSimThreadRunning_mutex;
    bool         m_bSimThreadRunning;

    // Used by sim thread
    bool isSimThreadToBeStopped();        /**	\brief Checks if the sim thread should be stopped. */
    void setSimThreadRunning(bool value); /**	\brief Sets the variable which is used in the vis thread to check if
                                             the thread is still alive. */

    boost::mutex              m_mutexTimelineSimulation;
    std::shared_ptr<CPUTimer> m_pTimelineSimulation;
    double m_lastTime;  // This is used to stop the timer, set m_lastTime, increase/decrease timeScale, startTimer again

    double              m_timeScale;
    std::vector<double> m_timeScaleList;
    std::size_t         m_timeScaleListIdx;
    void addToTimeScale(double step);

    void togglePauseSimulation();
    bool m_bPauseEnabled;

    boost::barrier m_barrier_start;  // waits for 2 threads! // if needed,
};
//=========================================================

#endif
