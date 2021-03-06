// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/states/simulationManager/PlaybackManagerBase.hpp"

#include <fstream>

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/singeltons/contexts/InputContext.hpp"

PlaybackManagerBase::PlaybackManagerBase()
    : m_bSimThreadRunning(false)
    , m_eSimThreadRunning(NONE)
    , m_averageIterationTime(0.0)
    , m_maxIterationTime(0.0)
    , m_nCurrentContacts(0)
    , m_bThreadToBeStopped(false)
    ,

    m_barrier_start(2)
{
    m_lastTime            = 0;
    m_pTimelineSimulation = std::shared_ptr<CPUTimer>(new CPUTimer);
    m_pTimelineSimulation->start();

    // Setup timeScale List;
    m_timeScaleList.push_back(0);
    for (int i = 1; i < 10; i++)
    {
        m_timeScaleList.push_back(i * 0.01);
    }
    for (int i = 1; i <= 30; i++)
    {
        m_timeScaleList.push_back(i * 0.1);
    }
    m_timeScaleListIdx = 19;
    m_timeScale        = m_timeScaleList[m_timeScaleListIdx];

    m_bPauseEnabled = false;
};

PlaybackManagerBase::~PlaybackManagerBase(){DESTRUCTOR_MESSAGE};

bool PlaybackManagerBase::isSimThreadRunning()
{
    boost::mutex::scoped_lock l(m_bSimThreadRunning_mutex);
    return m_bSimThreadRunning;
}

void PlaybackManagerBase::setSimThreadRunning(bool value)
{
    boost::mutex::scoped_lock l(m_bSimThreadRunning_mutex);
    m_bSimThreadRunning = value;
}
bool PlaybackManagerBase::isSimThreadToBeStopped()
{
    boost::mutex::scoped_lock l(m_bThreadToBeStopped_mutex);
    return m_bThreadToBeStopped;
};
void PlaybackManagerBase::setThreadToBeStopped(bool stop)
{
    boost::mutex::scoped_lock l(m_bThreadToBeStopped_mutex);
    m_bThreadToBeStopped = stop;
};

double PlaybackManagerBase::getTimelineSimulation()
{
    double x;
    m_mutexTimelineSimulation.lock();
    x = m_pTimelineSimulation->elapsedSec() * m_timeScale + m_lastTime;
    m_mutexTimelineSimulation.unlock();
    return x;
};

void PlaybackManagerBase::resetTimelineSimulation()
{
    m_mutexTimelineSimulation.lock();
    m_pTimelineSimulation->start();
    m_lastTime = 0;
    m_mutexTimelineSimulation.unlock();
};

void PlaybackManagerBase::getIterationTime(double& averageIterationTime, double& maxIterationTime)
{
    m_mutexIterationtime.lock();
    averageIterationTime = m_averageIterationTime;
    maxIterationTime     = m_maxIterationTime;
    m_mutexIterationtime.unlock();
};

void PlaybackManagerBase::setIterationTime(double averageIterationTime, double maxIterationTime)
{
    m_mutexIterationtime.lock();
    m_averageIterationTime = averageIterationTime;
    m_maxIterationTime     = maxIterationTime;
    m_mutexIterationtime.unlock();
};

void PlaybackManagerBase::addToTimeScale(double step)
{
    m_mutexTimelineSimulation.lock();
    // Reset the Timer
    m_lastTime = m_pTimelineSimulation->elapsedSec() * m_timeScale + m_lastTime;
    m_pTimelineSimulation->start();

    // Set mGlobalTimeFactor
    if (step > 0)
    {
        if (m_timeScaleListIdx + 1 < m_timeScaleList.size())
        {
            m_timeScaleListIdx++;
        }
    }
    else
    {
        if (m_timeScaleListIdx - 1 >= 0)
        {
            m_timeScaleListIdx--;
        }
    }

    m_timeScale = m_timeScaleList[m_timeScaleListIdx];

    m_mutexTimelineSimulation.unlock();
}

void PlaybackManagerBase::togglePauseSimulation()
{
    m_mutexTimelineSimulation.lock();
    if (m_bPauseEnabled == false)
    {
        m_bPauseEnabled = true;

        // Reset the Timer
        // m_pTimelineSimulation->stop();

        //        m_timeScale = 0;
    }
    else
    {
        m_bPauseEnabled = false;
        m_lastTime      = m_pTimelineSimulation->elapsedSec() * m_timeScale + m_lastTime;
        m_pTimelineSimulation->start();
    }

    m_mutexTimelineSimulation.unlock();
}

bool PlaybackManagerBase::isSimulationPaused()
{
    return m_bPauseEnabled;
}

double PlaybackManagerBase::getTimeScale()
{
    boost::mutex::scoped_lock l(m_mutexTimelineSimulation);
    return m_timeScale;
}

void PlaybackManagerBase::setNumberOfContacts(unsigned int& nContacts)
{
    boost::mutex::scoped_lock l(m_nCurrentContacts_mutex);
    m_nCurrentContacts = nContacts;
}
unsigned int PlaybackManagerBase::getNumberOfContacts()
{
    boost::mutex::scoped_lock l(m_nCurrentContacts_mutex);
    return m_nCurrentContacts;
}
