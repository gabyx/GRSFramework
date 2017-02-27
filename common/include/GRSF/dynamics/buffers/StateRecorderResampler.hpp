// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_buffers_StateRecorderResampler_hpp
#define GRSF_dynamics_buffers_StateRecorderResampler_hpp

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include <vector>

#include "GRSF/singeltons/contexts/RenderContext.hpp"

#include "GRSF/dynamics/buffers/StateRecorder.hpp"

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include "GRSF/dynamics/buffers/DynamicsState.hpp"

/**
* @brief only used in sim thread to resample and drop a new sim file if the option is selected in playback! Not mutex
* locks or something else!
*/
class StateRecorderResampler : public StateRecorder
{
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateRecorderResampler(const unsigned int nSimBodies) : StateRecorder(nSimBodies)
    {
        m_pStateArray.assign(3, DynamicsState());

        reset();
        m_fps       = 25;  // Standart value
        m_startTime = 0;
        m_endTime   = 1000000;
    };

    ~StateRecorderResampler(){};

    void tryToWrite(const DynamicsState* state, bool bInterpolate)
    {
        std::stringstream logstream;
        /*
       LOG(m_pStateRecorderLog, "Current resample time: " << m_currentResampleTime << "/ "<< m_startTime << " /" <<
       m_endTime;);
       */
        if (m_currentResampleTime <= m_endTime)
        {
            *m_pNextState = *state;
            if (state->m_t >= m_currentResampleTime)
            {
                // Check to interpolate...
                double diffTime = m_pNextState->m_t - m_pPrevState->m_t;
                if (diffTime != 0 && bInterpolate)
                {
                    double factor = (m_currentResampleTime - m_pPrevState->m_t) / (diffTime);
                    Interpolate::lerp(*m_pPrevState, *m_pNextState, *m_pLerpState, factor);
                    logstream << ", Interpolate Factor: " << factor;
                    this->write(m_pLerpState);
                    logstream << "StateRecorderResampler:: Writen interpolated State: " << m_pLerpState->m_t << " / "
                              << "currentTime: " << m_currentResampleTime;
                }
                else
                {
                    this->write(m_pNextState);
                    logstream << "StateRecorderResampler:: Writen State: " << m_pNextState->m_t << " / "
                              << "currentTime: " << m_currentResampleTime;
                }

                m_currentResampleTime += 1.0 / m_fps;  // Move resample time ahead! for next drop!
                this->m_pSimulationLog->logMessage(logstream);
            }
            std::swap(m_pPrevState, m_pNextState);
        }
    }

    void setFPS(double fps)
    {
        m_fps = fps;
    }

    void setTimeRange(double startTime, double endTime)
    {
        m_startTime           = startTime;
        m_currentResampleTime = m_startTime;
        m_endTime             = endTime;
    }

    void reset()
    {
        m_nStatesDropped      = 0;
        m_currentResampleTime = m_startTime;
        m_bFirstInsert        = true;

        m_pPrevState = &m_pStateArray[0];
        m_pNextState = &m_pStateArray[1];
        m_pLerpState = &m_pStateArray[2];
    }

    void setFolderPath(boost::filesystem::path folderPath)
    {
        m_folderPath = folderPath;
    }

private:
    std::vector<DynamicsState> m_pStateArray;
    DynamicsState*             m_pPrevState;
    DynamicsState*             m_pNextState;
    DynamicsState*             m_pLerpState;
    bool                       m_bFirstInsert;

    boost::filesystem::path m_folderPath;

    double m_currentResampleTime;
    double m_fps;  //[frames/second]
    double m_startTime;
    double m_endTime;

    unsigned int m_nStatesDropped;
};

#endif
