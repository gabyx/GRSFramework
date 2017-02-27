// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_buffers_RecorderSettings_hpp
#define GRSF_dynamics_buffers_RecorderSettings_hpp

#include "GRSF/common/TypeDefs.hpp"

/**
* @ingroup DynamicsGeneral
* @brief The recorder settings.
*/

class RecorderSettings
{
    public:
    DEFINE_LAYOUT_CONFIG_TYPES

    RecorderSettings()
    {
        m_eMode                = RECORD_EVERY_STEP;
        m_recordEveryXTimestep = 1;
    }
    enum RecorderMode
    {
        RECORD_EVERY_STEP,
        RECORD_EVERY_X_STEP,  ///< Set the number of how many steps to skip
        RECORD_NOTHING
    };

    bool outputCheck(unsigned int iterationCount)
    {
        if (m_eMode == RECORD_EVERY_X_STEP)
        {
            // std:: cout << iterationCount <<","<<m_recordEveryXTimestep<<std::endl;
            if (iterationCount % m_recordEveryXTimestep == 0)
            {
                return true;
            }
        }
        else if (m_eMode == RECORD_EVERY_STEP)
        {
            return true;
        }
        return false;
    }

    void setMode(RecorderMode mode)
    {
        m_eMode = mode;
        if (m_eMode == RECORD_EVERY_STEP)
        {
            m_recordEveryXTimestep = 1;
        }
    }

    RecorderMode& getMode()
    {
        return m_eMode;
    }

    void setEveryXTimestep(PREC everyXStep)
    {
        m_recordEveryXTimestep = everyXStep;
    }

    void setEveryXTimestep(PREC stepsPerSecond, PREC timeStep)
    {
        int everyXStep = std::floor(1.0 / (stepsPerSecond * timeStep));
        if (everyXStep < 1)
        {
            m_recordEveryXTimestep = 1;
        }
        else
        {
            m_recordEveryXTimestep = everyXStep;
        }
    }

    unsigned int getEveryXTimestep()
    {
        return m_recordEveryXTimestep;
    }

    private:
    RecorderMode m_eMode;
    unsigned int m_recordEveryXTimestep;
};

#endif
