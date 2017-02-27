// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_TimeStepperSettings_hpp
#define GRSF_dynamics_general_TimeStepperSettings_hpp

#include "GRSF/common/TypeDefs.hpp"

#include <boost/filesystem.hpp>

/**
* @ingroup DynamicsGeneral
* @brief The time stepper settings.
*/
struct TimeStepperSettings
{
    DEFINE_LAYOUT_CONFIG_TYPES
    TimeStepperSettings()
    {
        // standart values
        m_deltaT                = 0.001;
        m_startTime             = 0;
        m_endTime               = 10;
        m_simStateReferenceFile = boost::filesystem::path();   ///< The reference file which is used to simulate, either
                                                               /// continue or use_states!
        m_simDataReferenceFile   = boost::filesystem::path();  // No implemented yet
        m_eSimulateFromReference = NONE;
    }

    PREC m_deltaT;
    PREC m_startTime;
    PREC m_endTime;

    enum
    {
        NONE,
        CONTINUE,
        USE_STATES
    } m_eSimulateFromReference;
    boost::filesystem::path m_simStateReferenceFile;
    boost::filesystem::path m_simDataReferenceFile;
};

#endif
