// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_TimeStepperBase_hpp
#define GRSF_dynamics_general_TimeStepperBase_hpp

// Includes =================================
#include <cmath>
#include <fstream>

#include <boost/filesystem.hpp>

#include "GRSF/common/Asserts.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/common/CPUTimer.hpp"

#include "GRSF/common/BinaryFile.hpp"
#include "GRSF/common/SimpleLogger.hpp"
#include "GRSF/dynamics/general/MultiBodySimFile.hpp"

//===========================================

/**
* @ingroup DynamicsGeneral
* @brief The Moreau time stepper base class.
*/
template <typename TDerived, typename TTraits>
class TimeStepperBase
{
    public:
    DEFINE_LAYOUT_CONFIG_TYPES

    using Derived                 = TDerived;
    using Traits                  = TTraits;
    using RigidBodyType           = typename Traits::RigidBodyType;
    using CollisionSolverType     = typename Traits::CollisionSolverType;
    using InclusionSolverType     = typename Traits::InclusionSolverType;
    using DynamicsSystemType      = typename Traits::DynamicsSystemType;
    using TimeStepperSettingsType = typename Traits::TimeStepperSettingsType;

    TimeStepperBase(std::shared_ptr<DynamicsSystemType> pDynSys);
    ~TimeStepperBase();

    // The Core Objects ==================================
    std::shared_ptr<CollisionSolverType> m_pCollisionSolver;
    std::shared_ptr<InclusionSolverType> m_pInclusionSolver;
    std::shared_ptr<DynamicsSystemType>  m_pDynSys;
    // ===================================================

    inline void initLogs(const boost::filesystem::path& folder_path, const boost::filesystem::path& simDataFile = "");
    inline void closeAllFiles();
    inline void reset();

    inline PREC getTimeCurrent()
    {
        return m_currentSimulationTime;
    }

    inline unsigned int getIterationCount()
    {
        return m_IterationCounter;
    }

    // Solver Parameters
    TimeStepperSettingsType m_settings;

    double m_AvgTimeForOneIteration;
    double m_MaxTimeForOneIteration;

    inline bool finished()
    {
        return m_bFinished;
    }

    protected:
    void initLogs_impl(const boost::filesystem::path& folder_path, const boost::filesystem::path& simDataFile = "");
    void closeAllFiles_impl();
    void reset_impl();

    void doFirstHalfTimeStep(PREC ts, PREC timestep);
    void doSecondHalfTimeStep(PREC te, PREC timestep);

    void resetForNextIteration();

    // This is a minimal update of F, no checking if constant values are correct
    void updateFMatrix(const Quaternion& q, Matrix43& F_i);

    PREC m_currentSimulationTime;
    PREC m_startSimulationTime;

    int  m_IterationCounter;
    bool m_bIterationFinished;
    bool m_bFinished;

    // Timer for the Performance
    CPUTimer m_PerformanceTimer;
    double   m_startTime, m_endTime, m_startTimeCollisionSolver, m_endTimeCollisionSolver, m_startTimeInclusionSolver,
        m_endTimeInclusionSolver;

    // Collision Data file
    BinaryFile m_CollisionDataFile;

    // System Data file
    std::ofstream m_SystemDataFile;

    // Reference Sim File for Simulation
    MultiBodySimFile m_ReferenceSimFile;

    // General Log file
    Logging::Log *m_pSolverLog, *m_pSimulationLog;

    // Logs
    boost::filesystem::path m_SimFolderPath;
    boost::filesystem::path m_SystemDataFilePath;
    boost::filesystem::path m_CollisionDataFilePath;
    boost::filesystem::path m_SolverLogFilePath;
};

#include "GRSF/dynamics/general/TimeStepperBase.icc"

#endif
