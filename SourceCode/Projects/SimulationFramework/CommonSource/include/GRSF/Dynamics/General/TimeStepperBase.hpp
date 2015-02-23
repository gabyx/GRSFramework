/*
*  MoreauTimeStepper.h
*  DynamicSystem
*
*  Created by Gabriel Nützi on 21.03.10.
*  Copyright 2010 ETH. All rights reserved.
*
*/
#ifndef GRSF_Dynamics_General_MoreauTimeStepperBase_hpp
#define GRSF_Dynamics_General_MoreauTimeStepperBase_hpp

// Includes =================================
#include <fstream>
#include <cmath>

#include <boost/filesystem.hpp>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/Common/CPUTimer.hpp"

#include "GRSF/Common/BinaryFile.hpp"
#include "GRSF/Dynamics/General/MultiBodySimFile.hpp"
#include "GRSF/Common/SimpleLogger.hpp"

//===========================================



/**
* @ingroup DynamicsGeneral
* @brief The Moreau time stepper base class.
*/
template<typename TDerived, typename TTraits>
class TimeStepperBase {
public:

    DEFINE_LAYOUT_CONFIG_TYPES

    using Derived = TDerived;
    using Traits = TTraits;
    using RigidBodyType =  typename Traits::RigidBodyType;
    using CollisionSolverType = typename Traits::CollisionSolverType;
    using InclusionSolverType = typename Traits::InclusionSolverType;
    using DynamicsSystemType =  typename Traits::DynamicsSystemType;
    using TimeStepperSettingsType = typename Traits::TimeStepperSettingsType;


    TimeStepperBase(std::shared_ptr<DynamicsSystemType> pDynSys);
    ~TimeStepperBase();

    // The Core Objects ==================================
    std::shared_ptr<CollisionSolverType>  m_pCollisionSolver;
    std::shared_ptr<InclusionSolverType>  m_pInclusionSolver;
    std::shared_ptr<DynamicsSystemType>	  m_pDynSys;
    // ===================================================

    inline void initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile="");
    inline void closeAllFiles();
    inline void reset();

    inline PREC getTimeCurrent() {
        return m_currentSimulationTime;
    }

    inline unsigned int getIterationCount() {
        return m_IterationCounter;
    }

    // Solver Parameters
    TimeStepperSettingsType m_settings;


    double m_AvgTimeForOneIteration;
    double m_MaxTimeForOneIteration;

    inline bool finished() {
        return m_bFinished;
    }

protected:

    void initLogs_impl(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile="");
    void closeAllFiles_impl();
    void reset_impl();

    void doFirstHalfTimeStep(PREC ts, PREC timestep);
    void doSecondHalfTimeStep(PREC te, PREC timestep);


    //This is a minimal update of F, no checking if constant values are correct
    void updateFMatrix(const Quaternion & q, Matrix43 & F_i);

    PREC m_currentSimulationTime;
    PREC m_startSimulationTime;

    int m_IterationCounter;
    bool m_bIterationFinished;
    bool m_bFinished;

    // Timer for the Performance
    CPUTimer m_PerformanceTimer;
    double m_startTime, m_endTime, m_startTimeCollisionSolver, m_endTimeCollisionSolver, m_startTimeInclusionSolver, m_endTimeInclusionSolver;

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


#include "GRSF/Dynamics/General/TimeStepperBase.icc"

#endif
