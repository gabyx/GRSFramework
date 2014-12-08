﻿/*
*  MoreauTimeStepper.h
*  DynamicSystem
*
*  Created by Gabriel Nützi on 21.03.10.
*  Copyright 2010 ETH. All rights reserved.
*
*/
#ifndef GMSF_Dynamics_General_MoreauTimeStepper_hpp
#define GMSF_Dynamics_General_MoreauTimeStepper_hpp

// Includes =================================
#include <fstream>
#include <cmath>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "GMSF/Common/TypeDefs.hpp"
#include "GMSF/Common/LogDefines.hpp"
#include "GMSF/Common/AssertionDebug.hpp"

#include "GMSF/Common/CPUTimer.hpp"

#include DynamicsSystem_INCLUDE_FILE
#include CollisionSolver_INCLUDE_FILE
#include InclusionSolver_INCLUDE_FILE
#include StatePool_INCLUDE_FILE

#include "GMSF/Dynamics/Buffers/DynamicsState.hpp"
#include "GMSF/Dynamics/Buffers/FrontBackBuffer.hpp"
#include "GMSF/Common/BinaryFile.hpp"
#include "GMSF/Dynamics/General/MultiBodySimFile.hpp"
#include "GMSF/Common/SimpleLogger.hpp"

#include "GMSF/Dynamics/General/TimeStepperSettings.hpp"


//===========================================



/**
* @ingroup DynamicsGeneral
* @brief The Moreau time stepper.
*/

class MoreauTimeStepper {
public:

    DEFINE_TIMESTEPPER_CONFIG_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    MoreauTimeStepper(std::shared_ptr<DynamicsSystemType> pDynSys,  std::shared_ptr<StatePoolType>	pSysState);
    ~MoreauTimeStepper();

    // The Core Objects ==================================
    std::shared_ptr<CollisionSolverType>  m_pCollisionSolver;
    std::shared_ptr<InclusionSolverType>  m_pInclusionSolver;
    std::shared_ptr<DynamicsSystemType>	  m_pDynSys;
    std::shared_ptr<StatePoolType>		  m_pStatePool;
    // ===================================================

    void initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile="");
    void closeAllFiles();

    void reset();
    void doOneIteration();

    PREC getTimeCurrent();
    unsigned int getIterationCount();

    // Solver Parameters
    TimeStepperSettings m_settings;

    //Accessed only by Simulation manager, after doOneIteration();
    const DynamicsState * getBackStateBuffer();
    const DynamicsState * getFrontStateBuffer();


    //Performance Time of one Iteration (averaged)
    double m_AvgTimeForOneIteration;
    double m_MaxTimeForOneIteration;

    inline bool finished() {
        return m_bFinished;
    }

    inline void writeHeaderToSystemDataFile();
    inline void writeIterationToSystemDataFile(double globalTime);


    inline void writeIterationToCollisionDataFile();

protected:

    PREC m_currentSimulationTime;
    PREC m_startSimulationTime;

//    const unsigned int m_nSimBodies; // These are the dimensions for one Obj

    int m_IterationCounter;
    bool m_bIterationFinished;

    bool m_bFinished;

    // Timer for the Performance
    CPUTimer m_PerformanceTimer;
    double m_startTime, m_endTime, m_startTimeCollisionSolver, m_endTimeCollisionSolver, m_startTimeInclusionSolver, m_endTimeInclusionSolver;

    // Collision Data file
    BinaryFile m_CollisionDataFile;

    // System Data file
    boost::filesystem::ofstream m_SystemDataFile;

    // Reference Sim File for Simulation
    MultiBodySimFile m_ReferenceSimFile;

    // General Log file
    Logging::Log *m_pSolverLog, *m_pSimulationLog;

    using FrontBackBufferType = typename StatePoolType::FrontBackBufferType;
    FrontBackBufferType m_StateBuffers;
    //Solver state pool front and back buffer
    void swapStateBuffers();

    //DynamicsState m_state_m;  // middle state of iteration

    // Logs
    boost::filesystem::path m_SimFolderPath;
    boost::filesystem::path m_SystemDataFilePath;
    boost::filesystem::path m_CollisionDataFilePath;
    boost::filesystem::path m_SolverLogFilePath;
};

void MoreauTimeStepper::writeIterationToSystemDataFile(double globalTime) {
#if OUTPUT_SIMDATA_FILE == 1

    m_SystemDataFile << std::fixed
    << globalTime << "\t"
    << m_currentSimulationTime <<"\t"
    << (m_endTime-m_startTime) <<"\t"
    << (m_endTimeCollisionSolver-m_startTimeCollisionSolver) <<"\t"
    << (m_endTimeInclusionSolver-m_startTimeInclusionSolver) <<"\t"
    << m_AvgTimeForOneIteration <<"\t"
    << m_pCollisionSolver->getIterationStats() << "\t"
    << m_pInclusionSolver->getIterationStats() << std::endl;
#endif
}

void MoreauTimeStepper::writeHeaderToSystemDataFile() {
#if OUTPUT_SIMDATA_FILE == 1
    m_SystemDataFile <<"# "
    << "GlobalTime [s]" << "\t"
    << "SimulationTime [s]" <<"\t"
    << "TimeStepTime [s]" <<"\t"
    << "CollisionTime [s]" <<"\t"
    << "InclusionTime [s]" <<"\t"
    << "AvgIterTime [s]" <<"\t"
    << m_pCollisionSolver->getStatsHeader() << "\t"
    << m_pInclusionSolver->getStatsHeader() << std::endl;
#endif
}

void MoreauTimeStepper::writeIterationToCollisionDataFile() {
#if OUTPUT_COLLISIONDATA_FILE == 1

    double averageOverlap = 0;

    unsigned int nContacts = m_pCollisionSolver->m_collisionSet.size();
    m_CollisionDataFile << m_currentSimulationTime; // Write Time
    m_CollisionDataFile << nContacts; // Write number of Contacts
    for(unsigned int i=0; i<nContacts; i++) {
        averageOverlap += m_pCollisionSolver->m_collisionSet[i].m_overlap;
        m_CollisionDataFile<< (double)m_pCollisionSolver->m_collisionSet[i].m_overlap;
    }
    averageOverlap /= nContacts;
    m_CollisionDataFile<< (double)averageOverlap;
#endif
}

#endif
