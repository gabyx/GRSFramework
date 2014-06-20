/*
*  MoreauTimeStepper.h
*  DynamicSystem
*
*  Created by Gabriel Nützi on 21.03.10.
*  Copyright 2010 ETH. All rights reserved.
*
*/
#ifndef MoreauTimeStepper_hpp
#define MoreauTimeStepper_hpp

// Includes =================================
#include <fstream>
#include <cmath>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

#include "CPUTimer.hpp"

#include DynamicsSystem_INCLUDE_FILE
#include CollisionSolver_INCLUDE_FILE
#include InclusionSolver_INCLUDE_FILE

#include "DynamicsState.hpp"
#include "FrontBackBuffer.hpp"
#include "BinaryFile.hpp"
#include "MultiBodySimFile.hpp"
#include "SimpleLogger.hpp"

#include "TimeStepperSettings.hpp"


//===========================================



/**
* @ingroup DynamicsGeneral
* @brief The Moreau time stepper.
*/

class MoreauTimeStepper {
public:

    DEFINE_TIMESTEPPER_CONFIG_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    MoreauTimeStepper(boost::shared_ptr<DynamicsSystemType> pDynSys,  boost::shared_ptr<StatePoolType>	pSysState);
    ~MoreauTimeStepper();

    // The Core Objects ==================================
    boost::shared_ptr<CollisionSolverType>  m_pCollisionSolver;
    boost::shared_ptr<InclusionSolverType>  m_pInclusionSolver;
    boost::shared_ptr<DynamicsSystemType>	  m_pDynSys;
    boost::shared_ptr<StatePoolType>		  m_pStatePool;
    // ===================================================

    void initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile="");
    void closeAllFiles();

    void reset();
    void doOneIteration();

    PREC getTimeCurrent();
    unsigned int getIterationCount();

    // Solver Parameters
    TimeStepperSettings m_Settings;

    //Accessed only by Simulation manager, after doOneIteration();
    boost::shared_ptr<const DynamicsState > getBackStateBuffer();
    boost::shared_ptr<const DynamicsState > getFrontStateBuffer();


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

    typedef FrontBackBuffer<DynamicsState, FrontBackBufferPtrType::SharedPtr, FrontBackBufferMode::BackConst> FrontBackBufferType;
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
