/*
*  MoreauTimeStepper.h
*  DynamicSystem
*
*  Created by Gabriel Nützi on 21.03.10.
*  Copyright 2010 ETH. All rights reserved.
*
*/
#ifndef GRSF_dynamics_general_MoreauTimeStepper_hpp
#define GRSF_dynamics_general_MoreauTimeStepper_hpp

// Includes =================================
#include <fstream>
#include <cmath>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/AssertionDebug.hpp"

#include "GRSF/dynamics/general/TimeStepperSettings.hpp"
#include "GRSF/dynamics/general/TimeStepperBase.hpp"
#include StatePool_INCLUDE_FILE

#include "GRSF/dynamics/buffers/DynamicsState.hpp"
#include "GRSF/dynamics/buffers/FrontBackBuffer.hpp"


//===========================================

/**
* @ingroup DynamicsGeneral
* @brief The Moreau time stepper.
*/

struct MoreauTimeStepperTraits{
    DEFINE_TIMESTEPPER_CONFIG_TYPES;
    using TimeStepperSettingsType = TimeStepperSettings;
};

class MoreauTimeStepper: public TimeStepperBase<MoreauTimeStepper,MoreauTimeStepperTraits> {
public:

    DEFINE_TIMESTEPPER_CONFIG_TYPES

    MoreauTimeStepper(std::shared_ptr<DynamicsSystemType> pDynSys,  std::shared_ptr<StatePoolType>	pSysState);
    ~MoreauTimeStepper();

    // The Core Objects ==================================
    std::shared_ptr<StatePoolType>		  m_pStatePool;
    // ===================================================

    void initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile="");
    void closeAllFiles();

    void reset();
    void doTimeStep();

    //Accessed only by Simulation manager, after doOneIteration();
    const DynamicsState * getBackStateBuffer();
    const DynamicsState * getFrontStateBuffer();


    inline void writeHeaderToSystemDataFile();
    inline void writeIterationToSystemDataFile(double globalTime);
    inline void writeIterationToCollisionDataFile();

protected:

    inline  void afterFirstTimeStep() {};
    inline  void afterSecondTimeStep() {};
    inline  void doInputTimeStep(PREC T) {};


    using FrontBackBufferType = typename StatePoolType::FrontBackBufferType;
    FrontBackBufferType m_StateBuffers;
    //Solver state pool front and back buffer
    void swapStateBuffers();

};

void MoreauTimeStepper::writeIterationToSystemDataFile(double globalTime) {
#ifdef OUTPUT_SIMDATA_FILE

    m_SystemDataFile << std::fixed
    << globalTime << "\t"
    << m_startSimulationTime <<"\t"
    << (m_endTime-m_startTime) <<"\t"
    << (m_endTimeCollisionSolver-m_startTimeCollisionSolver) <<"\t"
    << (m_endTimeInclusionSolver-m_startTimeInclusionSolver) <<"\t"
    << m_AvgTimeForOneIteration <<"\t"
    << m_pDynSys->m_simBodies.size() <<"\t"
    << m_pCollisionSolver->getIterationStats() << "\t"
    << m_pInclusionSolver->getIterationStats() << std::endl;
#endif
}

void MoreauTimeStepper::writeHeaderToSystemDataFile() {
#ifdef OUTPUT_SIMDATA_FILE
    m_SystemDataFile <<"# "
    << "GlobalTime [s]" << "\t"
    << "SimulationTime [s]" <<"\t"
    << "TimeStepTime [s]" <<"\t"
    << "CollisionTime [s]" <<"\t"
    << "InclusionTime [s]" <<"\t"
    << "AvgIterTime [s]" <<"\t"
    << "nSimBodies" <<"\t"
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
