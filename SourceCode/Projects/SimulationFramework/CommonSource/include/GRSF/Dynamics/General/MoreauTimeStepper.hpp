/*
*  MoreauTimeStepper.h
*  DynamicSystem
*
*  Created by Gabriel Nützi on 21.03.10.
*  Copyright 2010 ETH. All rights reserved.
*
*/
#ifndef GRSF_Dynamics_General_MoreauTimeStepper_hpp
#define GRSF_Dynamics_General_MoreauTimeStepper_hpp

// Includes =================================
#include <fstream>
#include <cmath>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/Dynamics/General/TimeStepperSettings.hpp"
#include "GRSF/Dynamics/General/TimeStepperBase.hpp"
#include StatePool_INCLUDE_FILE

#include "GRSF/Dynamics/Buffers/DynamicsState.hpp"
#include "GRSF/Dynamics/Buffers/FrontBackBuffer.hpp"


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
    void doOneIteration();

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
