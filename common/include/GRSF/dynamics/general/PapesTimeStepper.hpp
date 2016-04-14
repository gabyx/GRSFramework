// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_PapesTimeStepper_hpp
#define GRSF_dynamics_general_PapesTimeStepper_hpp

// Includes =================================
#include <fstream>
#include <cmath>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/Asserts.hpp"

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

struct PapesTimeStepperTraits{
    DEFINE_TIMESTEPPER_CONFIG_TYPES;
    using TimeStepperSettingsType = TimeStepperSettings;
};

class PapesTimeStepper: public TimeStepperBase<PapesTimeStepper,MoreauTimeStepperTraits> {
public:

    DEFINE_TIMESTEPPER_CONFIG_TYPES

    PapesTimeStepper(std::shared_ptr<DynamicsSystemType> pDynSys,  std::shared_ptr<StatePoolType>	pSysState);
    ~PapesTimeStepper();

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

void PapesTimeStepper::writeIterationToSystemDataFile(double globalTime) {
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

void PapesTimeStepper::writeHeaderToSystemDataFile() {
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

void PapesTimeStepper::writeIterationToCollisionDataFile() {
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
