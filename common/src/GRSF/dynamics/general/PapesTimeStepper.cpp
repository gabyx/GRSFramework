// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/dynamics/general/PapesTimeStepper.hpp"

#include "GRSF/common/LogDefines.hpp"


PapesTimeStepper::PapesTimeStepper(std::shared_ptr<DynamicsSystemType> pDynSys,
                                     std::shared_ptr<StatePoolType>	pSysState)
    :TimeStepperBase<PapesTimeStepper,MoreauTimeStepperTraits>(pDynSys)
{
    m_pCollisionSolver = std::shared_ptr<CollisionSolverType>(new CollisionSolverType(m_pDynSys));
    m_pInclusionSolver = std::shared_ptr<InclusionSolverType>(new InclusionSolverType(m_pCollisionSolver,m_pDynSys));
    // Instantiate all Core Objects
    m_pStatePool = pSysState;
};




PapesTimeStepper::~PapesTimeStepper() {
    DECONSTRUCTOR_MESSAGE
};


void PapesTimeStepper::closeAllFiles() {
    TimeStepperBase::closeAllFiles_impl();
}


void PapesTimeStepper::initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile  ) {
    TimeStepperBase::initLogs_impl(folder_path,simDataFile);
    writeHeaderToSystemDataFile();
}



void PapesTimeStepper::reset() {


    m_pSimulationLog->logMessage("---> Reset StatePool...");
    m_pStatePool->resetStatePool(m_pDynSys->m_bodiesInitStates); // Sets initial values to front and back;
    m_StateBuffers = m_pStatePool->getFrontBackBuffer();

     if(m_settings.m_eSimulateFromReference != TimeStepperSettings::NONE) {

        if(!m_ReferenceSimFile.openRead(m_settings.m_simStateReferenceFile,true,m_pDynSys->m_simBodies.size())) {
            std::stringstream error;
            error << "Could not open file: " << m_settings.m_simStateReferenceFile.string()<<std::endl;
            error << "File errors: " <<std::endl<< m_ReferenceSimFile.getErrorString();
            m_pSolverLog->logMessage( error.str());
            ERRORMSG(error);
        }
        LOG(m_pSimulationLog,"---> Opened Reference SimFile: m_settings.m_simStateReferenceFile"<<std::endl);

        if(m_settings.m_eSimulateFromReference != TimeStepperSettings::CONTINUE) {
            //Inject the end state into the front buffer
            m_ReferenceSimFile.getEndState(*m_StateBuffers.m_pFront);
            LOG(m_pSimulationLog,"---> Injected first state of Reference SimFile into StateBuffer"<<std::endl);
            ERRORMSG("apply to bodies not implemented")
            //m_pDynSys->applyDynamicsStateToSimBodies(*m_StateBuffers.m_pFront);
        }

    }else{
        // Apply all init states to the bodies!
        m_pSimulationLog->logMessage("---> Initialize Bodies...");
        m_pDynSys->applyInitStatesToBodies();
    }

    //m_ReferenceSimFile.writeOutAllStateTimes();

    TimeStepperBase::reset_impl();


};



const DynamicsState *
PapesTimeStepper::getBackStateBuffer() {
    return m_StateBuffers.m_pBack;
}



const DynamicsState *
PapesTimeStepper::getFrontStateBuffer() {
    return m_StateBuffers.m_pFront;
}


void PapesTimeStepper::doOneIteration() {
    static int iterations=0; // Average is reset after 1000 Iterations



    LOGSLLEVEL1(m_pSolverLog, "---> Do one time-step =================================" <<std::endl <<
                "---> t_S: " << m_currentSimulationTime << std::endl;);


    m_bIterationFinished = false;

    iterations++;
    m_IterationCounter++;

    m_startTime = m_PerformanceTimer.elapsedSec();



    //Force switch
    //boost::thread::yield();

    // If we should load the state from a reference file! Do this here!
    if(m_settings.m_eSimulateFromReference == TimeStepperSettings::USE_STATES && !m_bFinished) {
        m_ReferenceSimFile >> m_StateBuffers.m_pFront;
        ERRORMSG("Not implemented")
        //m_pDynSys->applyDynamicsStateToSimBodies(*m_StateBuffers.m_pFront);
    }


    // Swap front and back buffers!
    swapStateBuffers();

#if SOLVERLOG_LOGLEVEL == 1
      unsigned int when = (m_settings.m_endTime-m_settings.m_startTime)/m_settings.m_deltaT / 10.0;
      if(when<=0){when=1;}
      if(m_IterationCounter % when == 0){
            LOG(m_pSolverLog,"--->  m_tB: " << m_currentSimulationTime<<std::endl; );
      }
#endif


      LOGSLLEVEL3(m_pSolverLog,"---> m_tB: " << m_currentSimulationTime <<std::endl; );



    //Calculate Midpoint Rule ============================================================
    // Time Step to ==================================================================

    m_startSimulationTime = m_currentSimulationTime;
    doFirstHalfTimeStep(m_startSimulationTime, m_settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    doInputTimeStep(m_settings.m_deltaT/2.0);
    // Custom Calculations after first timestep
    afterFirstTimeStep();

    m_currentSimulationTime = m_startSimulationTime + m_settings.m_deltaT/2.0;
    // ====================================================================================

    m_pInclusionSolver->resetForNextTimestep(); // Clears the contact graph!

    // Solve Collision
    m_startTimeCollisionSolver = m_PerformanceTimer.elapsedSec();
    m_pCollisionSolver->solveCollision();
    m_endTimeCollisionSolver =   m_PerformanceTimer.elapsedSec();

    //Solve Contact Problem
    //boost::thread::yield();
    m_startTimeInclusionSolver = m_PerformanceTimer.elapsedSec();
    m_pInclusionSolver->solveInclusionProblem();
    m_endTimeInclusionSolver = m_PerformanceTimer.elapsedSec();

    //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    //boost::thread::yield();


    // ===================================================================================

    // Middle Time Step ==================================================================
    m_currentSimulationTime = m_startSimulationTime + m_settings.m_deltaT ;

    doSecondHalfTimeStep(m_currentSimulationTime, m_settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    doInputTimeStep(m_settings.m_deltaT/2.0);
    // Custom Calculations after second timestep
    afterSecondTimeStep();
    // ====================================================================================


    // Apply all rigid body local states to the Front buffer and set the time!
    m_StateBuffers.m_pFront->m_t= m_currentSimulationTime;
    m_pDynSys->applySimBodiesToDynamicsState(*m_StateBuffers.m_pFront);



    LOGSLLEVEL3(m_pSolverLog,"---> m_tE: " << m_currentSimulationTime <<std::endl );


    //Force switch
    //boost::thread::yield();

    m_endTime = m_PerformanceTimer.elapsedSec();


    // Measure Time again
    if (m_IterationCounter%100==0) {
        m_AvgTimeForOneIteration=0;
        iterations = 1;
    }
    m_AvgTimeForOneIteration =  ( (m_endTime-m_startTime) + m_AvgTimeForOneIteration*(iterations-1)) / iterations;
    if (m_AvgTimeForOneIteration > m_MaxTimeForOneIteration) {
        m_MaxTimeForOneIteration = m_AvgTimeForOneIteration;
    }


    LOGSLLEVEL1( m_pSolverLog,  "---> Iteration Time: "<<std::setprecision(5)<<(m_endTime-m_startTime)<<std::endl
     <<  "---> End time-step ====================================" <<std::endl<<std::endl; );


    // Check if we can finish the timestepping!
    if(m_settings.m_eSimulateFromReference == TimeStepperSettings::USE_STATES ) {
        m_bFinished =  !m_ReferenceSimFile.isGood();
    } else {
        m_bFinished =  m_StateBuffers.m_pFront->m_t >= m_settings.m_endTime;
    }

    m_bIterationFinished = true;
}


void PapesTimeStepper::swapStateBuffers() {
    m_StateBuffers = m_pStatePool->swapFrontBackBuffer();
}




