//=========================================================

/*=========================================================
definitions of template class MoreauTimeStepper
_________________________________________________________*/
#include <iostream>

#include "GMSF/Dynamics/General/MoreauTimeStepper.hpp"

#include DynamicsSystem_INCLUDE_FILE
#include InclusionSolver_INCLUDE_FILE
#include StatePool_INCLUDE_FILE

#include "GMSF/Common/LogDefines.hpp"



MoreauTimeStepper::MoreauTimeStepper(std::shared_ptr<DynamicsSystemType> pDynSys,  std::shared_ptr<StatePoolType>	pSysState)
{

    if(Logging::LogManager::getSingleton().existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingleton().getLog("SimulationLog");
    } else {
        ERRORMSG("There is no SimulationLog in the LogManager... Did you create it?")
    }


    m_pSolverLog = nullptr;

    // Instanciate all Core Objects
    m_pStatePool = pSysState;


    m_pDynSys = pDynSys;

    m_pCollisionSolver = std::shared_ptr<CollisionSolverType>(new CollisionSolverType(m_pDynSys));

    m_pInclusionSolver = std::shared_ptr<InclusionSolverType>(new InclusionSolverType(m_pCollisionSolver,m_pDynSys));

};




MoreauTimeStepper::~MoreauTimeStepper() {
    m_CollisionDataFile.close();
    m_SystemDataFile.close();
    DECONSTRUCTOR_MESSAGE
};


void MoreauTimeStepper::closeAllFiles() {

    Logging::LogManager::getSingleton().destroyLog("SolverLog");
    m_pSolverLog = nullptr;

    m_CollisionDataFile.close();
    m_SystemDataFile.close();
}


void MoreauTimeStepper::initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile  ) {

    // Set new Simfile Path
    m_SimFolderPath = folder_path;
    std::string filename;

    // Set new SystemDataFile path (construct new if string is empty)
    if(simDataFile.empty()) {
        m_SystemDataFilePath = m_SimFolderPath;
        filename = SYSTEM_DATA_FILE_PREFIX;
        filename += ".dat";
        m_SystemDataFilePath /= filename;
    }

    // Set new CollisionDataFile path
    m_CollisionDataFilePath = m_SimFolderPath;
    filename = COLLISION_DATA_FILE_PREFIX;
    filename += ".dat";
    m_CollisionDataFilePath /= filename;

    // Set new SolverFile path
    m_SolverLogFilePath = m_SimFolderPath;
    filename = SOLVER_LOG_FILE_PREFIX;
    filename += ".log";
    m_SolverLogFilePath /= filename;


    // Set up all Logs;

    m_pSolverLog = new Logging::Log("SolverLog");
    Logging::LogManager::getSingleton().registerLog(m_pSolverLog);

#if SOLVERLOG_TOFILE == 1
    m_pSolverLog->addSink(new Logging::LogSinkFile("SolverLog-File",m_SolverLogFilePath));
#endif
#if SOLVERLOG_TOCONSOLE == 1
    m_pSolverLog->addSink(new Logging::LogSinkCout("SolverLog-Cout"));
#endif

    m_pDynSys->initializeLog(m_pSolverLog);
    m_pInclusionSolver->initializeLog(m_pSolverLog,m_SimFolderPath);
    m_pCollisionSolver->initializeLog(m_pSolverLog);

    // SystemDataFile
#if OUTPUT_SIMDATA_FILE == 1
    m_SystemDataFile.close();
    m_SystemDataFile.open(m_SystemDataFilePath, std::ios_base::app | std::ios_base::out);
    writeHeaderToSystemDataFile();
    m_SystemDataFile.clear();
#endif

    // CollisionDataFile
#if OUTPUT_COLLISIONDATA_FILE == 1
    m_CollisionDataFile.open(m_CollisionDataFilePath,std::ios::binary | std::ios::out);
#endif


}



void MoreauTimeStepper::reset() {
    //set standart values for parameters
    m_IterationCounter = 0;
    m_bIterationFinished = false;

    m_pSimulationLog->logMessage("---> Reset StatePool...");

    m_pStatePool->resetStatePool(m_pDynSys->m_bodiesInitStates); // Sets initial values to front and back;
    m_StateBuffers = m_pStatePool->getFrontBackBuffer();

    m_pSimulationLog->logMessage("---> Reset DynamicsSystem...");
    m_pDynSys->reset();
    m_settings = m_pDynSys->getSettingsTimeStepper();

    m_pSimulationLog->logMessage("---> Reset CollisionSolver...");
    m_pCollisionSolver->reset();

    m_pSimulationLog->logMessage("---> Reset InclusionSolver...");
    m_pInclusionSolver->reset();

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
        m_pDynSys->applyInitStatesToBodies();
    }

    //m_ReferenceSimFile.writeOutAllStateTimes();

    m_currentSimulationTime = m_settings.m_startTime;
    m_StateBuffers.m_pFront->m_t= m_currentSimulationTime;

    m_AvgTimeForOneIteration = 0;
    m_MaxTimeForOneIteration = 0;

    m_bFinished = false;

    m_PerformanceTimer.start();
};


MoreauTimeStepper::PREC MoreauTimeStepper::getTimeCurrent() {
    return m_currentSimulationTime;
}


unsigned int MoreauTimeStepper::getIterationCount() {
    return m_IterationCounter;
}



const DynamicsState *
MoreauTimeStepper::getBackStateBuffer() {
    return m_StateBuffers.m_pBack;
}



const DynamicsState *
MoreauTimeStepper::getFrontStateBuffer() {
    return m_StateBuffers.m_pFront;
}


void MoreauTimeStepper::doOneIteration() {
    static std::stringstream logstream;

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
    // Middle Time Step ==================================================================

    m_startSimulationTime = m_currentSimulationTime;
    m_pDynSys->doFirstHalfTimeStep(m_startSimulationTime, m_settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    m_pDynSys->doInputTimeStep(m_settings.m_deltaT/2.0);
    // Custom Calculations after first timestep
    m_pDynSys->afterFirstTimeStep();

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

    m_pDynSys->doSecondHalfTimeStep(m_currentSimulationTime, m_settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    m_pDynSys->doInputTimeStep(m_settings.m_deltaT/2.0);
    // Custom Calculations after second timestep
    m_pDynSys->afterSecondTimeStep();
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


void MoreauTimeStepper::swapStateBuffers() {
    m_StateBuffers = m_pStatePool->swapFrontBackBuffer();
}



