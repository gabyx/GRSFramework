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
#include <iostream>
#include <fstream>
#include <cmath>

#include <boost/timer/timer.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

#include "DynamicsState.hpp"
#include "FrontBackBuffer.hpp"
#include "BinaryFile.hpp"
#include "MultiBodySimFile.hpp"
#include "SimpleLogger.hpp"

#include InclusionSolver_INCLUDE_FILE
#include CollisionSolver_INCLUDE_FILE
#include DynamicsSystem_INCLUDE_FILE

#include "MPICommunication.hpp"
#include "BodyCommunicator.hpp"

#include "TimeStepperSettings.hpp"
//===========================================




/**
* @ingroup DynamicsGeneral
* @brief The Moreau time stepper.
*/

class MoreauTimeStepper {
public:

    DEFINE_TIMESTEPPER_CONFIG_TYPES

    typedef typename MPILayer::ProcessCommunicator ProcessCommunicatorType;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    MoreauTimeStepper(boost::shared_ptr<DynamicsSystemType> pDynSys,
                      boost::shared_ptr<ProcessCommunicatorType > pProcCommunicator);
    ~MoreauTimeStepper();

    // The Core Objects ==================================
    boost::shared_ptr<CollisionSolverType>  m_pCollisionSolver;
    boost::shared_ptr<InclusionSolverType>  m_pInclusionSolver;
    boost::shared_ptr<DynamicsSystemType>	m_pDynSys;
    boost::shared_ptr<ProcessCommunicatorType > m_pProcCommunicator;
    boost::shared_ptr<BodyCommunicator > m_pBodyCommunicator;
    // ===================================================

    void initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile="");
    void closeAllFiles();

    void reset();
    void doOneIteration();

    PREC getTimeCurrent();
    unsigned int getIterationCount();

    // Solver Parameters
    TimeStepperSettings m_Settings;

    // General Log file
    Logging::Log *m_pSolverLog, *m_pSimulationLog;

    //Performance Time of one Iteration (averaged)
    PREC m_AvgTimeForOneIteration;
    PREC m_MaxTimeForOneIteration;

    inline bool finished();
    inline void writeIterationToSystemDataFile(PREC globalTime);
    inline void writeHeaderToSystemDataFile();
    inline void writeIterationToCollisionDataFile();

protected:

    PREC m_currentSimulationTime;
    PREC m_startSimulationTime;

    int m_IterationCounter;
    bool m_bIterationFinished;
    bool m_bFinished;

    // Timer for the Performance
    boost::timer::cpu_timer m_PerformanceTimer;
    PREC m_startTime, m_endTime,
    m_startTimeCollisionSolver, m_endTimeCollisionSolver,
    m_startTimeInclusionSolver, m_endTimeInclusionSolver,
    m_startBodyCommunication, m_endBodyCommunication;

    // Collision Data file
    BinaryFile m_CollisionDataFile;

    // System Data file
    boost::filesystem::ofstream m_SystemDataFile;

    // Reference Sim File for Simulation
    MultiBodySimFile m_ReferenceSimFile;


    // Logs
    boost::filesystem::path m_SimFolderPath;
    boost::filesystem::path m_SystemDataFilePath;
    boost::filesystem::path m_CollisionDataFilePath;
    boost::filesystem::path m_SolverLogFilePath;


};

//=========================================================

/*=========================================================
definitions of template class MoreauTimeStepper
_________________________________________________________*/


MoreauTimeStepper::MoreauTimeStepper(boost::shared_ptr<DynamicsSystemType> pDynSys,
                                     boost::shared_ptr<ProcessCommunicatorType > pProcCommunicator):
    m_ReferenceSimFile(),
    m_pSolverLog(NULL),
    m_pDynSys(pDynSys),
    m_pProcCommunicator(pProcCommunicator) {


    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        ERRORMSG("There is no SimulationLog in the LogManager... Did you create it?")
    }


    m_pBodyCommunicator =  boost::shared_ptr<BodyCommunicator >(
                            new BodyCommunicator(m_pDynSys,  m_pProcCommunicator) );

    m_pCollisionSolver = boost::shared_ptr<CollisionSolverType>(new CollisionSolverType(m_pDynSys));
    m_pInclusionSolver = boost::shared_ptr<InclusionSolverType>(new InclusionSolverType(m_pBodyCommunicator, m_pCollisionSolver,m_pDynSys, m_pProcCommunicator));

};




MoreauTimeStepper::~MoreauTimeStepper() {
    m_CollisionDataFile.close();
    m_SystemDataFile.close();
    DECONSTRUCTOR_MESSAGE
};


void MoreauTimeStepper::closeAllFiles() {

    Logging::LogManager::getSingletonPtr()->destroyLog("SolverLog");
    m_pSolverLog = NULL;

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
    Logging::LogManager::getSingletonPtr()->registerLog(m_pSolverLog);

#if LogToFileSolver == 1
    m_pSolverLog->addSink(new Logging::LogSinkFile("SolverLog-File",m_SolverLogFilePath));
#endif
#if LogToConsoleSolver == 1
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

    m_pSimulationLog->logMessage("---> Reset DynamicsSystem...");
    m_pDynSys->reset();
    m_pDynSys->getSettings(m_Settings);

    m_pSimulationLog->logMessage("---> Reset CollisionSolver...");
    m_pCollisionSolver->reset();

    m_pSimulationLog->logMessage("---> Reset InclusionSolver...");
    m_pInclusionSolver->reset();


    m_AvgTimeForOneIteration = 0;
    m_MaxTimeForOneIteration = 0;

    m_currentSimulationTime = m_Settings.m_startTime;



    if(m_Settings.m_eSimulateFromReference != TimeStepperSettings::NONE) {

        //TODO Open all simfiles references for the bodies
        //LOG(m_pSimulationLog,"---> Opened Reference SimFile: m_Settings.m_simStateReferenceFile"<<std::endl);

        if(m_Settings.m_eSimulateFromReference != TimeStepperSettings::CONTINUE) {
            //Inject the end state into the front buffer
            //TODO
        }

    }


    // Apply all init states to the bodies!
    m_pSimulationLog->logMessage("---> Initialize Bodies...");
    m_pDynSys->applyInitStatesToBodies();


    m_bFinished = false;

    m_PerformanceTimer.stop();
    m_PerformanceTimer.start();
};


MoreauTimeStepper::PREC MoreauTimeStepper::getTimeCurrent() {
    return m_currentSimulationTime;
}


unsigned int MoreauTimeStepper::getIterationCount() {
    return m_IterationCounter;
}



void MoreauTimeStepper::doOneIteration() {
    static std::stringstream logstream;

    static int iterations=0; // Average is reset after 1000 Iterations

#if CoutLevelSolver>0
    LOG(m_pSolverLog, "---> Do one time-step =================================" <<std::endl;);
#endif

    m_bIterationFinished = false;

    iterations++;
    m_IterationCounter++;


    m_startTime = ((double)m_PerformanceTimer.elapsed().wall)*1e-9;

    //Force switch
    //boost::thread::yield();

#if CoutLevelSolver==1
     unsigned int when = (m_Settings.m_endTime-m_Settings.m_startTime)/m_Settings.m_deltaT / 10.0;
      if(when<=0){when=1;}
      if(m_IterationCounter % when == 0){
            LOG(m_pSolverLog,"--->  m_t: " << m_currentSimulationTime<<std::endl; );
      }
#endif

#if CoutLevelSolver>1
      LOG(m_pSolverLog,"---> m_t Begin: " << m_currentSimulationTime <<std::endl; );
#endif

    //Calculate Midpoint Rule ============================================================
    // Middle Time Step for all LOCAL Bodies==============================================
    // Remote bodies belong to other processes which are timestepped
    m_startSimulationTime = m_currentSimulationTime;
    m_pDynSys->doFirstHalfTimeStep(m_startSimulationTime, m_Settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    m_pDynSys->doInputTimeStep(m_Settings.m_deltaT/2.0);
    // Custom Calculations after first timestep
    m_pDynSys->afterFirstTimeStep();

    m_currentSimulationTime = m_startSimulationTime + m_Settings.m_deltaT/2.0;
    // ====================================================================================

    m_startBodyCommunication = ((PREC)m_PerformanceTimer.elapsed().wall)*1e-9;
    m_pBodyCommunicator->communicate(m_currentSimulationTime);
    m_endBodyCommunication = ((PREC)m_PerformanceTimer.elapsed().wall)*1e-9;

    /* Communicate all bodies which are in the overlap zone or are out of the processes topology!

     Sending
     Get for each LOCAL Body (overlapping region) where the body is overlapping and the according neighbour process rank
            Get the corresponding topology for the body -> correspondingProcess
            Get all sendNeighbours to the correspondingProcess where the body is in the overlap zone!

                for each neighbour in sendNeighbours
                    if already communicated ( in list bodyToCommunicatedNeigbours, make flag that it is used!) -> send update to neighbour
                    if is not communicated (not in list  bodyToCommunicatedNeigbours, add to list, make flag that it is used)
                        if body is in our process topology = correspondingProcess:
                            -> send whole body to neighbour, send it as remote to neighbour
                        if If body is no more in our process topology != correspondingProcess
                            -> send whole body to neighbour, send it as local to neighbour
                            -> remove body from our local list, it is no more used!

                for each entry which is not flaged in bodyToCommunicatedNeigbours
                    -> remove process index and send removal (or send nothing ) to this process index for this body


     For each sendNeighbour entry make one message with:
          Each information depending on flag (update,whole, removal)

     Receiving
     Receive all updates or new local bodies from neighbours:
     Process all neighbour messages (also if empty):

         (if we receive a removal, remove it from our remote list)
         all remotes which have not been updated are no longer remotes, so remove them! :-)

         if we receive update , search for the remote body and update it
         if we receive a new (local or remote) body, add it either to the remote or local list depending on what it is, check if the body belongs to our topology
    */


    m_pInclusionSolver->resetForNextIter(); // Clears the contact graph and other inclusion related stuff!

    // Solve Collision
    m_startTimeCollisionSolver = ((double)m_PerformanceTimer.elapsed().wall)*1e-9;
    m_pCollisionSolver->solveCollision();
    m_endTimeCollisionSolver =   ((double)m_PerformanceTimer.elapsed().wall)*1e-9;

    //Solve Contact Problem
    //boost::thread::yield();
    m_startTimeInclusionSolver = ((double)m_PerformanceTimer.elapsed().wall)*1e-9;
    m_pInclusionSolver->solveInclusionProblem(m_currentSimulationTime);
    m_endTimeInclusionSolver = ((double)m_PerformanceTimer.elapsed().wall)*1e-9;

    //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    //boost::thread::yield();


    // ===================================================================================
    // Middle Time Step ==================================================================
    m_currentSimulationTime = m_startSimulationTime + m_Settings.m_deltaT ;
    m_pDynSys->doSecondHalfTimeStep(m_currentSimulationTime, m_Settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    m_pDynSys->doInputTimeStep(m_Settings.m_deltaT/2.0);
    // Custom Calculations after second timestep
    m_pDynSys->afterSecondTimeStep();
    // ====================================================================================



#if CoutLevelSolver>2
      LOG(m_pSolverLog,"---> m_t End: " << m_currentSimulationTime <<std::endl );
#endif

    //Force switch
    //boost::thread::yield();

    m_endTime = ((double)m_PerformanceTimer.elapsed().wall)*1e-9;

    // Measure Time again
    if (m_IterationCounter%100==0) {
        m_AvgTimeForOneIteration=0;
        iterations = 1;
    }
    m_AvgTimeForOneIteration = ( (m_endTime - m_startTime) + m_AvgTimeForOneIteration*(iterations-1)) / iterations;
    if (m_AvgTimeForOneIteration > m_MaxTimeForOneIteration) {
        m_MaxTimeForOneIteration = m_AvgTimeForOneIteration;
    }

#if CoutLevelSolver>0
    LOG( m_pSolverLog,  "---> Iteration Time: "<<std::setprecision(5)<<(m_endTime-m_startTime)<<std::endl
    <<  "---> End time-step ====================================" <<std::endl<<std::endl; );
#endif

    // Check if we can finish the timestepping!
    if(m_Settings.m_eSimulateFromReference == TimeStepperSettings::USE_STATES ) {
        m_bFinished =  !m_ReferenceSimFile.isGood();
    } else {
        m_bFinished =  m_currentSimulationTime >= m_Settings.m_endTime;
    }

    m_bIterationFinished = true;
}


bool MoreauTimeStepper::finished() {
    return m_bFinished;
}

void MoreauTimeStepper::writeIterationToSystemDataFile(double globalTime) {
#if OUTPUT_SIMDATA_FILE == 1
    m_SystemDataFile
    << globalTime << "\t"
    << m_startSimulationTime <<"\t"
    << (m_endTime-m_startTime) <<"\t"
    << (m_endTimeCollisionSolver-m_startTimeCollisionSolver) <<"\t"
    << (m_endTimeInclusionSolver-m_startTimeInclusionSolver) <<"\t"
    << (m_endBodyCommunication -  m_startBodyCommunication) << "\t"
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
    << "BodyCommunicationTime [s]" << "\t"
    << "AvgIterTime [s]" <<"\t"
    << m_pCollisionSolver->getStatsHeader() << "\t"
    << m_pInclusionSolver->getStatsHeader() << std::endl;
#endif
}


void MoreauTimeStepper::writeIterationToCollisionDataFile() {
#if OUTPUT_COLLISIONDATA_FILE == 1

    double averageOverlap = 0;

    unsigned int nContacts = m_pCollisionSolver->m_collisionSet.size();
    m_CollisionDataFile << (double)m_StateBuffers.m_pFront->m_t; // Write Time
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
