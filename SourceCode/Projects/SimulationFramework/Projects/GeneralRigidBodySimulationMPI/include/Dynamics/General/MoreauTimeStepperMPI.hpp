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

#include "TimeStepperSettings.hpp"

#include "SimpleLogger.hpp"

#include InclusionSolver_INCLUDE_FILE
#include CollisionSolver_INCLUDE_FILE
#include DynamicsSystem_INCLUDE_FILE

#include "MPICommunication.hpp"
#include "NeighbourCommunicator.hpp"
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
                      boost::shared_ptr<NeighbourCommunicator > pNbCommunicator);
    ~MoreauTimeStepper();

    // The Core Objects ==================================
    boost::shared_ptr<CollisionSolverType>  m_pCollisionSolver;
    boost::shared_ptr<InclusionSolverType>  m_pInclusionSolver;
    boost::shared_ptr<DynamicsSystemType>	m_pDynSys;
    boost::shared_ptr<NeighbourCommunicator > m_pNbCommunicator;
    // ===================================================

    void initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile="");
    void closeAllFiles();

    void reset();
    void doOneIteration();

    double getTimeCurrent();
    unsigned int getIterationCount();

    // Solver Parameters
    TimeStepperSettings m_Settings;

    // General Log file
    Logging::Log *m_pSolverLog, *m_pSimulationLog;

    //Performance Time of one Iteration (averaged)
    double m_AvgTimeForOneIteration;
    double m_MaxTimeForOneIteration;

    inline bool finished();
    inline void writeIterationToSystemDataFile(double globalTime);
    inline void writeIterationToCollisionDataFile();

protected:

    PREC m_currentSimulationTime;

    int m_IterationCounter;
    bool m_bIterationFinished;
    bool m_bFinished;

    // Timer for the Performance
    boost::timer::cpu_timer m_PerformanceTimer;
    double m_startTime, m_endTime, m_startTimeCollisionSolver, m_endTimeCollisionSolver, m_startTimeInclusionSolver, m_endTimeInclusionSolver;

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
                                     boost::shared_ptr<NeighbourCommunicator > pNbCommunicator):
    m_ReferenceSimFile(NDOFqObj,NDOFuObj) {


    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        ERRORMSG("There is no SimulationLog in the LogManager... Did you create it?")
    }

    m_pSolverLog = NULL;

    m_pDynSys = pDynSys;

    m_pNbCommunicator = pNbCommunicator;

    m_pCollisionSolver = boost::shared_ptr<CollisionSolverType>(new CollisionSolverType(m_pDynSys));
    m_pInclusionSolver = boost::shared_ptr<InclusionSolverType>(new InclusionSolverType(m_pCollisionSolver,m_pDynSys));

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
    m_pInclusionSolver->initializeLog(m_pSolverLog,m_SolverLogFilePath);
    m_pCollisionSolver->initializeLog(m_pSolverLog);

    // SystemDataFile
#if OUTPUT_SYSTEMDATA_FILE == 1
    m_SystemDataFile.close();
    m_SystemDataFile.open(m_SystemDataFilePath, std::ios_base::app | std::ios_base::out);
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
    m_pDynSys->getSettings(m_Settings, m_pInclusionSolver->m_Settings);

    m_pSimulationLog->logMessage("---> Reset CollisionSolver...");
    m_pCollisionSolver->reset();

    m_pSimulationLog->logMessage("---> Reset InclusionSolver...");
    m_pInclusionSolver->reset();


    m_AvgTimeForOneIteration = 0;
    m_MaxTimeForOneIteration = 0;

    m_currentSimulationTime = 0;



    if(m_Settings.m_eSimulateFromReference != TimeStepperSettings::NONE) {

        //TODO Open all simfiles references for the bodies
        //LOG(m_pSimulationLog,"---> Opened Reference SimFile: m_Settings.m_simStateReferenceFile"<<std::endl);

        if(m_Settings.m_eSimulateFromReference != TimeStepperSettings::CONTINUE) {
            //Inject the end state into the front buffer
            //TODO
        }

    }

    m_bFinished = false;
};


double MoreauTimeStepper::getTimeCurrent() {
    return m_currentSimulationTime;
}


unsigned int MoreauTimeStepper::getIterationCount() {
    return m_IterationCounter;
}



void MoreauTimeStepper::doOneIteration() {
    static std::stringstream logstream;

    static int iterations=0; // Average is reset after 1000 Iterations

#if CoutLevelSolver>0
    LOG(m_pSolverLog, "% Do one time-step =================================" <<std::endl;);
#endif

    m_PerformanceTimer.stop();
    m_PerformanceTimer.start();

    m_bIterationFinished = false;

    iterations++;
    m_IterationCounter++;


    //Force switch
    //boost::thread::yield();

#if CoutLevelSolver==1
      if(m_IterationCounter % 10000 == 1){
            LOG(m_pSolverLog,"% m_t: " << m_currentSimulationTime <<std::endl; );
      }
#endif

#if CoutLevelSolver>2
      LOG(m_pSolverLog,"m_t Begin: " << m_currentSimulationTime <<std::endl; );
#endif

    //Calculate Midpoint Rule ============================================================
    // Middle Time Step for all LOCAL Bodies==============================================
    // Remote bodies belong to other processes which are timestepped
    double ts = m_currentSimulationTime;
    m_pDynSys->doFirstHalfTimeStep(ts, m_Settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    m_pDynSys->doInputTimeStep(m_Settings.m_deltaT/2.0);
    // Custom Calculations after first timestep
    m_pDynSys->afterFirstTimeStep();

    m_currentSimulationTime = ts + m_Settings.m_deltaT/2.0;
    // ====================================================================================

    m_pNbCommunicator->communicate(m_currentSimulationTime);
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







    m_pInclusionSolver->resetForNextIter(); // Clears the contact graph!

    // Solve Collision
    m_startTimeCollisionSolver = ((double)m_PerformanceTimer.elapsed().wall)*1e-9;
    m_pCollisionSolver->solveCollision();
    m_endTimeCollisionSolver =   ((double)m_PerformanceTimer.elapsed().wall)*1e-9;

    //Solve Contact Problem
    //boost::thread::yield();
    m_startTimeInclusionSolver = ((double)m_PerformanceTimer.elapsed().wall)*1e-9;
    m_pInclusionSolver->solveInclusionProblem();
    m_endTimeInclusionSolver = ((double)m_PerformanceTimer.elapsed().wall)*1e-9;

    //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    //boost::thread::yield();


    // ===================================================================================
    // Middle Time Step ==================================================================
    m_currentSimulationTime = ts + m_Settings.m_deltaT ;
    m_pDynSys->doSecondHalfTimeStep(m_currentSimulationTime, m_Settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    m_pDynSys->doInputTimeStep(m_Settings.m_deltaT/2.0);
    // Custom Calculations after second timestep
    m_pDynSys->afterSecondTimeStep();
    // ====================================================================================



#if CoutLevelSolver>2
      LOG(m_pSolverLog,"m_t End: " << m_currentSimulationTime <<std::endl );
#endif

    //Force switch
    //boost::thread::yield();


    // Measure Time again
    if (m_IterationCounter%100==0) {
        m_AvgTimeForOneIteration=0;
        iterations = 1;
    }
    m_AvgTimeForOneIteration = ( ((double)m_PerformanceTimer.elapsed().wall)*1e-9  + m_AvgTimeForOneIteration*(iterations-1)) / iterations;
    if (m_AvgTimeForOneIteration > m_MaxTimeForOneIteration) {
        m_MaxTimeForOneIteration = m_AvgTimeForOneIteration;
    }

#if CoutLevelSolver>0
    //LOG( m_pSolverLog,  "% Iteration Time: "<<std::setprecision(5)<<(double)(m_endTime-m_startTime)<<std::endl
    // <<  "% End time-step ====================================" <<std::endl<<std::endl; );
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
#if OUTPUT_SYSTEMDATA_FILE == 1

    m_SystemDataFile
    << globalTime << "\t"
    << m_currentSimulationTime <<"\t"
    << (double)(m_endTime-m_startTime) <<"\t"
    << (double)(m_endTimeCollisionSolver-m_startTimeCollisionSolver) <<"\t"
    << (double)(m_endTimeInclusionSolver-m_startTimeInclusionSolver) <<"\t"
    << m_AvgTimeForOneIteration <<"\t"
    << m_pInclusionSolver->getIterationStats() << std::endl;
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
