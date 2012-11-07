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
#include <Eigen/Dense>

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

#include "InclusionSolverCONoGMPI.hpp"
#include "CollisionSolverMPI.hpp"
#include "DynamicsSystemMPI.hpp"


//===========================================




/**
* @ingroup DynamicsGeneral
* @brief The Moreau time stepper.
*/
template< typename TConfigTimeStepper>
class MoreauTimeStepper {
public:

    typedef TConfigTimeStepper TimeStepperConfigType;
    DEFINE_TIMESTEPPER_CONFIG_TYPES_OF( TConfigTimeStepper )


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    MoreauTimeStepper(const unsigned int nSimBodies, boost::shared_ptr<DynamicsSystemType> pDynSys);
    ~MoreauTimeStepper();

    // The Core Objects ==================================
    boost::shared_ptr<CollisionSolverType>  m_pCollisionSolver;
    boost::shared_ptr<InclusionSolverType>  m_pInclusionSolver;
    boost::shared_ptr<DynamicsSystemType>	m_pDynSys;
    // ===================================================

    void initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile="");
    void closeAllFiles();

    void reset();
    void doOneIteration();

    double getTimeCurrent();

    // Solver Parameters
    TimeStepperSettings<LayoutConfigType> m_Settings;

    //Accessed only by Simulation manager, after doOneIteration();
    boost::shared_ptr<const DynamicsState<LayoutConfigType> > getBackStateBuffer();
    boost::shared_ptr<const DynamicsState<LayoutConfigType> > getFrontStateBuffer();

    // General Log file
    Logging::Log*	m_pSolverLog;

    //Performance Time of one Iteration (averaged)
    double m_AvgTimeForOneIteration;
    double m_MaxTimeForOneIteration;

    inline bool finished();
    inline void writeIterationToSystemDataFile(double globalTime);
    inline void writeIterationToCollisionDataFile();

protected:

    const unsigned int m_nDofu, m_nDofq; // These are the global dimensions of q and u
    const unsigned int m_nDofuObj, m_nDofqObj, m_nSimBodies; // These are the dimensions for one Obj

    int m_IterationCounter;

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



    typedef FrontBackBuffer<DynamicsState<LayoutConfigType>, FrontBackBufferPtrType::SharedPtr, FrontBackBufferMode::BackConst> FrontBackBufferType;
    FrontBackBufferType m_StateBuffers;
    //Solver state pool front and back buffer
    void swapStateBuffers();

    DynamicsState<LayoutConfigType> m_state_m;  // middle state of iteration

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

template< typename TConfigTimeStepper>
MoreauTimeStepper<  TConfigTimeStepper>::MoreauTimeStepper(const unsigned int nSimBodies, boost::shared_ptr<DynamicsSystemType> pDynSys):
    m_state_m(nSimBodies),
    m_nSimBodies(nSimBodies),
    m_nDofqObj(NDOFqObj),
    m_nDofuObj(NDOFuObj),
    m_nDofq(m_nSimBodies * m_nDofqObj),
    m_nDofu(m_nSimBodies * m_nDofuObj),
    m_ReferenceSimFile(NDOFqObj,NDOFuObj) {

    m_pSolverLog = NULL;

    m_pDynSys = pDynSys;
    m_pDynSys->init();

    m_pCollisionSolver = boost::shared_ptr<CollisionSolverType>(new CollisionSolverType(m_pDynSys->m_SimBodies, m_pDynSys->m_Bodies));
    m_pInclusionSolver = boost::shared_ptr<InclusionSolverType>(new InclusionSolverType(m_pCollisionSolver,m_pDynSys));

};



template< typename TConfigTimeStepper>
MoreauTimeStepper<  TConfigTimeStepper>::~MoreauTimeStepper() {
    m_CollisionDataFile.close();
    m_SystemDataFile.close();
    DECONSTRUCTOR_MESSAGE
};

template< typename TConfigTimeStepper>
void MoreauTimeStepper<  TConfigTimeStepper>::closeAllFiles() {

    Logging::LogManager::getSingletonPtr()->destroyLog("SolverLog");
    m_pSolverLog = NULL;

    m_CollisionDataFile.close();
    m_SystemDataFile.close();
}

template< typename TConfigTimeStepper>
void MoreauTimeStepper<  TConfigTimeStepper>::initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile  ) {

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


template< typename TConfigTimeStepper>
void MoreauTimeStepper<  TConfigTimeStepper>::reset() {
    //set standart values for parameters
    m_IterationCounter = 0;

    m_pDynSys->reset();
    m_pDynSys->getSettings(m_Settings, m_pInclusionSolver->m_Settings);

    m_pCollisionSolver->reset();
    m_pInclusionSolver->reset();


    m_AvgTimeForOneIteration = 0;
    m_MaxTimeForOneIteration = 0;

    m_bFinished = false;
};

template< typename TConfigTimeStepper>
double MoreauTimeStepper<  TConfigTimeStepper>::getTimeCurrent() {
    return m_StateBuffers.m_pBack->m_t;
}

template< typename TConfigTimeStepper>
boost::shared_ptr<
const DynamicsState<
typename TConfigTimeStepper::DynamicsSystemType::RigidBodyType::RigidBodyConfigType::LayoutConfigType
>
>
MoreauTimeStepper<  TConfigTimeStepper>::getBackStateBuffer() {
    return m_StateBuffers.m_pBack;
}


template< typename TConfigTimeStepper>
boost::shared_ptr<
const DynamicsState<
typename TConfigTimeStepper::DynamicsSystemType::RigidBodyType::RigidBodyConfigType::LayoutConfigType
>
>
MoreauTimeStepper<  TConfigTimeStepper>::getFrontStateBuffer() {
    return m_StateBuffers.m_pFront;
}

template< typename TConfigTimeStepper>
void MoreauTimeStepper<  TConfigTimeStepper>::doOneIteration() {
    static std::stringstream logstream;

    static int iterations=0; // Average is reset after 1000 Iterations

#if CoutLevelSolver>0
    LOG(m_pSolverLog, "% Do one time-step =================================" <<std::endl;);
#endif

    m_PerformanceTimer.stop();
    m_PerformanceTimer.start();

    iterations++;
    m_IterationCounter++;

    //Force switch
    //boost::thread::yield();


    //Calculate Midpoint Rule ============================================================
    // Middle Time Step for all LOCAL Bodies==============================================
    // Remote bodies belong to other processes which are timestepped
    m_pDynSys->doFirstHalfTimeStep(m_Settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    m_pDynSys->doInputTimeStep(m_Settings.m_deltaT/2.0);
    // Custom Calculations after first timestep
    m_pDynSys->afterFirstTimeStep();
    // ====================================================================================



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
    m_pDynSys->doSecondHalfTimeStep(m_Settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    m_pDynSys->doInputTimeStep(m_Settings.m_deltaT/2.0);
    // Custom Calculations after second timestep
    m_pDynSys->afterSecondTimeStep();
    // ====================================================================================



#if CoutLevelSolver>1
//      LOG(m_pSolverLog,   << "m_pFront->m_t: " << m_StateBuffers.m_pFront->m_t<<std::endl
//                          << "m_pFront->m_q: " << m_StateBuffers.m_pFront->m_q.transpose()<<std::endl
//                          << "m_pFront->m_u: " << m_StateBuffers.m_pFront->m_u.transpose()<<std::endl;);
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
    if(m_Settings.m_eSimulateFromReference == TimeStepperSettings<LayoutConfigType>::USE_STATES ) {
        m_bFinished =  !m_ReferenceSimFile.isGood();
    } else {
        m_bFinished =  m_StateBuffers.m_pFront->m_t >= m_Settings.m_endTime;
    }
}

template< typename TConfigTimeStepper>
bool MoreauTimeStepper<  TConfigTimeStepper>::finished() {
    return m_bFinished;
}
template< typename TConfigTimeStepper>
void MoreauTimeStepper<  TConfigTimeStepper>::writeIterationToSystemDataFile(double globalTime) {
#if OUTPUT_SYSTEMDATA_FILE == 1

    m_SystemDataFile
    << globalTime << "\t"
    << m_StateBuffers.m_pBack->m_t <<"\t"
    << (double)(m_endTime-m_startTime) <<"\t"
    << (double)(m_endTimeCollisionSolver-m_startTimeCollisionSolver) <<"\t"
    << (double)(m_endTimeInclusionSolver-m_startTimeInclusionSolver) <<"\t"
    << m_AvgTimeForOneIteration <<"\t"
    << m_pInclusionSolver->getIterationStats() << std::endl;
#endif
}
template< typename TConfigTimeStepper>
void MoreauTimeStepper<  TConfigTimeStepper>::writeIterationToCollisionDataFile() {
#if OUTPUT_COLLISIONDATA_FILE == 1

    double averageOverlap = 0;

    unsigned int nContacts = m_pCollisionSolver->m_CollisionSet.size();
    m_CollisionDataFile << (double)m_StateBuffers.m_pFront->m_t; // Write Time
    m_CollisionDataFile << nContacts; // Write number of Contacts
    for(unsigned int i=0; i<nContacts; i++) {
        averageOverlap += m_pCollisionSolver->m_CollisionSet[i].m_overlap;
        m_CollisionDataFile<< (double)m_pCollisionSolver->m_CollisionSet[i].m_overlap;
    }
    averageOverlap /= nContacts;
    m_CollisionDataFile<< (double)averageOverlap;
#endif
}

#endif
