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
#include <Eigen/Dense>
#include <OGRE/Ogre.h>
#include <boost/filesystem.hpp>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"
#include "DynamicsState.hpp"
#include "FrontBackBuffer.hpp"
#include "BinaryFile.hpp"
#include "MultiBodySimFile.hpp"

#include "TimeStepperSettings.hpp"

//===========================================




/**
* @ingroup DynamicsGeneral
* @brief The Moreau time stepper.
*/
template< typename TLayoutConfig ,
          typename TDynamicsSystem,
          typename TCollisionSolver,
          typename TInclusionSolver,
          typename TStatePool>
class MoreauTimeStepper {
public:
  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  MoreauTimeStepper(const unsigned int nSimBodies, boost::shared_ptr<TDynamicsSystem> pDynSys,  boost::shared_ptr<TStatePool>	pSysState);
  ~MoreauTimeStepper();

  // The Core Objects ==================================
  boost::shared_ptr<TCollisionSolver>  m_pCollisionSolver;
  boost::shared_ptr<TInclusionSolver>  m_pInclusionSolver;
  boost::shared_ptr<TDynamicsSystem>	m_pDynSys;
  boost::shared_ptr<TStatePool>		   m_pStatePool;
  // ===================================================

  void initLogs(const boost::filesystem::path &folder_path);
  void closeAllFiles();
  void initialize( boost::shared_ptr<TDynamicsSystem> pDynSys, boost::shared_ptr<TStatePool>	pSysState);
  void reset();
  void doOneIteration();

  double getTimeCurrent();

  // Solver Parameters
  TimeStepperSettings<TLayoutConfig> m_Settings;

  //Accessed only by Simulation manager, after doOneIteration();
  boost::shared_ptr<const DynamicsState<TLayoutConfig> > getBackStateBuffer();
  boost::shared_ptr<const DynamicsState<TLayoutConfig> > getFrontStateBuffer();

  // General Log file
  Ogre::Log*	m_pSolverLog;

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
  Ogre::Timer m_PerformanceTimer;
  double m_startTime, m_endTime, m_startTimeCollisionSolver, m_endTimeCollisionSolver, m_startTimeInclusionSolver, m_endTimeInclusionSolver;

   // Collision Data file
  BinaryFile m_CollisionDataFile;

  // System Data file
  std::ofstream m_SystemDataFile;

  // Reference Sim File for Simulation
  MultiBodySimFile<TLayoutConfig> m_ReferenceSimFile;

  //Solver state pool front and back buffer
  void swapStateBuffers();
  FrontBackBuffer<TLayoutConfig> m_StateBuffers;

  DynamicsState<TLayoutConfig> m_state_m;  // end state of iteration

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
#include <iostream>
#include "DynamicsSystem.hpp"

#include "LogDefines.hpp"

template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
MoreauTimeStepper<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::MoreauTimeStepper(const unsigned int nSimBodies, boost::shared_ptr<TDynamicsSystem> pDynSys,  boost::shared_ptr<TStatePool>	pSysState):
m_state_m(nSimBodies),
m_nSimBodies(nSimBodies),
m_nDofqObj(NDOFqObj),
m_nDofuObj(NDOFuObj),
m_nDofq(m_nSimBodies * m_nDofqObj),
m_nDofu(m_nSimBodies * m_nDofuObj)
{

  m_pSolverLog = NULL;

  // Instanciate all Core Objects
  m_pStatePool = pSysState;

  m_pDynSys = pDynSys;
  m_pDynSys->init();

  m_pCollisionSolver = boost::shared_ptr<TCollisionSolver>(new TCollisionSolver(nSimBodies, m_pDynSys->m_SimBodies, m_pDynSys->m_Bodies));
  m_pInclusionSolver = boost::shared_ptr<TInclusionSolver>(new TInclusionSolver(m_pCollisionSolver,m_pDynSys));

};



template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::~MoreauTimeStepper()
{
   m_CollisionDataFile.close();
   m_SystemDataFile.close();
  DECONSTRUCTOR_MESSAGE
};

template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
void MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::closeAllFiles(){
   if(m_pSolverLog != NULL){
    Ogre::LogManager::getSingletonPtr()->destroyLog(m_pSolverLog);
   }
   m_pSolverLog = NULL;

   m_CollisionDataFile.close();
   m_SystemDataFile.close();
}

template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
void MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::initLogs(  const boost::filesystem::path &folder_path ){

  // Set new Simfile Path
  m_SimFolderPath = folder_path;

  // Set new SystemDataFile path
  m_SystemDataFilePath = m_SimFolderPath;
  std::string filename = SYSTEM_DATA_FILE_PREFIX;
  filename += ".dat";
  m_SystemDataFilePath /= filename;

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
  if(m_pSolverLog != NULL){
    Ogre::LogManager::getSingletonPtr()->destroyLog(m_pSolverLog);
  }
  m_pSolverLog = NULL;

#if LogToFileSolver == 1
  m_pSolverLog = Ogre::LogManager::getSingleton().createLog(m_SolverLogFilePath.string(),false,true,false);
  // If not sucessfull in writing the log to the Sim folder, take default log!
  if(m_pSolverLog == NULL){
    m_pSolverLog = Ogre::LogManager::getSingleton().createLog("SolverLogFile.log",false,true,false);
  }

#else
  m_pSolverLog = Ogre::LogManager::getSingleton().createLog(m_SolverLogFilePath.string(),false,true,true);
  // If not sucessfull in writing the log to the Sim folder, take default log!
  if(m_pSolverLog == NULL){
    m_pSolverLog = Ogre::LogManager::getSingleton().createLog("SolverLogFile.log",false,true,true);
  }
#endif
#if LogToConsoleSolver == 1
  m_pSolverLog->setDebugOutputEnabled(true);
#else
  m_pSolverLog->setDebugOutputEnabled(false);
#endif
  m_pSolverLog->setTimeStampEnabled(false);


  m_pDynSys->initializeLog(m_pSolverLog);
  m_pInclusionSolver->initializeLog(m_pSolverLog,m_SolverLogFilePath);
  m_pCollisionSolver->initializeLog(m_pSolverLog);

  // SystemDataFile
#if OUTPUT_SYSTEMDATA_FILE == 1
  m_SystemDataFile.close();
  m_SystemDataFile.open(m_SystemDataFilePath.string().c_str());
  m_SystemDataFile.clear();
#endif

   // CollisionDataFile
#if OUTPUT_COLLISIONDATA_FILE == 1
  m_CollisionDataFile.open(m_CollisionDataFilePath,std::ios::binary | std::ios::out);
#endif


}


template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
void MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::reset()
{
  //set standart values for parameters
  m_IterationCounter = 0;

  m_pStatePool->resetStatePool();
  m_StateBuffers = m_pStatePool->getFrontBackBuffer();

  m_pDynSys->reset();
  m_pDynSys->getSettings(m_Settings, m_pInclusionSolver->m_Settings);

  m_pCollisionSolver->reset();
  m_pInclusionSolver->reset();


  if(m_Settings.m_eSimulateFromReference != TimeStepperSettings<TLayoutConfig>::NONE){
     if(!m_ReferenceSimFile.openSimFileRead(m_Settings.m_stateReferenceFile,m_nSimBodies,true)){
        std::stringstream error;
        error << "Could not open file: " << m_Settings.m_stateReferenceFile.string()<<std::endl;
        error << "File errors: " <<std::endl<< m_ReferenceSimFile.getErrorString();
         m_pSolverLog->logMessage( error.str());
         ERRORMSG(error);
     }

     //m_ReferenceSimFile.writeOutAllStateTimes();

     //Inject the end state into the front buffer
     m_ReferenceSimFile.getEndState(*m_StateBuffers.m_pFront);
  }

  m_PerformanceTimer.reset();
  m_AvgTimeForOneIteration = 0;
  m_MaxTimeForOneIteration = 0;

  m_bFinished = false;
};

template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
double MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::getTimeCurrent()
{
  return m_StateBuffers.m_pBack->m_t;
}
template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
boost::shared_ptr<const DynamicsState<TLayoutConfig> > MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::getBackStateBuffer()
{
  return m_StateBuffers.m_pBack;
}
template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
boost::shared_ptr<const DynamicsState<TLayoutConfig> > MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::getFrontStateBuffer()
{
  return m_StateBuffers.m_pFront;
}

template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
void MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::doOneIteration()
{
  static std::stringstream logstream;

  static int iterations=0; // Average is reset after 1000 Iterations

#if CoutLevelSolver>0
  CLEARLOG;
  logstream << "% Do one time-step =================================" <<std::endl; LOG(m_pSolverLog);
#endif

  m_PerformanceTimer.reset();
  m_startTime = (double)m_PerformanceTimer.getMicroseconds() * 1.0e-6;

  iterations++;
  m_IterationCounter++;

  //Force switch
  //boost::thread::yield();

  // If we should load the state from a reference file! Do this here!
  if(m_Settings.m_eSimulateFromReference == TimeStepperSettings<TLayoutConfig>::USE_STATES && !m_bFinished){
     m_ReferenceSimFile >> m_StateBuffers.m_pFront;
  }

  // Swap front and back buffers!
  swapStateBuffers();

  //Calculate Midpoint Rule ============================================================
  // Middle Time Step ==================================================================
  m_pDynSys->doFirstHalfTimeStep(m_StateBuffers.m_pBack.get(), &m_state_m, m_Settings.m_deltaT/2.0);
  // Custom Integration for Inputs
  m_pDynSys->doInputTimeStep(m_Settings.m_deltaT/2.0);
  // Custom Calculations after first timestep
  m_pDynSys->afterFirstTimeStep(m_StateBuffers.m_pBack.get());
  // ====================================================================================

  m_pInclusionSolver->resetForNextIter(); // Clears the contact graph!

  // Solve Collision
  m_startTimeCollisionSolver = (double)m_PerformanceTimer.getMicroseconds() * 1.0e-6;
  m_pCollisionSolver->solveCollision(&m_state_m);
  m_endTimeCollisionSolver = (double)m_PerformanceTimer.getMicroseconds() * 1.0e-6;

  //Solve Contact Problem
  //boost::thread::yield();
  m_startTimeInclusionSolver = (double)m_PerformanceTimer.getMicroseconds() * 1.0e-6;
  m_pInclusionSolver->solveInclusionProblem(m_StateBuffers.m_pBack.get(), &m_state_m, m_StateBuffers.m_pFront.get());
  m_endTimeInclusionSolver = (double)m_PerformanceTimer.getMicroseconds() * 1.0e-6;

  //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
  //boost::thread::yield();


  // ===================================================================================

  // Middle Time Step ==================================================================
  m_pDynSys->doSecondHalfTimeStep(&m_state_m, m_StateBuffers.m_pFront.get(), m_Settings.m_deltaT/2.0);
  // Custom Integration for Inputs
  m_pDynSys->doInputTimeStep(m_Settings.m_deltaT/2.0);
  // Custom Calculations after second timestep
  m_pDynSys->afterSecondTimeStep(m_StateBuffers.m_pBack.get());
  // ====================================================================================

  #if CoutLevelSolver>1
  	CLEARLOG;
      logstream << "m_pFront->m_t: " << m_StateBuffers.m_pFront->m_t<<endl;
      logstream << "m_pFront->m_q: " << m_StateBuffers.m_pFront->m_q.transpose()<<endl;
      logstream << "m_pFront->m_u: " << m_StateBuffers.m_pFront->m_u.transpose()<<endl;
      LOG(m_pSolverLog);
  #endif

  //Force switch
  //boost::thread::yield();


  // Measure Time again
  m_endTime = (double) m_PerformanceTimer.getMicroseconds() * 1.0e-6;
  if (m_IterationCounter%100==0){
    m_AvgTimeForOneIteration=0;
    iterations = 1;
  }
  m_AvgTimeForOneIteration = ((m_endTime-m_startTime) + m_AvgTimeForOneIteration*(iterations-1)) / iterations;
  if (m_AvgTimeForOneIteration > m_MaxTimeForOneIteration){
    m_MaxTimeForOneIteration = m_AvgTimeForOneIteration;
  }

#if CoutLevelSolver>0
  CLEARLOG;
  logstream << "% Iteration Time: "<<std::setprecision(5)<<(double)(m_endTime-m_startTime)<<std::endl;
  logstream << "% End time-step ====================================" <<std::endl<<std::endl; LOG(m_pSolverLog);
#endif

   // Check if we can finish the timestepping!
   if(m_Settings.m_eSimulateFromReference == TimeStepperSettings<TLayoutConfig>::USE_STATES ){
      m_bFinished =  !m_ReferenceSimFile.isGood();
   }else{
      m_bFinished =  m_StateBuffers.m_pFront->m_t >= m_Settings.m_endTime;
   }
}

template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
bool MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::finished(){
   return m_bFinished;
}

template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
void MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::writeIterationToSystemDataFile(double globalTime){
   #if OUTPUT_SYSTEMDATA_FILE == 1
         m_SystemDataFile
        << globalTime << "\t"
        << m_StateBuffers.m_pBack->m_t <<"\t"
        << (double)(m_endTime-m_startTime) <<"\t"
        << (double)(m_endTimeCollisionSolver-m_startTimeCollisionSolver) <<"\t"
        << (double)(m_endTimeInclusionSolver-m_startTimeInclusionSolver) <<"\t"
        << m_AvgTimeForOneIteration <<"\t"
        << m_pInclusionSolver->m_bUsedGPU<<"\t"
        << m_pInclusionSolver->m_nContacts<<"\t"
        << m_pInclusionSolver->m_iterationsNeeded<<"\t"
        << m_pInclusionSolver->m_bConverged<<"\t"
        << m_pInclusionSolver->m_isFinite<<"\t"
        << m_pInclusionSolver->m_timeProx<<"\t"
        << m_pInclusionSolver->m_proxIterationTime<<"\t"
        << m_pDynSys->m_CurrentStateEnergy <<"\t"
        << m_pInclusionSolver->m_G_conditionNumber<<"\t"
        << m_pInclusionSolver->m_G_notDiagDominant<<"\t"
        << m_pInclusionSolver->m_PercussionPool.getPoolSize()<<std::endl;

   #endif
}
template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
void MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::writeIterationToCollisionDataFile(){
   #if OUTPUT_COLLISIONDATA_FILE == 1

   double averageOverlap = 0;

   unsigned int nContacts = m_pCollisionSolver->m_CollisionSet.size();
      m_CollisionDataFile << (double)m_StateBuffers.m_pFront->m_t; // Write Time
      m_CollisionDataFile << nContacts; // Write number of Contacts
      for(unsigned int i=0; i<nContacts;i++){
         averageOverlap += m_pCollisionSolver->m_CollisionSet[i].m_overlap;
         m_CollisionDataFile<< (double)m_pCollisionSolver->m_CollisionSet[i].m_overlap;
      }
      averageOverlap /= nContacts;
      m_CollisionDataFile<< (double)averageOverlap;
   #endif
}

template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool>
void MoreauTimeStepper<TLayoutConfig,TDynamicsSystem, TCollisionSolver, TInclusionSolver, TStatePool>::swapStateBuffers(){
  m_StateBuffers = m_pStatePool->swapFrontBackBuffer();
}

#endif
