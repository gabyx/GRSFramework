/*
*  MoreauTimeStepperMPI.h
*  DynamicSystem
*
*  Created by Gabriel Nützi on 21.03.10.
*  Copyright 2010 ETH. All rights reserved.
*
*/
#ifndef GRSF_Dynamics_General_MoreauTimeStepperMPI_hpp
#define GRSF_Dynamics_General_MoreauTimeStepperMPI_hpp

// Includes =================================
#include <iostream>
#include <fstream>
#include <cmath>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/Dynamics/General/MoreauTimeStepperBase.hpp"

#include "GRSF/Dynamics/General/MPICommunication.hpp"
#include "GRSF/Dynamics/General/BodyCommunicator.hpp"

#include "GRSF/Dynamics/General/TimeStepperSettings.hpp"
//===========================================

#include "GRSF/Common/CPUTimer.hpp"


/**
* @ingroup DynamicsGeneral
* @brief The Moreau time stepper.
*/

struct MoreauTimeStepperMPITraits{
    DEFINE_TIMESTEPPER_CONFIG_TYPES;
    using TimeStepperSettingsType = TimeStepperSettings;
};

class MoreauTimeStepperMPI : public MoreauTimeStepperBase<MoreauTimeStepperMPI,MoreauTimeStepperMPITraits> {
public:

    DEFINE_TIMESTEPPER_CONFIG_TYPES

    using ProcessCommunicatorType = MPILayer::ProcessCommunicator;
    using TopologyBuilderType = MPILayer::TopologyBuilder;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    MoreauTimeStepperMPI(std::shared_ptr<DynamicsSystemType> pDynSys,
                         std::shared_ptr<ProcessCommunicatorType > pProcCommunicator,
                         std::shared_ptr<TopologyBuilderType> pTopologyBuilder);
    ~MoreauTimeStepperMPI();

    // The Core Objects ==================================
    std::shared_ptr<ProcessCommunicatorType > m_pProcCommunicator;
    std::shared_ptr<BodyCommunicator > m_pBodyCommunicator;
    std::shared_ptr<TopologyBuilderType > m_pTopologyBuilder;
    // ===================================================

    void initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile="");
    void closeAllFiles();

    void reset();
    void resetTopology();
    void doOneIteration();

    inline void writeIterationToSystemDataFile(PREC globalTime);
    inline void writeHeaderToSystemDataFile();
    inline void writeIterationToCollisionDataFile();

protected:
    inline  void afterFirstTimeStep() {};
    inline  void afterSecondTimeStep() {};
    inline  void doInputTimeStep(PREC T) {};


    // Timer for the Performance
    CPUTimer m_PerformanceTimer;
    PREC m_startBodyCommunication, m_endBodyCommunication;
};



MoreauTimeStepperMPI::MoreauTimeStepperMPI( std::shared_ptr<DynamicsSystemType> pDynSys,
                                            std::shared_ptr<ProcessCommunicatorType > pProcCommunicator,
                                            std::shared_ptr<TopologyBuilderType> pTopologyBuilder):
    MoreauTimeStepperBase<MoreauTimeStepperMPI, MoreauTimeStepperMPITraits>(pDynSys),
    m_pProcCommunicator(pProcCommunicator), m_pTopologyBuilder(pTopologyBuilder) {


    m_pBodyCommunicator =  std::shared_ptr<BodyCommunicator >( new BodyCommunicator(m_pDynSys,  m_pProcCommunicator) );

    m_pCollisionSolver = std::shared_ptr<CollisionSolverType>(new CollisionSolverType(m_pDynSys));
    m_pInclusionSolver = std::shared_ptr<InclusionSolverType>(new InclusionSolverType(m_pBodyCommunicator,
                                                                                      m_pCollisionSolver,
                                                                                      m_pDynSys,
                                                                                      m_pProcCommunicator));

};




MoreauTimeStepperMPI::~MoreauTimeStepperMPI() {
    closeAllFiles();
    DECONSTRUCTOR_MESSAGE
};


void MoreauTimeStepperMPI::closeAllFiles() {
    MoreauTimeStepperBase::closeAllFiles_impl();
}


void MoreauTimeStepperMPI::initLogs(  const boost::filesystem::path &folder_path, const boost::filesystem::path &simDataFile  ) {

   MoreauTimeStepperBase::initLogs_impl(folder_path,simDataFile);
   writeHeaderToSystemDataFile();

}



void MoreauTimeStepperMPI::reset() {


    if(m_settings.m_eSimulateFromReference != TimeStepperSettings::NONE) {

        //TODO Open all simfiles references for the bodies
        //LOG(m_pSimulationLog,"---> Opened Reference SimFile: m_settings.m_simStateReferenceFile"<<std::endl);
        ERRORMSG("NOT IMPLEMENTED")
        if(m_settings.m_eSimulateFromReference != TimeStepperSettings::CONTINUE) {
            //Inject the end state into the front buffer
            //TODO
        }
    }else{
        // Apply all init states to the bodies!
        m_pSimulationLog->logMessage("---> Initialize Bodies...");
        m_pDynSys->applyInitStatesToBodies();
    }

    m_pSimulationLog->logMessage("---> Reset BodyCommunicator...");
    m_pBodyCommunicator->reset();

    MoreauTimeStepperBase::reset_impl();

};


void MoreauTimeStepperMPI::resetTopology() {

    m_pSimulationLog->logMessage("---> Reset BodyCommunicator...");
    m_pBodyCommunicator->resetTopology();

    m_pSimulationLog->logMessage("---> Reset CollisionSolver...");
    m_pCollisionSolver->resetTopology();

    m_pSimulationLog->logMessage("---> Reset InclusionSolver..");
    m_pInclusionSolver->resetTopology();
};



void MoreauTimeStepperMPI::doOneIteration() {

    static int iterations=0; // Average is reset after 1000 Iterations

    LOGSLLEVEL1(m_pSolverLog, "---> Do one time-step =================================" <<std::endl <<
                "---> t_S: " << m_currentSimulationTime << std::endl;);


    m_bIterationFinished = false;

    iterations++;
    m_IterationCounter++;


    m_startTime = m_PerformanceTimer.elapsedSec();

    //Force switch
    //boost::thread::yield();

#if SOLVERLOG_LOGLEVEL == 1
     unsigned int when = (m_settings.m_endTime-m_settings.m_startTime)/m_settings.m_deltaT / 10.0;
      if(when<=0){when=1;}
      if(m_IterationCounter % when == 0){
            LOG(m_pSolverLog,"--->  m_t: " << m_currentSimulationTime<<std::endl; );
      }
#endif

    LOGSLLEVEL2(m_pSolverLog,"---> m_t Begin: " << m_currentSimulationTime <<std::endl; );


    //Calculate Midpoint Rule ============================================================
    // Middle Time Step for all LOCAL Bodies==============================================
    // Remote bodies belong to other processes which are timestepped
    m_startSimulationTime = m_currentSimulationTime;
    doFirstHalfTimeStep(m_startSimulationTime, m_settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    doInputTimeStep(m_settings.m_deltaT/2.0);
    // Custom Calculations after first timestep
    afterFirstTimeStep();

    m_currentSimulationTime = m_startSimulationTime + m_settings.m_deltaT/2.0;
    // ====================================================================================

    m_startBodyCommunication = m_PerformanceTimer.elapsedSec();
    m_pBodyCommunicator->communicate(m_currentSimulationTime);
    m_endBodyCommunication = m_PerformanceTimer.elapsedSec();



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


    m_pInclusionSolver->resetForNextTimestep(); // Clears the contact graph and other inclusion related stuff!

    // Solve Collision
    m_startTimeCollisionSolver = m_PerformanceTimer.elapsedSec();
    m_pCollisionSolver->solveCollision();
    m_endTimeCollisionSolver =   m_PerformanceTimer.elapsedSec();

    //Solve Contact Problem
    //boost::thread::yield();
    m_startTimeInclusionSolver = m_PerformanceTimer.elapsedSec();
    m_pInclusionSolver->solveInclusionProblem(m_currentSimulationTime);
    m_endTimeInclusionSolver = m_PerformanceTimer.elapsedSec();



    // ===================================================================================
    // Middle Time Step ==================================================================
    m_currentSimulationTime = m_startSimulationTime + m_settings.m_deltaT ;
    doSecondHalfTimeStep(m_currentSimulationTime, m_settings.m_deltaT/2.0);
    // Custom Integration for Inputs
    doInputTimeStep(m_settings.m_deltaT/2.0);
    // Custom Calculations after second timestep
    afterSecondTimeStep();
    // ====================================================================================




    LOGSLLEVEL3(m_pSolverLog,"---> m_t End: " << m_currentSimulationTime <<std::endl );


    //Force switch
    //boost::thread::yield();

    m_endTime = m_PerformanceTimer.elapsedSec();

    // Measure Time again
    if (m_IterationCounter%100==0) {
        m_AvgTimeForOneIteration=0;
        iterations = 1;
    }
    m_AvgTimeForOneIteration = ( (m_endTime - m_startTime) + m_AvgTimeForOneIteration*(iterations-1)) / iterations;
    if (m_AvgTimeForOneIteration > m_MaxTimeForOneIteration) {
        m_MaxTimeForOneIteration = m_AvgTimeForOneIteration;
    }

    LOGSLLEVEL1( m_pSolverLog,  "---> Iteration Time: "<<std::setprecision(5)<<(m_endTime-m_startTime)<<std::endl
    <<  "---> End time-step ====================================" <<std::endl<<std::endl; );


    // Check if we can finish the timestepping!
    if(m_settings.m_eSimulateFromReference == TimeStepperSettings::USE_STATES ) {
        m_bFinished =  !m_ReferenceSimFile.isGood();
    } else {
        m_bFinished =  m_currentSimulationTime >= m_settings.m_endTime;
    }

    m_bIterationFinished = true;
}

void MoreauTimeStepperMPI::writeIterationToSystemDataFile(double globalTime) {
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
void MoreauTimeStepperMPI::writeHeaderToSystemDataFile() {
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


void MoreauTimeStepperMPI::writeIterationToCollisionDataFile() {
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
