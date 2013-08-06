﻿#ifndef SimulationManagerMPI_hpp
#define SimulationManagerMPI_hpp

#include <mpi.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "SceneParserMPI.hpp"

#include "MPIInformation.hpp"
#include "MPICommunication.hpp"

template <typename TDynamicsSystemType> class StateRecorder;
template <typename TDynamicsSystemType> class StateRecorderBody;


template<typename TConfig>
class SimulationManagerMPI {
public:

    DEFINE_CONFIG_TYPES_OF(TConfig)

    SimulationManagerMPI();
    ~SimulationManagerMPI();

    void setup();
    void setup(boost::filesystem::path sceneFilePath);

    void startSim();

private:

    void initSim();

    unsigned int m_nSimBodies, m_nGlobalSimBodies;


    // File Paths for one Simulation, always reset ==============================
    boost::filesystem::path m_SimFolderPath;

    struct SettingsSimThread {
        double m_EndTime;
    } m_SettingsSimThread;


    Logging::Log *  m_pSimulationLog;

    boost::shared_ptr<StateRecorderBody<DynamicsSystemType> >   m_pStateRecorder;
    boost::shared_ptr< SceneParserMPI<TConfig> > m_pSceneParser;

    boost::shared_ptr< TimeStepperType >	m_pTimestepper;
    boost::shared_ptr< DynamicsSystemType > m_pDynSys;
    boost::shared_ptr<NeighbourCommunicator<DynamicsSystemType> > m_pNbCommunicator;

    typedef typename MPILayer::ProcessCommunicator<LayoutConfigType>::ProcessInfoType ProcessInfoType;
    boost::shared_ptr< MPILayer::ProcessCommunicator<LayoutConfigType> > m_pProcCommunicator;


    bool checkNumberOfBodiesInProcess();
};


// Implementation
#include "SimulationManagerMPI.icc"


#endif // SimulationManagerMPI_HPP
