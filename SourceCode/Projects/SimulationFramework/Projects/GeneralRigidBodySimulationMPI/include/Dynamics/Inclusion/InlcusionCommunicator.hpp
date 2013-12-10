#ifndef InclusionCommunicator_hpp
#define InclusionCommunicator_hpp

#include "TypeDefs.hpp"

class InclusionCommunicator {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    typedef typename MPILayer::ProcessCommunicator                                      ProcessCommunicatorType;
    typedef typename ProcessCommunicatorType::ProcessInfoType                           ProcessInfoType;
    typedef typename ProcessCommunicatorType::ProcessInfoType::ProcessTopologyType      ProcessTopologyType;

    typedef typename DynamicsSystemType::RigidBodySimContainerType                      RigidBodyContainerType;
    typedef typename DynamicsSystemType::GlobalGeometryMapType                          GlobalGeometryMapType;


    typedef NeighbourMap<NeighbourDataInclusionCommunication> NeighbourMapType;


    BodyCommunicator(boost::shared_ptr< DynamicsSystemType> pDynSys ,
                          boost::shared_ptr< ProcessCommunicatorType > pProcCom);

    ~BodyCommunicator(){

    }

    void communicate(PREC currentSimTime);

private:

    /**
    * The NeighbourMessageWrapperBodies class needs access, to be able to serialize all together!
    */
    template<typename TNeighbourCommunicator> friend class MPILayer::NeighbourMessageWrapperInclusion;


    MPILayer::NeighbourMessageWrapperInclusion< InclusionCommunicator > m_message;

    boost::shared_ptr< DynamicsSystemType> m_pDynSys;
    boost::shared_ptr< ProcessCommunicatorType > m_pProcCom;
    boost::shared_ptr< ProcessInfoType > m_pProcInfo;


    RankIdType m_rank;

    ProcessTopologyType * m_pProcTopo;
    const typename ProcessTopologyType::NeighbourRanksListType & m_nbRanks;


    RigidBodyContainerType & m_globalRemote;
    RigidBodyContainerType & m_globalLocal;


    NeighbourMapType m_nbDataMap;   ///< map which gives all neighbour data structures

    Logging::Log *  m_pSimulationLog;

};


InclusionCommunicator::InclusionCommunicator(  boost::shared_ptr< DynamicsSystemType> pDynSys ,
                                               boost::shared_ptr< ProcessCommunicatorType > pProcCom):
            m_pDynSys(pDynSys),
            m_globalLocal(pDynSys->m_SimBodies),
            m_globalRemote(pDynSys->m_RemoteSimBodies),
            m_pProcCom(pProcCom),
            m_pProcInfo(m_pProcCom->getProcInfo()),
            m_pProcTopo(m_pProcCom->getProcInfo()->getProcTopo()),
            m_nbDataMap(m_pProcCom->getProcInfo()->getRank()),
            m_rank(m_pProcCom->getProcInfo()->getRank()),
            m_nbRanks(m_pProcCom->getProcInfo()->getProcTopo()->getNeighbourRanks()),
            m_message(this)
{


    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        ERRORMSG("SimulationLog does not yet exist? Did you create it?")
    }

    // Initialize all NeighbourDatas
    for(auto rankIt = m_nbRanks.begin() ; rankIt != m_nbRanks.end(); rankIt++) {
        LOGNC(m_pSimulationLog,"--->InclusionCommunicator: Add neighbour data for process rank: "<<*rankIt<<std::endl;);
        auto res = m_nbDataMap.insert(*rankIt);
        ASSERTMSG(res.second,"Could not insert in m_nbDataMap for rank: " << *rankIt);
    }
    m_pSimulationLog->logMessage("--->InclusionCommunicator: Initialized all NeighbourDatas");


    // Initialize the buffer in the Process Communicator
    m_pProcCom->initializeBuffers();

    m_pSimulationLog->logMessage("---> Initialized BodyCommunicator");
}

#endif
