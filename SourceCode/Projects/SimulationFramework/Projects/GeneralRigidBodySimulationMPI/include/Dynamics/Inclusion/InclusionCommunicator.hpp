#ifndef InclusionCommunicator_hpp
#define InclusionCommunicator_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "MPIMessages.hpp"

#include "NeighbourMap.hpp"
#include "NeighbourDataInclusionCommunication.hpp"

class ContactGraph;

class InclusionCommunicator {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    //Cannot template on this type, template cyclic dependency because ContactGraph also template on InlcusionComm
    // Dont template on InclusionCommunicator and ContactGraph
    typedef ContactGraph ContactGraphType;

    typedef typename MPILayer::ProcessCommunicator                                      ProcessCommunicatorType;
    typedef typename ProcessCommunicatorType::ProcessInfoType                           ProcessInfoType;
    typedef typename ProcessCommunicatorType::ProcessInfoType::ProcessTopologyType      ProcessTopologyType;

    typedef typename DynamicsSystemType::RigidBodySimContainerType                      RigidBodyContainerType;
    typedef typename DynamicsSystemType::GlobalGeometryMapType                          GlobalGeometryMapType;


    typedef NeighbourMap<NeighbourDataInclusionCommunication> NeighbourMapType;

    InclusionCommunicator(boost::shared_ptr< BodyCommunicator> pBodyComm,
                          boost::shared_ptr< DynamicsSystemType> pDynSys ,
                          boost::shared_ptr< ProcessCommunicatorType > pProcCom):

            m_pDynSys(pDynSys),
            m_globalLocal(pDynSys->m_SimBodies),
            m_globalRemote(pDynSys->m_RemoteSimBodies),
            m_pProcCom(pProcCom),
            m_pProcInfo(m_pProcCom->getProcInfo()),
            m_pProcTopo(m_pProcCom->getProcInfo()->getProcTopo()),
            m_pBodyComm(pBodyComm),
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
            LOGIC(m_pSimulationLog,"--->InclusionCommunicator: Add neighbour data for process rank: "<<*rankIt<<std::endl;);
            auto res = m_nbDataMap.insert(*rankIt);
            ASSERTMSG(res.second,"Could not insert in m_nbDataMap for rank: " << *rankIt);
        }
        m_pSimulationLog->logMessage("--->InclusionCommunicator: Initialized all NeighbourDatas");

        m_nbRanksSending.reserve(m_nbRanks.size());
        m_nbRanksReceiving.reserve(m_nbRanks.size());
        m_nbRanksSending.append

        m_pSimulationLog->logMessage("---> Initialized InclusionCommunicator");
    }

    void setContactGraph(boost::shared_ptr< ContactGraphType> pGraph){
        m_pContactGraph = pGraph;
    }


    ~InclusionCommunicator(){

    }

    void communicateRemoteContacts(){
        LOGIC(m_pSimulationLog,"---> InclusionCommunication: Send remote contacts (initialize global prox)"<< std::endl;)
        // First for each neighbour, communicate the ids
        sendContactMessageToNeighbours();

        receiveMessagesFromNeighbours();

        LOGIC(m_pSimulationLog,"---> InclusionCommunication: finished"<< std::endl;)
    }

    void clearNeighbourMap(){
        m_nbDataMap.emptyAllNeighbourData();
    }

    NeighbourMapType * getNeighbourMap(){return &m_nbDataMap;}

private:



    void sendContactMessageToNeighbours(){
        LOGIC(m_pSimulationLog,"MPI>\t Send message (CONTACT_MESSAGE) to neighbours!"<<std::endl;)

        for(auto it = m_nbRanks.begin(); it != m_nbRanks.end(); it++){
            LOGBC(m_pSimulationLog,"--->\t\t Send contact message to neighbours with rank: "<< *it <<std::endl;)
            // Instanciate a MessageWrapper which contains a boost::serialization function!
            m_message.setRank(*it);
            m_pProcCom->sendMessageToRank(m_message,*it, MPILayer::MPIMessageTag::Type::CONTACT_MESSAGE );
        }
        LOGBC(m_pSimulationLog,"MPI>\t Send finished!"<<std::endl;)
    }

    void receiveMessagesFromNeighbours(){
        LOGIC(m_pSimulationLog,"MPI>\t Receive all messages (CONTACT_MESSAGE) from neighbours!"<<std::endl;)
        // set the rank of the receiving message automatically! inside the function!
        m_pProcCom->receiveMessageFromRanks(m_message, m_nbRanks, MPILayer::MPIMessageTag::Type::CONTACT_MESSAGE );
        LOGIC(m_pSimulationLog,"MPI>\t Receive finished!"<<std::endl;)

        // Wait for all sends to complete, Important because we issue a nonblocking send in sendMessagesToNeighbours
        m_pProcCom->waitForAllSends();
    }



    /**
    * The NeighbourMessageWrapperInclusion class needs access, to be able to serialize all together!
    */
    template<typename TNeighbourCommunicator> friend class MPILayer::NeighbourMessageWrapperInclusion;


    MPILayer::NeighbourMessageWrapperInclusion< InclusionCommunicator > m_message;

    boost::shared_ptr< DynamicsSystemType >      m_pDynSys;
    boost::shared_ptr< ProcessCommunicatorType > m_pProcCom;
    boost::shared_ptr< ProcessInfoType >         m_pProcInfo;
    boost::shared_ptr< ContactGraphType >        m_pContactGraph;
    boost::shared_ptr< BodyCommunicator>         m_pBodyComm;

    RankIdType m_rank;

    ProcessTopologyType * m_pProcTopo;
    const typename ProcessTopologyType::NeighbourRanksListType & m_nbRanks;

    std::set<RankIdType> m_nbRanksSending;
    std::set<RankIdType> m_nbRanksReceiving;

    RigidBodyContainerType & m_globalRemote;
    RigidBodyContainerType & m_globalLocal;


    NeighbourMapType m_nbDataMap;   ///< map which gives all neighbour data structures

    Logging::Log *  m_pSimulationLog;

};


#endif
