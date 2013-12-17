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
            m_messageContact(this),m_messageMultiplicity(this)
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

//        m_nbRanksSending.reserve(m_nbRanks.size());
//        m_nbRanksReceiving.reserve(m_nbRanks.size());
//        m_nbRanksSending.append

        m_pSimulationLog->logMessage("---> Initialized InclusionCommunicator");
    }

    void setContactGraph(boost::shared_ptr< ContactGraphType> pGraph){
        m_pContactGraph = pGraph;
    }


    ~InclusionCommunicator(){

    }

    void communicateRemoteContacts(){
        LOGIC(m_pSimulationLog,"---> InclusionCommunication: Send remote contacts (initialize global prox)"<< std::endl;)

        m_nbRanksSendRecvLocal.clear();
        m_nbRanksSendRecvRemote.clear();

        // First for each neighbour, communicate the ids of all remote body which have contacts
        sendContactMessageToNeighbours();

        receiveContactMessagesFromNeighbours();

        // Send all multiplicity factors of all local splitBodyNodes
        sendBodyMultiplicityMessageToNeighbours();
        // Recv all multiplicity factor of all remote bodies
        recvBodyMultiplicityMessageFromNeighbours();

        LOGIC(m_pSimulationLog,"---> InclusionCommunication: finished"<< std::endl;)
    }

    void clearNeighbourMap(){
        m_nbDataMap.emptyAllNeighbourData();
    }

    NeighbourMapType * getNeighbourMap(){return &m_nbDataMap;}

private:



    void sendContactMessageToNeighbours(){
        LOGIC(m_pSimulationLog,"MPI>\t Send message (EXTERNALCONTACTS_MESSAGE) to neighbours!"<<std::endl;)

        for(auto it = m_nbRanks.begin(); it != m_nbRanks.end(); it++){
            LOGBC(m_pSimulationLog,"--->\t\t Send contact message to neighbours with rank: "<< *it <<std::endl;)
            // Instanciate a MessageWrapper which contains a boost::serialization function!
            m_messageContact.setRank(*it);
            m_pProcCom->sendMessageToRank(m_messageContact,*it, MPILayer::MPIMessageTag::Type::EXTERNALCONTACTS_MESSAGE );
        }
        LOGBC(m_pSimulationLog,"MPI>\t Send finished!"<<std::endl;)
    }

    void receiveContactMessagesFromNeighbours(){
        LOGIC(m_pSimulationLog,"MPI>\t Receive all messages (EXTERNALCONTACTS_MESSAGE) from neighbours!"<<std::endl;)
        // set the rank of the receiving message automatically! inside the function!
        m_pProcCom->receiveMessageFromRanks(m_messageContact, m_nbRanks, MPILayer::MPIMessageTag::Type::EXTERNALCONTACTS_MESSAGE );
        LOGIC(m_pSimulationLog,"MPI>\t Receive finished!"<<std::endl;)

        // Wait for all sends to complete, Important because we issue a nonblocking send in sendMessagesToNeighbours
        m_pProcCom->waitForAllSends();
    }

    void sendBodyMultiplicityMessageToNeighbours(){
        LOGIC(m_pSimulationLog,"MPI>\t Send message (SPLITBODYFACTOR_MESSAGE) to neighbours!"<<std::endl;)

        for(auto it = m_nbRanksSendRecvLocal.begin(); it != m_nbRanksSendRecvLocal.end(); it++){
            LOGBC(m_pSimulationLog,"--->\t\t Send contact message to neighbours with rank: "<< *it <<std::endl;)
            // Instanciate a MessageWrapper which contains a boost::serialization function!
            m_messageMultiplicity.setRank(*it);
            m_pProcCom->sendMessageToRank(m_messageMultiplicity,*it, MPILayer::MPIMessageTag::Type::SPLITBODYFACTOR_MESSAGE );
        }
        LOGBC(m_pSimulationLog,"MPI>\t Send finished!"<<std::endl;)


    }

    void recvBodyMultiplicityMessageFromNeighbours(){
        LOGIC(m_pSimulationLog,"MPI>\t Receive all messages (SPLITBODYFACTOR_MESSAGE) from neighbours!"<<std::endl;)
        // set the rank of the receiving message automatically! inside the function!
        m_pProcCom->receiveMessageFromRanks(m_messageMultiplicity, m_nbRanksSendRecvRemote, MPILayer::MPIMessageTag::Type::SPLITBODYFACTOR_MESSAGE );
        LOGIC(m_pSimulationLog,"MPI>\t Receive finished!"<<std::endl;)

        // Wait for all sends to complete, Important because we issue a nonblocking send in sendMessagesToNeighbours
        // We dont send to all neighbours but this should not be a problem for the underlying MPI_Wait call
        m_pProcCom->waitForAllSends();
    }



    /**
    * The NeighbourMessageWrapperInclusion class needs access, to be able to serialize all together!
    */
    template<typename TNeighbourCommunicator> friend class MPILayer::NeighbourMessageWrapperInclusionContact;
    template<typename TNeighbourCommunicator> friend class MPILayer::NeighbourMessageWrapperInclusionMultiplicity;

    MPILayer::NeighbourMessageWrapperInclusionContact< InclusionCommunicator > m_messageContact;
    MPILayer::NeighbourMessageWrapperInclusionMultiplicity< InclusionCommunicator > m_messageMultiplicity;

    boost::shared_ptr< DynamicsSystemType >      m_pDynSys;
    boost::shared_ptr< ProcessCommunicatorType > m_pProcCom;
    boost::shared_ptr< ProcessInfoType >         m_pProcInfo;
    boost::shared_ptr< ContactGraphType >        m_pContactGraph;
    boost::shared_ptr< BodyCommunicator>         m_pBodyComm;

    RankIdType m_rank;

    ProcessTopologyType * m_pProcTopo;
    const typename ProcessTopologyType::NeighbourRanksListType & m_nbRanks;

    std::set<RankIdType> m_nbRanksSendRecvRemote;
    /** If rank is in this set then there are remote bodies: send Velocities to rank,
                                                             recv BillateralUpdate from rank*/

    std::set<RankIdType> m_nbRanksSendRecvLocal;
    /** If rank is in this set then there are local bodies: recv Velocities from rank,
                                                            send BillateralUpdate to rank */

    RigidBodyContainerType & m_globalRemote;
    RigidBodyContainerType & m_globalLocal;


    NeighbourMapType m_nbDataMap;   ///< map which gives all neighbour data structures

    Logging::Log *  m_pSimulationLog;

};


#endif
