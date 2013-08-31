#ifndef NeighbourCommunicator_hpp
#define NeighbourCommunicator_hpp


#include "TypeDefs.hpp"
#include "LogDefines.hpp"


#include <srutil/delegate/delegate.hpp> // Use fast SR delegates
#include <boost/shared_ptr.hpp>

#include "RigidBody.hpp"
#include "RigidBodyGarbageCollector.hpp"

#include "NeighbourMap.hpp"
#include "BodyInfoMap.hpp"
#include "MPIMessages.hpp"
#include "MPICommunication.hpp"

template< typename TRigidBody>
class RigidBodyAddRemoveNotificator {
public:

    typedef TRigidBody RigidBodyType;
    typedef typename RigidBodyType::LayoutConfigType LayoutConfigType;
    DEFINE_LAYOUT_CONFIG_TYPES_OF(RigidBodyType::LayoutConfigType)

    RigidBodyAddRemoveNotificator() {
        m_LocalNotifyAddList.clear();
//        m_AddRemoteDelegateList.clear();
        m_LocalNotifyRemoveList.clear();
//        m_RemoveRemoteDelegateList.clear();
    }

#ifdef SRUTIL_DELEGATE_PREFERRED_SYNTAX
    typedef srutil::delegate<void, (RigidBodyType*) > AddDelegate; ///< This is the delegate type which is used, when a new body is added then all delegates are invoked in the list.
#else
    typedef srutil::delegate1<void, RigidBodyType*  > AddDelegate;
#endif

#ifdef SRUTIL_DELEGATE_PREFERRED_SYNTAX
    typedef srutil::delegate<void, (RigidBodyType*) > RemoveDelegate; ///< This is the delegate type which is used, when a body is removed then all delegates are invoked in the list.
#else
    typedef srutil::delegate1<void, RigidBodyType*  > RemoveDelegate;
#endif

    /** Adds a new Delegate for a add notification of a local body*/
    void addDelegateLocalAdd(const AddDelegate & cD) {
        m_LocalNotifyAddList.push_back(cD);
    }
    /** Adds a new Delegate for a remove notifaction of a local body*/
    void addNotificationLocalRemove(const RemoveDelegate & cD) {
        m_LocalNotifyRemoveList.push_back(cD);
    }

protected:

    /** Invokes all delegates for a add notifaction of a local body*/
    void invokeAllAddBodyLocal(RigidBodyType *body) const {
        typename std::vector<AddDelegate>::const_iterator it;
        for(it = m_LocalNotifyAddList.begin(); it != m_LocalNotifyAddList.end(); it++) {
            (*it)(body);
        }
    }
    /** Invokes all delegates for a remove notifaction of a local body*/
    void invokeAllRemoveBodyLocal(RigidBodyType *body) const {
        typename std::vector<RemoveDelegate>::const_iterator it;
        for(it = m_LocalNotifyRemoveList.begin(); it != m_LocalNotifyRemoveList.end(); it++) {
            (*it)(body);
        }
    }

private:

    std::vector<AddDelegate> m_LocalNotifyAddList;
//    std::vector<ContactDelegate> m_AddRemoteDelegateList;
    std::vector<RemoveDelegate> m_LocalNotifyRemoveList;
//    std::vector<ContactDelegate> m_RemoveRemoteDelegateList;
};




template<typename TDynamicsSystem>
class NeighbourCommunicator: public RigidBodyAddRemoveNotificator<typename TDynamicsSystem::RigidBodyType> {
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig              DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef typename MPILayer::ProcessCommunicator<DynamicsSystemType>                  ProcessCommunicatorType;
    typedef typename ProcessCommunicatorType::ProcessInfoType                           ProcessInfoType;
    typedef typename ProcessInfoType::RankIdType                                        RankIdType;
    typedef typename ProcessCommunicatorType::ProcessInfoType::ProcessTopologyType      ProcessTopologyType;

    typedef typename DynamicsSystemType::RigidBodySimContainerType                      RigidBodyContainerType;
    typedef typename DynamicsSystemType::GlobalGeometryMapType                          GlobalGeometryMapType;

    typedef BodyInfoMap<DynamicsSystemType,RankIdType>                                   BodyInfoMapType;
    typedef NeighbourMap<DynamicsSystemType, BodyInfoMapType>                            NeighbourMapType;

    NeighbourCommunicator(typename DynamicsSystemType::RigidBodySimContainerType & globalLocal,
                          typename DynamicsSystemType::RigidBodySimContainerType & globalRemote,
                          typename DynamicsSystemType::GlobalGeometryMapType & globalGeoms,
                          boost::shared_ptr< ProcessCommunicatorType > pProcCom);

    ~NeighbourCommunicator(){

    }

    void communicate(PREC currentSimTime);

private:

    /**
    * The NeighbourMessageWrapper class needs access, to be able to serialize all together!
    */
    template<typename TNeighbourCommunicator> friend class MPILayer::NeighbourMessageWrapper;

    /**
    * @brief Sends a combined message with all info to the neighbour which then extracts it
    */
    void sendMessagesToNeighbours();

    /**
    * @brief Receives all messages in any order
    */
    void receiveMessagesFromNeighbours();

    /**
    * @brief Clean up after the sending!
    */
    void cleanUp();

    PREC m_currentSimTime;
    MPILayer::NeighbourMessageWrapper< NeighbourCommunicator<TDynamicsSystem> > m_message;


    boost::shared_ptr< ProcessCommunicatorType > m_pProcCom;

    boost::shared_ptr< ProcessInfoType > m_pProcInfo;
    RankIdType m_rank;

    ProcessTopologyType * m_pProcTopo;
    const typename ProcessTopologyType::NeighbourRanksListType & m_nbRanks;



    RigidBodyContainerType & m_globalRemote;
    RigidBodyContainerType & m_globalLocal;
    GlobalGeometryMapType & m_globalGeoms;

    BodyInfoMapType m_bodyToInfo; ///< map which gives all overlapping processes to the body

    NeighbourMapType m_nbDataMap;   ///< map which gives all neighbour data structures

    std::set< RigidBodyType * > m_localBodiesToDelete;

    Logging::Log *  m_pSimulationLog;

};

template<typename TDynamicsSystem>
NeighbourCommunicator<TDynamicsSystem>::NeighbourCommunicator(  typename DynamicsSystemType::RigidBodySimContainerType & globalLocal,
                                                                typename DynamicsSystemType::RigidBodySimContainerType & globalRemote,
                                                                typename DynamicsSystemType::GlobalGeometryMapType & globalGeoms,
                                                                boost::shared_ptr< ProcessCommunicatorType > pProcCom):
            m_globalLocal(globalLocal),
            m_globalRemote(globalRemote),
            m_globalGeoms(globalGeoms),
            m_pProcCom(pProcCom),
            m_pProcInfo(m_pProcCom->getProcInfo()),
            m_pProcTopo(m_pProcCom->getProcInfo()->getProcTopo()),
            m_nbDataMap(m_pProcCom->getProcInfo()->getRank(),m_bodyToInfo),
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
    typename ProcessTopologyType::NeighbourRanksListType::const_iterator rankIt;
    for(rankIt = m_nbRanks.begin() ; rankIt != m_nbRanks.end(); rankIt++) {
        LOG(m_pSimulationLog,"---> Add neighbour data for process rank: "<<*rankIt<<std::endl;);
        auto res = m_nbDataMap.insert(*rankIt);
        ASSERTMSG(res.second,"Could not insert in m_nbDataMap for rank: " << *rankIt);
    }
    m_pSimulationLog->logMessage("---> Initialized all NeighbourDatas");

    // Fill in all BodyInfos for the local bodies (remote bodies are not considered, there should not be any of those)
    typename RigidBodyContainerType::iterator it;
    for(it = m_globalLocal.begin(); it != m_globalLocal.end(); it++) {
        ASSERTMSG(m_pProcTopo->belongsBodyToProcess(*it), "Body with id: "<< RigidBodyId::getBodyIdString(*it) <<" does not belong to process? How did you initialize your bodies?")
        auto res = m_bodyToInfo.insert(*it, m_rank);
        ASSERTMSG(res.second,"Could not insert in m_bodyToInfo for rank: " << m_rank);
    }



    m_pSimulationLog->logMessage("---> Initialized NeighbourCommunicator");
}

template<typename TDynamicsSystem>
void NeighbourCommunicator<TDynamicsSystem>::communicate(PREC currentSimTime){

    m_currentSimTime = currentSimTime;

    // Find all local bodies which overlap
    typename ProcessTopologyType::NeighbourRanksListType neighbours;

    LOG(m_pSimulationLog,"---> Communicate: Update neighbour data structures:"<<std::endl;)
    for(typename RigidBodyContainerType::iterator it = m_globalLocal.begin(); it != m_globalLocal.end(); it++) {
        RigidBodyType * body = (*it);

        typename BodyInfoMapType::DataType * bodyInfo = m_bodyToInfo.getBodyInfo(body);
        bodyInfo->resetNeighbourFlags(); // Reset the overlap flag to false! (addLocalBodyExclusive uses this assumption!)

        //Check overlapping processes
        //TODO (We should return a map of cellNUmbers -> To Rank (any cell which has no rank
        bool overlapsOwnProcess;
        LOG(m_pSimulationLog,"---> Communicate: Overlap Test..."<<std::endl;)
        bool overlapsNeighbours = m_pProcTopo->checkOverlap(body, neighbours, overlapsOwnProcess);


        LOGASSERTMSG(!(overlapsNeighbours == false && overlapsOwnProcess == false) ,m_pSimulationLog,
                     "Body with id: " << RigidBodyId::getBodyIdString(body) << " does not belong to our process rank: " << m_rank << " nor to any neighbour!")


        // Insert this body into the underlying structure for all nieghbours exclusively! (if no overlap, it is removed everywhere)
        LOG(m_pSimulationLog,"---> Communicate: Add neighbours exclusively..."<<std::endl;)
        m_nbDataMap.addLocalBodyExclusive(body,neighbours.begin(), neighbours.end());

        //Check owner of this body
        typename ProcessInfoType::RankIdType belongingRank;
        m_pProcTopo->belongsBodyToProcess(body,belongingRank);

        //Check if belonging rank is in the neighbours or our own
        if(belongingRank != m_rank){
            if( m_nbRanks.find(belongingRank) == m_nbRanks.end() ){
                LOG(m_pSimulationLog,"---> Body with id: " << RigidBodyId::getBodyIdString(body) <<" belongs to no neighbour!, "<<
                "This is not good as we cannot send any message to some other rank other then a neighbour!");
                ERRORMSG("---> Body with id: " << RigidBodyId::getBodyIdString(body) <<" belongs to no neighbour!, "<<
                "This is not good as we cannot send any message to some other rank other then a neighbour!");
            }
        }
        LOG(m_pSimulationLog,"---> Body with id: " << RigidBodyId::getBodyIdString(body) <<" has owner rank: "<<
            (belongingRank) << ", proccess rank: " << m_pProcInfo->getRank()<<std::endl;)

        //Set the owning rank for this body:

        bodyInfo->m_ownerRank = belongingRank;
        bodyInfo->m_overlapsThisRank = overlapsOwnProcess;

        if( bodyInfo->m_ownerRank != m_rank){
            LOG(m_pSimulationLog,"--->Communicate: Body with id: " << RigidBodyId::getBodyIdString(body) <<" marked for deletion after send!" <<std::endl;)
            m_localBodiesToDelete.insert(body);
        }

        // Status output
        typename ProcessTopologyType::NeighbourRanksListType::iterator itRank;
        for(itRank = neighbours.begin(); itRank != neighbours.end(); itRank++) {
            LOG(m_pSimulationLog,"--->Communicate: Body with id: " << RigidBodyId::getBodyIdString(body) <<" overlaps Neigbour with Rank: "<< (*itRank) <<std::endl;)
        }
    }
    LOG(m_pSimulationLog,"---> Communicate: Update neighbour data structures complete!"<<std::endl;)


    LOG(m_pSimulationLog,"---> Communicate: Send structure to neighbours!"<<std::endl;)
    sendMessagesToNeighbours();

    cleanUp();

    LOG(m_pSimulationLog,"---> Communicate: Receive all structures from neighbours!"<<std::endl;)
    receiveMessagesFromNeighbours();

}


template<typename TDynamicsSystem>
void NeighbourCommunicator<TDynamicsSystem>::sendMessagesToNeighbours(){

    m_localBodiesToDelete.clear();

    const typename ProcessTopologyType::NeighbourRanksListType & nbRanks = m_pProcTopo->getNeighbourRanks();
    for(typename ProcessTopologyType::NeighbourRanksListType::const_iterator it = nbRanks.begin(); it != nbRanks.end(); it++){
        LOG(m_pSimulationLog,"---> Communicate: Send message to neighbours with rank: "<< *it <<std::endl;)
        // Instanciate a MessageWrapper which contains a boost::serialization function!
        m_message.setRank(*it);
        m_pProcCom->sendMessageToRank(m_message,*it, MPILayer::MPIMessageTag::NEIGHBOUR_MESSAGE );
    }
}

template<typename TDynamicsSystem>
void NeighbourCommunicator<TDynamicsSystem>::receiveMessagesFromNeighbours(){

    const typename ProcessTopologyType::NeighbourRanksListType & nbRanks = m_pProcTopo->getNeighbourRanks();
    // set the rank of from the receiving message automatically! inside the function!
    m_pProcCom->receiveMessageFromRanks(m_message, nbRanks.begin(), nbRanks.end(),  MPILayer::MPIMessageTag::NEIGHBOUR_MESSAGE );

}


template<typename TDynamicsSystem>
void NeighbourCommunicator<TDynamicsSystem>::cleanUp(){

    //Delete all bodies in the list
    for(auto it = m_localBodiesToDelete.begin(); it != m_localBodiesToDelete.end(); it++){
        RigidBodyType * body = body;

        auto bodyInfo = m_bodyToInfo.getBodyInfo(body);
        LOGASSERTMSG(bodyInfo, m_pSimulationLog , "Body info for local body with id: " << (body)->m_id << " does not exist!");
        LOGASSERTMSG(bodyInfo->m_isRemote == false , m_pSimulationLog , "Local body to delete is not a local body?!" );


        for( auto rankIt = bodyInfo->m_neighbourRanks.begin(); rankIt != bodyInfo->m_neighbourRanks.end(); rankIt++){
            LOGASSERTMSG( rankIt->second.m_bOverlaps == true,  m_pSimulationLog , "This local body with id: " << (body)->m_id << " does not overlap rank: " << rankIt->first << " should have been deleted during sending! (REMOVAL)");

            if( rankIt->second.m_bOverlaps == true ){
                bool res = m_nbDataMap.getNeighbourData(rankIt->first)->deleteLocalBodyData(body);
                LOGASSERTMSG( res,  m_pSimulationLog , "This local body with id: " << (body)->m_id << " could not be deleted in neighbour data rank: " << rankIt->first);
            }
            else{
                LOGASSERTMSG(rankIt->second.m_bOverlaps == true, m_pSimulationLog , "This local body with id: " << (body)->m_id << " does not overlap rank: " << rankIt->first << " should have been deleted during sending! (REMOVAL)");
            }
        }


        this->invokeAllRemoveBodyLocal(body);

        bool res = m_globalLocal.removeAndDeleteBody(body);
        LOGASSERTMSG( res == true, m_pSimulationLog , "Remote Body with id: " << (body)->m_id << " could not be deleted in m_globalRemote!");
        // FROM now it needs to be sure that this body is no where else in the system anymore!
    }

    m_localBodiesToDelete.clear();

}


#endif
