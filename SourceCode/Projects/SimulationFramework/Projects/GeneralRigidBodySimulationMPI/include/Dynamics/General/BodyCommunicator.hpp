#ifndef NeighbourCommunicator_hpp
#define NeighbourCommunicator_hpp


#include "TypeDefs.hpp"
#include "LogDefines.hpp"


#include <srutil/delegate/delegate.hpp> // Use fast SR delegates
#include <boost/shared_ptr.hpp>



#include "MPIMessages.hpp"
#include "MPICommunication.hpp"

#include "BodyInfoMap.hpp"
#include "NeighbourMap.hpp"
#include "NeighbourDataBodyCommunication.hpp"

class RigidBodyAddRemoveNotificator {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

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
    void addDelegateLocalRemove(const RemoveDelegate & cD) {
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



class BodyCommunicator: public RigidBodyAddRemoveNotificator{;
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    typedef typename MPILayer::ProcessCommunicator                                      ProcessCommunicatorType;
    typedef typename ProcessCommunicatorType::ProcessInfoType                           ProcessInfoType;
    typedef typename ProcessCommunicatorType::ProcessInfoType::ProcessTopologyType      ProcessTopologyType;

    typedef typename DynamicsSystemType::RigidBodySimContainerType                      RigidBodyContainerType;
    typedef typename DynamicsSystemType::GlobalGeometryMapType                          GlobalGeometryMapType;

    typedef NeighbourMap<NeighbourDataBodyCommunication>     NeighbourMapType;

    BodyCommunicator(boost::shared_ptr< DynamicsSystemType> pDynSys ,
                          boost::shared_ptr< ProcessCommunicatorType > pProcCom);

    ~BodyCommunicator(){

    }

    void communicate(PREC currentSimTime);

private:

    /**
    * The NeighbourMessageWrapperBodies class needs access, to be able to serialize all together!
    */
    template<typename TNeighbourCommunicator> friend class MPILayer::NeighbourMessageWrapperBodies;

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

    bool checkReceiveForRemotes();

    void printAllNeighbourRanks();

    PREC m_currentSimTime;
    MPILayer::NeighbourMessageWrapperBodies< BodyCommunicator > m_message;

    boost::shared_ptr< DynamicsSystemType> m_pDynSys;
    boost::shared_ptr< ProcessCommunicatorType > m_pProcCom;
    boost::shared_ptr< ProcessInfoType > m_pProcInfo;


    RankIdType m_rank;

    ProcessTopologyType * m_pProcTopo;
    const typename ProcessTopologyType::NeighbourRanksListType & m_nbRanks;


    RigidBodyContainerType & m_globalRemote;
    RigidBodyContainerType & m_globalLocal;
    GlobalGeometryMapType & m_globalGeometries;

    NeighbourMapType m_nbDataMap;   ///< map which gives all neighbour data structures
    template<typename List>
    void addLocalBodyExclusiveToNeighbourMap(RigidBodyType * body, const List & neighbourRanks);

    std::set< RigidBodyType * > m_localBodiesToDelete;

    Logging::Log *  m_pSimulationLog;

};


BodyCommunicator::BodyCommunicator(  boost::shared_ptr< DynamicsSystemType> pDynSys ,
                                     boost::shared_ptr< ProcessCommunicatorType > pProcCom):
            m_pDynSys(pDynSys),
            m_globalLocal(pDynSys->m_SimBodies),
            m_globalRemote(pDynSys->m_RemoteSimBodies),
            m_globalGeometries(pDynSys->m_globalGeometries),
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
        LOGNC(m_pSimulationLog,"--->BodyCommunicator Add neighbour data for process rank: "<<*rankIt<<std::endl;);
        auto res = m_nbDataMap.insert(*rankIt);
        ASSERTMSG(res.second,"Could not insert in m_nbDataMap for rank: " << *rankIt);
    }
    m_pSimulationLog->logMessage("---> Initialized all NeighbourDatas");

    // Fill in all BodyInfos for the local bodies (remote bodies are not considered, there should not be any of those)
    for(auto it = m_globalLocal.begin(); it != m_globalLocal.end(); it++) {
        ASSERTMSG(m_pProcTopo->belongsBodyToProcess(*it), "Body with id: "<< RigidBodyId::getBodyIdString(*it) <<" does not belong to process? How did you initialize your bodies?")
        (*it)->m_pBodyInfo = new RigidBodyType::BodyInfoType(m_rank);
    }

    // Initialize the buffer in the Process Communicator
    m_pProcCom->initializeBuffers();

    m_pSimulationLog->logMessage("---> Initialized BodyCommunicator");
}


void BodyCommunicator::communicate(PREC currentSimTime){
    LOGNC(m_pSimulationLog,"---> Communicate: Send and Receive message from/to neighbours, t = "<<currentSimTime<< std::endl;)

    m_currentSimTime = currentSimTime;

    // Find all local bodies which overlap
    typename ProcessTopologyType::NeighbourRanksListType neighbours;

    LOGNC(m_pSimulationLog,"--->\t Update neighbour data structures with LOCAL bodies:"<<std::endl;)
    for(typename RigidBodyContainerType::iterator it = m_globalLocal.begin(); it != m_globalLocal.end(); it++) {
        RigidBodyType * body = (*it);


        LOGNC(m_pSimulationLog,"--->\t\t Reset neighbours flags..."<<std::endl;)
        body->m_pBodyInfo->resetNeighbourFlags(); // Reset the overlap flag to false! (addLocalBodyExclusive uses this assumption!)

        //Check overlapping processes
        //TODO (We should return a map of cellNUmbers -> To Rank (any cell which has no rank
        bool overlapsOwnProcess;
        LOGNC(m_pSimulationLog,"--->\t\t Overlap Test..."<<std::endl;)
        bool overlapsNeighbours = m_pProcTopo->checkOverlap(body, neighbours, overlapsOwnProcess);


        LOGASSERTMSG(!(overlapsNeighbours == false && overlapsOwnProcess == false) ,m_pSimulationLog,
                     "Body with id: " << RigidBodyId::getBodyIdString(body) << " does not belong to our process rank: " << m_rank << " nor to any neighbour!")


        // Insert this body into the underlying structure for all nieghbours exclusively! (if no overlap, it is removed everywhere)
        LOGNC(m_pSimulationLog,"--->\t\t Add neighbours exclusively..."<<std::endl;)
        addLocalBodyExclusiveToNeighbourMap(body,neighbours);
//        m_nbDataMap.addLocalBodyExclusive(body,neighbours);

        //Check owner of this body
        typename ProcessInfoType::RankIdType ownerRank;
        m_pProcTopo->belongsBodyToProcess(body,ownerRank);
        //Check if belonging rank is in the neighbours or our own
        if(ownerRank != m_rank){
            if( m_nbRanks.find(ownerRank) == m_nbRanks.end() ){
                LOGNC(m_pSimulationLog,"--->\t Body with id: " << RigidBodyId::getBodyIdString(body) <<" belongs to no neighbour!, "<<"This is not good as we cannot send any message to some other rank other then a neighbour!");
                ERRORMSG("---> Body with id: " << RigidBodyId::getBodyIdString(body) <<" belongs to no neighbour!, "<<"This is not good as we cannot send any message to some other rank other then a neighbour!");
            }
        }
        LOGNC(m_pSimulationLog,"--->\t\t Body with id: " << RigidBodyId::getBodyIdString(body) <<" has owner rank: "<< (ownerRank) << ", proccess rank: " << m_pProcInfo->getRank()<<std::endl;)

        //Set the owning rank for this body:

        body->m_pBodyInfo->m_ownerRank = ownerRank;
        body->m_pBodyInfo->m_overlapsThisRank = overlapsOwnProcess;

        // Status output
//        typename ProcessTopologyType::NeighbourRanksListType::iterator itRank;
//        for(itRank = neighbours.begin(); itRank != neighbours.end(); itRank++) {
//            LOGNC(m_pSimulationLog,"--->\t\t Body with id: " << RigidBodyId::getBodyIdString(body) <<" overlaps Neigbour with Rank: "<< (*itRank) <<std::endl;)
//        }

    }
    LOGNC(m_pSimulationLog,"--->\t Update neighbour data structures complete!"<<std::endl;)

    //printAllNeighbourRanks();
    sendMessagesToNeighbours();
    cleanUp();

    receiveMessagesFromNeighbours();

    if(checkReceiveForRemotes()){
        LOGNC(m_pSimulationLog,"--->\t Update for Remotes: OK"<< std::endl;)
    }else{
        LOGASSERTMSG(false, m_pSimulationLog,"--->\t Update for Remotes: FAILED"<< std::endl)
    }

    LOGNC(m_pSimulationLog,"---> Communicate: finished"<< std::endl;)


}



template<typename List>
void BodyCommunicator::addLocalBodyExclusiveToNeighbourMap(RigidBodyType * body,const List & neighbourRanks)
{
    STATIC_ASSERT( (std::is_same<RankIdType, typename List::value_type>::value) );
    // Add this local body exclusively to the given neighbours

    // Loop over all incoming  ranks
    typename List::const_iterator rankIt;
    for( rankIt = neighbourRanks.begin();rankIt!= neighbourRanks.end();rankIt++){
        // insert the new element into body info --> (rank, flags)
        std::pair<typename BodyProcessInfo::RankToFlagsType::iterator,bool> res =
                   body->m_pBodyInfo->m_neighbourRanks.insert(
                                        typename BodyProcessInfo::RankToFlagsType::value_type(*rankIt,typename RigidBodyType::BodyInfoType::Flags(true))
                                                       );

        res.first->second.m_overlaps = true; // set the Flags for the existing or the newly inserted entry (rank,flags)

        if(res.second){//if inserted we need to add this body to the underlying neighbour data
           //add to the data
           auto pairlocalData = m_nbDataMap.getNeighbourData(*rankIt)->addLocalBodyData(body);
           ASSERTMSG(pairlocalData.second, "Insert to neighbour data rank: " << *rankIt << " in process rank: " <<m_rank << " failed!");
           pairlocalData.first->m_commStatus = NeighbourMapType::DataType::LocalData::SEND_NOTIFICATION; // No need because is set automatically in constructor
        }else{
            ASSERTMSG(m_nbDataMap.getNeighbourData(*rankIt)->getLocalBodyData(body),"body with id "<< RigidBodyId::getBodyIdString(body) << " in neighbour structure rank: " << *rankIt << " does not exist?" );
            ASSERTMSG(m_nbDataMap.getNeighbourData(*rankIt)->getLocalBodyData(body)->m_commStatus ==  NeighbourMapType::DataType::LocalData::SEND_UPDATE,
                      "m_commStatus for body with id: " << RigidBodyId::getBodyIdString(body) << " in neighbour structure rank: " << *rankIt << "should be in update mode!");
        }


    }

    // Clean up of the neighbour structure is done after communication!
    // We cannot already remove the local bodies from the neighbours for
    // which the flag m_overlaps = false because, this needs to be communicated first! (such that any neighbour does not request any update anymore!)
    // So set the flags for this body to SEND_REMOVE for all his neighbours which have m_overlaps = false

    for( typename BodyProcessInfo::RankToFlagsType::iterator rankToFlagsIt = body->m_pBodyInfo->m_neighbourRanks.begin();
        rankToFlagsIt != body->m_pBodyInfo->m_neighbourRanks.end(); rankToFlagsIt++ ){

        if( rankToFlagsIt->second.m_overlaps == false){

            auto * localData = m_nbDataMap.getNeighbourData(rankToFlagsIt->first)->getLocalBodyData(body);

            if(localData->m_commStatus == NeighbourMapType::DataType::LocalData::SEND_UPDATE){
                localData->m_commStatus = NeighbourMapType::DataType::LocalData::SEND_REMOVE;
            }else if (localData->m_commStatus = NeighbourMapType::DataType::LocalData::SEND_NOTIFICATION){
                // Falls notification
            }
        }

    }

}


bool BodyCommunicator::checkReceiveForRemotes(){
    bool m_ok = true;
    for(auto it = m_globalRemote.begin(); it != m_globalRemote.end(); it++){

        LOGASSERTMSG((*it)->m_pBodyInfo , m_pSimulationLog, "bodyInfoPtr is NULL! ");

        if((*it)->m_pBodyInfo->m_receivedUpdate == false ){
            LOGNC(m_pSimulationLog,"---> WARNING: Remote body with id: " << RigidBodyId::getBodyIdString(*it) << " has not received an update!" << std::endl;)
            m_ok = false;
        }else{
           // Set to false for next iteration!
           (*it)->m_pBodyInfo->m_receivedUpdate == false;
        }
    }
    return m_ok;
}



void BodyCommunicator::sendMessagesToNeighbours(){
    LOGNC(m_pSimulationLog,"MPI>\t Send messages to neighbours!"<<std::endl;)
    m_localBodiesToDelete.clear();

    for(typename ProcessTopologyType::NeighbourRanksListType::const_iterator it = m_nbRanks.begin(); it != m_nbRanks.end(); it++){
        LOGNC(m_pSimulationLog,"--->\t\t Send message to neighbours with rank: "<< *it <<std::endl;)
        // Instanciate a MessageWrapper which contains a boost::serialization function!
        m_message.setRank(*it);
        m_pProcCom->sendMessageToRank(m_message,*it, MPILayer::MPIMessageTag::NEIGHBOUR_MESSAGE );
    }
    LOGNC(m_pSimulationLog,"MPI>\t Send finished!"<<std::endl;)
}


void BodyCommunicator::receiveMessagesFromNeighbours(){
    LOGNC(m_pSimulationLog,"MPI>\t Receive all messages from neighbours!"<<std::endl;)
    // set the rank of the receiving message automatically! inside the function!
    m_pProcCom->receiveMessageFromRanks(m_message, m_nbRanks, MPILayer::MPIMessageTag::NEIGHBOUR_MESSAGE );
    LOGNC(m_pSimulationLog,"MPI>\t Receive finished!"<<std::endl;)

    // Wait for all sends to complete, Important because we issue a nonblocking send in sendMessagesToNeighbours
    m_pProcCom->waitForAllSends();
}



void BodyCommunicator::cleanUp(){
    LOGNC(m_pSimulationLog,"--->\t CleanUp Routine " <<std::endl;)
    //Delete all bodies in the list
    for(auto it = m_localBodiesToDelete.begin(); it != m_localBodiesToDelete.end(); it++){
        RigidBodyType * body = *it;


        LOGASSERTMSG((*it)->m_pBodyInfo->m_isRemote == false , m_pSimulationLog , "Local body to delete is not a local body?!" );

        for( auto rankIt = (*it)->m_pBodyInfo->m_neighbourRanks.begin(); rankIt != (*it)->m_pBodyInfo->m_neighbourRanks.end(); rankIt++){
            if( rankIt->second.m_inNeighbourMap == true ){
                bool res = m_nbDataMap.getNeighbourData(rankIt->first)->deleteLocalBodyData(body);
                LOGASSERTMSG( res,  m_pSimulationLog , "This local body with id: " << RigidBodyId::getBodyIdString(body)<< " could not be deleted in neighbour data rank: " << rankIt->first);
            }
        }
        // body tries to delete this also, but does not matter
        delete body->m_pBodyInfo;


        bool res = m_globalLocal.removeAndDeleteBody(body);
        LOGASSERTMSG( res == true, m_pSimulationLog , "Remote Body with id: " << RigidBodyId::getBodyIdString(body)<< " could not be deleted in m_globalRemote!");
        // FROM now it needs to be sure that this body is no where else in the system anymore!

        LOGNC(m_pSimulationLog,"--->\t Deleted body with id: "<< RigidBodyId::getBodyIdString(body) <<std::endl;)
    }

    m_localBodiesToDelete.clear();

}


void BodyCommunicator::printAllNeighbourRanks(){
    for(typename RigidBodyContainerType::iterator it = m_globalLocal.begin(); it != m_globalLocal.end(); it++) {
        RigidBodyType * body = (*it);

        LOGASSERTMSG(body->m_pBodyInfo, m_pSimulationLog , "Body info for local body with id: " << (body)->m_id << " does not exist!");
        LOGASSERTMSG(body->m_pBodyInfo->m_isRemote == false , m_pSimulationLog , "Local body to delete is not a local body?!" );
        LOGNC(m_pSimulationLog,"BodyInfoRanks for body: "<<RigidBodyId::getBodyIdString(body)<< std::endl)
        for( auto ranksIt = body->m_pBodyInfo->m_neighbourRanks.begin(); ranksIt !=  body->m_pBodyInfo->m_neighbourRanks.end();ranksIt++ ){
            LOGNC(m_pSimulationLog, "("<< ranksIt->first << ","<<ranksIt->second.m_overlaps <<"), ")
        }
        LOGNC(m_pSimulationLog, std::endl)
    }
}



#endif
