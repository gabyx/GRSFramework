#ifndef NeighbourCommunicator_hpp
#define NeighbourCommunicator_hpp


#include "TypeDefs.hpp"
#include "LogDefines.hpp"


#include <srutil/delegate/delegate.hpp> // Use fast SR delegates
#include <boost/shared_ptr.hpp>

#include "RigidBody.hpp"
#include "RigidBodyGarbageCollector.hpp"

#include "NeighbourData.hpp"
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
    void addNotificationLocalAdd(const AddDelegate & cD) {
        m_LocalNotifyAddList.push_back(cD);
    }
    /** Adds a new Delegate for a remove notifaction of a local body*/
    void addNotificationLocalRemove(const RemoveDelegate & cD) {
        m_LocalNotifyRemoveList.push_back(cD);
    }

private:

    /** Invokes all delegates for a add notifaction of a local body*/
    void addBodyLocal(RigidBodyType *body) const {
        typename std::vector<AddDelegate>::const_iterator it;
        for(it = m_LocalNotifyAddList.begin(); it != m_LocalNotifyAddList.end(); it++) {
            (*it)(body);
        }
    }
    /** Invokes all delegates for a remove notifaction of a local body*/
    void removeBodyLocal(RigidBodyType *body) const {
        typename std::vector<RemoveDelegate>::const_iterator it;
        for(it = m_LocalNotifyRemoveList.begin(); it != m_LocalNotifyRemoveList.end(); it++) {
            (*it)(body);
        }
    }


    std::vector<AddDelegate> m_LocalNotifyAddList;
//    std::vector<ContactDelegate> m_AddRemoteDelegateList;
    std::vector<RemoveDelegate> m_LocalNotifyRemoveList;
//    std::vector<ContactDelegate> m_RemoveRemoteDelegateList;
};

template<typename TRigidBody, typename TRankId>
class BodyProcessInfo{
public:
        typedef TRankId RankIdType;
        typedef TRigidBody RigidBodyType;

        BodyProcessInfo(RigidBodyType *body,
                        RankIdType ownRank,
                        bool overlapsThisRank = true,
                        bool isRemote = false): m_body(body), m_ownerRank(ownRank), m_overlapsThisRank(true), m_isRemote(isRemote){};
        /**
        * Data structure in the Map: Rank -> Flags, Flags define the behaviour what needs to be done with this Body.
        * m_bToRemove: Used to decide if body is removed from the corresponding neigbourS
        */
        struct Flags{
            Flags():m_bToRemove(true){};
            bool m_bToRemove; ///< Remove flag from this ranks neighbour data
        };

        typedef std::map<RankIdType, Flags> RankToFlagsType;
        RankToFlagsType m_NeighbourRanks;

        RankIdType m_ownerRank;   ///< The process rank to which this body belongs (changes during simulation, if change -> send to other process)
        bool m_overlapsThisRank; ///< True if body overlaps this process!, if false

        /**
        If ownerRank is not this Process and  m_overlapsThisRank = false:
            Send to full body neighbour and delete here
        If ownerRank is not this Process and  m_overlapsThisRank = true:
            Send to full body neighbour and add to remote Bodies to receive updates
        */

        RigidBodyType * m_body;

        bool m_isRemote;
};


template<typename TDynamicsSystem>
class NeighbourCommunicator: public RigidBodyAddRemoveNotificator<typename TDynamicsSystem::RigidBodyType> {
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig              DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef typename MPILayer::ProcessCommunicator<DynamicsSystemType>           ProcessCommunicatorType;
    typedef typename ProcessCommunicatorType::ProcessInfoType                           ProcessInfoType;
    typedef typename ProcessCommunicatorType::ProcessInfoType::ProcessTopologyType      ProcessTopologyType;

    typedef typename DynamicsSystemType::RigidBodySimContainer          RigidBodyContainerType;

    typedef BodyProcessInfo<RigidBodyType, typename ProcessInfoType::RankIdType> BodyProcessInfoType;
    typedef std::map<
                       typename RigidBodyType::RigidBodyIdType,
                       BodyProcessInfoType *
                       > BodyToInfoMapType;

    NeighbourCommunicator(typename DynamicsSystemType::RigidBodySimContainer & globalLocal,
                          typename DynamicsSystemType::RigidBodySimContainer & globalRemote,
                          boost::shared_ptr< ProcessCommunicatorType > pProcCom);

    ~NeighbourCommunicator(){
        //Delete all body infos
        typename BodyToInfoMapType::iterator it;
        for(it=m_bodyToInfo.begin();it != m_bodyToInfo.end();it++){
            delete it->second;
        }
    }

    void communicate();

private:


    boost::shared_ptr< ProcessCommunicatorType > m_pProcCom;
    boost::shared_ptr< ProcessInfoType > m_pProcInfo;
    ProcessTopologyType * m_pProcTopo;

    RigidBodyContainerType & m_globalRemote;
    RigidBodyContainerType & m_globalLocal;

    BodyToInfoMapType m_bodyToInfo; ///< map which gives all overlapping processes to the body

    NeighbourMap<DynamicsSystemType, BodyToInfoMapType> m_nbDataMap;


    Logging::Log *  m_pSimulationLog;

};

template<typename TDynamicsSystem>
NeighbourCommunicator<TDynamicsSystem>::NeighbourCommunicator(  typename DynamicsSystemType::RigidBodySimContainer & globalLocal,
                                                                typename DynamicsSystemType::RigidBodySimContainer & globalRemote,
                                                                boost::shared_ptr< ProcessCommunicatorType > pProcCom):
            m_globalLocal(globalLocal),
            m_globalRemote(globalRemote),
            m_pProcCom(pProcCom),
            m_pProcInfo(m_pProcCom->getProcInfo()),
            m_pProcTopo(m_pProcCom->getProcInfo()->getProcTopo()),
            m_nbDataMap(m_pProcCom->getProcInfo()->getRank(),m_bodyToInfo)
{


    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        ERRORMSG("SimulationLog does not yet exist? Did you create it?")
    }

    // Initialize all NeighbourDatas
    const std::vector<unsigned int> & nbRanks = m_pProcCom->getProcInfo()->getProcTopo()->getNeigbourRanks();
    for(int i=0; i< nbRanks.size(); i++) {
        LOG(m_pSimulationLog,"---> Add neighbour data for process rank: "<<nbRanks[i]<<std::endl;);
        m_nbDataMap.addNewNeighbourData( nbRanks[i]);
    }
    m_pSimulationLog->logMessage("---> Initialized all NeighbourDatas");

    // Fill in all infos for the local bodies (remote bodies are not considered, there should not be any of those)
    typename RigidBodyContainerType::iterator it;
    typename ProcessInfoType::RankIdType rank = m_pProcCom->getProcInfo()->getRank();
    for(it = m_globalLocal.begin(); it != m_globalLocal.end(); it++) {

        ASSERTMSG(m_pProcTopo->belongsBodyToProcess(*it), "Body with id: "<< RigidBodyId::getBodyIdString(*it) <<" does not belong to process? How did you initialize your bodies?")
        m_bodyToInfo.insert(
                            typename BodyToInfoMapType::value_type( (*it)->m_id,
                                                                     new BodyProcessInfoType(*it,rank)
                                                                   )
                            );
    }



    m_pSimulationLog->logMessage("---> Initialized NeighbourCommunicator");
}

template<typename TDynamicsSystem>
void NeighbourCommunicator<TDynamicsSystem>::communicate() {
    // Find all local bodies which overlap
    std::vector<typename ProcessInfoType::RankIdType> neighbours;


    typename RigidBodyContainerType::iterator it;
    LOG(m_pSimulationLog,"---> Communicate: Update neighbour data structures:"<<std::endl;)
    for(it = m_globalLocal.begin(); it != m_globalLocal.end(); it++) {
        RigidBodyType * body = (*it);

        //Check overlapping processes
        bool overlapsOwnProcess;
        LOG(m_pSimulationLog,"---> Communicate: Overlap Test..."<<std::endl;)
        bool overlapsNeighbours = m_pProcTopo->checkOverlap(body, neighbours, overlapsOwnProcess);

        // Insert this body into the underlying structure for all nieghbours exclusively! (if no overlap, it is removed everywhere)
        LOG(m_pSimulationLog,"---> Communicate: Add neighbours exclusively..."<<std::endl;)
        m_nbDataMap.addLocalBodyExclusive(body,neighbours);

        //Check owner of this body
        typename ProcessInfoType::RankIdType belongingRank;
        m_pProcTopo->belongsBodyToProcess(body,belongingRank);
        LOG(m_pSimulationLog,"---> Body with id: " << RigidBodyId::getBodyIdString(body) <<" has owner rank: "<<
            (belongingRank) << ", proccess rank: " << m_pProcInfo->getRank()<<std::endl;)

        //Set the owning rank for this body:
        typename BodyToInfoMapType::iterator bodyInfoIt = m_bodyToInfo.find(body->m_id);
        ASSERTMSG(bodyInfoIt != m_bodyToInfo.end(),"No body info found for body id: " << RigidBodyId::getBodyIdString(body) << " !");
        bodyInfoIt->second->m_ownerRank = belongingRank;
        bodyInfoIt->second->m_overlapsThisRank = overlapsOwnProcess;

        // If overlap: put into the neighbour data container
        typename std::vector<typename ProcessInfoType::RankIdType>::iterator itRank;
        for(itRank = neighbours.begin(); itRank != neighbours.end(); itRank++) {
            LOG(m_pSimulationLog,"---> Body with id: " << RigidBodyId::getBodyIdString(body) <<" overlaps Neigbour with Rank: "<< (*itRank) <<std::endl;)
            //m_nbDataMap[*itRank].m_localBodies[body->m_id] = body;
        }



    }
}

#endif
