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

    NeighbourCommunicator(typename DynamicsSystemType::RigidBodySimContainer & globalLocal,
                          typename DynamicsSystemType::RigidBodySimContainer & globalRemote,
                          boost::shared_ptr< ProcessCommunicatorType > pProcCom)
        : m_globalLocal(globalLocal), m_globalRemote(globalRemote), m_pProcCom(pProcCom),
        m_pProcInfo(m_pProcCom->getProcInfo()), m_pProcTopo(m_pProcCom->getProcInfo()->getProcTopo())

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

        // Fill in all ranks for the local bodies (remote bodies are not considered, there should not be any of those)
        typename RigidBodyContainerType::iterator it;
        typename ProcessInfoType::RankIdType rank = m_pProcCom->getProcInfo()->getRank();
        for(it = m_globalLocal.begin();it != m_globalLocal.end();it++){
            ASSERTMSG(m_pProcTopo->belongsBodyToProcess(*it), "Body with id: "<< (*it)->m_id <<" does not belong to process? How did you initialize your bodies?")
            m_bodyToProcess[(*it)->m_id] = rank;
        }

       m_pSimulationLog->logMessage("---> Initialized NeighbourCommunicator");
    }

    void communicate(){
        // Find all local bodies which overlap
        std::vector<typename ProcessInfoType::RankIdType> neighbours;
        typename std::vector<typename ProcessInfoType::RankIdType>::iterator itRank;

        typename RigidBodyContainerType::iterator it;
        for(it = m_globalLocal.begin();it != m_globalLocal.end();it++){
            RigidBodyType * body = (*it);
            m_pProcTopo->checkOverlap(body, neighbours);
            // If overlap: put into the neighbour data container
            for(itRank = neighbours.begin();itRank != neighbours.end();itRank++){
                LOG(m_pSimulationLog,"Body with id: " << body->m_id <<" overlaps Neigbour with Rank: "<< (*itRank))
                //m_nbDataMap[*itRank].m_localBodies[body->m_id] = body;
            }

        }



    }

private:


    boost::shared_ptr< ProcessCommunicatorType > m_pProcCom;
    boost::shared_ptr< ProcessInfoType > m_pProcInfo;
    ProcessTopologyType * m_pProcTopo;

    RigidBodyContainerType & m_globalRemote;
    RigidBodyContainerType & m_globalLocal;


    NeighbourMap<DynamicsSystemType,typename ProcessInfoType::RankIdType> m_nbDataMap;

    // Map, body to owning process
    std::map<typename RigidBodyType::RigidBodyIdType, typename ProcessInfoType::RankIdType > m_bodyToProcess;

    Logging::Log *  m_pSimulationLog;

};

#endif
