#ifndef GRSF_Dynamics_General_BodyCommunicator_hpp
#define GRSF_Dynamics_General_BodyCommunicator_hpp


#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"


#include <srutil/delegate/delegate.hpp> // Use fast SR delegates
#include <memory>

#include DynamicsSystem_INCLUDE_FILE

#include "GRSF/Dynamics/General/NeighbourMap.hpp"
#include "GRSF/Dynamics/General/NeighbourDataBodyCommunication.hpp"

#include "GRSF/Dynamics/General/MPICommunication.hpp"
#include "GRSF/Dynamics/General/MPIMessages.hpp"

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
        for(it = m_LocalNotifyAddList.begin(); it != m_LocalNotifyAddList.end(); ++it) {
            (*it)(body);
        }
    }
    /** Invokes all delegates for a remove notifaction of a local body*/
    void invokeAllRemoveBodyLocal(RigidBodyType *body) const {
        typename std::vector<RemoveDelegate>::const_iterator it;
        for(it = m_LocalNotifyRemoveList.begin(); it != m_LocalNotifyRemoveList.end(); ++it) {
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

    using ProcessCommunicatorType = MPILayer::ProcessCommunicator                                     ;
    using ProcessInfoType = typename ProcessCommunicatorType::ProcessInfoType                          ;
    using ProcessTopologyType = typename ProcessCommunicatorType::ProcessInfoType::ProcessTopologyType     ;

    using RigidBodyContainerType = typename DynamicsSystemType::RigidBodySimContainerType                     ;
    using GlobalGeometryMapType = typename DynamicsSystemType::GlobalGeometryMapType                         ;

    using NeighbourMapType = NeighbourMap<NeighbourDataBodyCommunication>    ;

    BodyCommunicator(std::shared_ptr< DynamicsSystemType> pDynSys ,
                     std::shared_ptr< ProcessCommunicatorType > pProcComm);

    ~BodyCommunicator(){

    }

    void reset();
    /** Reinitializes all neighbour ranks and data structures */
    void resetTopology();

    void communicate(PREC currentSimTime);

    NeighbourMapType * getNeighbourMap(){return &m_nbDataMap;}

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

    std::shared_ptr< DynamicsSystemType> m_pDynSys;
    std::shared_ptr< ProcessCommunicatorType > m_pProcComm;

    RankIdType m_rank;

    ProcessTopologyType * m_pProcTopo;
    typename ProcessTopologyType::NeighbourRanksListType m_nbRanks;
    typename ProcessTopologyType::NeighbourRanksListType m_nbRanksEmpty; // All neighbours with no simulated bodies

    RigidBodyContainerType & m_globalLocal;
    RigidBodyContainerType & m_globalRemote;
    GlobalGeometryMapType & m_globalGeometries;

    NeighbourMapType m_nbDataMap;   ///< map which gives all neighbour data structures
    template<typename List>
    void addLocalBodyExclusiveToNeighbourMap(RigidBodyType * body, const List & neighbourRanks);

    std::set< RigidBodyType * > m_localBodiesToDelete;

    PREC m_currentSimTime;
    MPILayer::NeighbourMessageWrapperBodies< BodyCommunicator > m_message;

    Logging::Log *  m_pSimulationLog;

};


#endif
