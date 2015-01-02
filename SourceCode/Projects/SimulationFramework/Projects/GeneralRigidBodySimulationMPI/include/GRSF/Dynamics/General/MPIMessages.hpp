#ifndef GRSF_Dynamics_General_MPIMessages_hpp
#define GRSF_Dynamics_General_MPIMessages_hpp

#include <boost/tuple/tuple.hpp>
#include <boost/type_traits.hpp>

#include <boost/filesystem/path.hpp>
#include <boost/serialization/level.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/split_member.hpp>


#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Dynamics/General/MPIMessageTag.hpp"
#include "GRSF/Dynamics/General/MPISerializationHelpersEigen.hpp"
#include "GRSF/Dynamics/Collision/Geometry/MPISerializationHelpersGeometry.hpp"

#include "GRSF/Singeltons/FileManager.hpp"
#include "GRSF/Common/SimpleLogger.hpp"

#include "GRSF/Dynamics/General/QuaternionHelpers.hpp"


#include "GRSF/Dynamics/General/RigidBodyFunctionsMPI.hpp"


namespace boost {
namespace serialization {

template<class Archive >
void serialize(Archive& ar, boost::filesystem::path & p,
               const unsigned int version) {
    boost::filesystem::path::string_type s;
    if(Archive::is_saving::value)
        s = p.string();
    ar & s;
    if(Archive::is_loading::value)
        p = s;
}

};
};

namespace MPILayer {


template<typename... T1>
class GenericMessage {
public:
	using Tuple = std::tuple<T1...>;
    static const std::size_t TupleSize = std::tuple_size<Tuple>::value;

    /** Universal reference: This class can also hold references in the tuple */
    template<typename... T>
    GenericMessage(T&&... args): m_data( std::forward<T>(args)... ){}

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        serialize_imp< 0 >(ar,version);
    }

    template<std::size_t N, class Archive>
    typename std::enable_if< (N < TupleSize-1 )>::type
    serialize_imp(Archive & ar, const unsigned int version) {
        ar & std::get<N>(m_data);
        serialize_imp<N+1, Archive>(ar,version);
    }

    template<std::size_t N, class Archive>
    typename std::enable_if< N == TupleSize-1  >::type
    serialize_imp(Archive & ar, const unsigned int version) {
         ar & std::get<N>(m_data);
    }

    Tuple m_data;
};


template<typename TNeighbourCommunicator >
class NeighbourMessageWrapperBodies : public boost::serialization::traits<  NeighbourMessageWrapperBodies<TNeighbourCommunicator>,
                                                                            boost::serialization::object_serializable,
                                                                            boost::serialization::track_never>
{
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    static const MPIMessageTag m_tag = MPIMessageTag::BODY_MESSAGE;

    using NeighbourCommunicatorType = TNeighbourCommunicator;
    using RigidBodyIdType = typename RigidBodyType::RigidBodyIdType                          ;

    using ProcessCommunicatorType = typename NeighbourCommunicatorType::ProcessCommunicatorType      ;
    using ProcessInfoType = typename NeighbourCommunicatorType::ProcessInfoType              ;
    using RankIdType = typename NeighbourCommunicatorType::RankIdType                   ;
    using ProcessTopologyType = typename NeighbourCommunicatorType::ProcessTopologyType          ;
    using RigidBodyContainerType = typename NeighbourCommunicatorType::RigidBodyContainerType       ;

    using BodyInfoType = typename RigidBodyType::BodyInfoType                             ;

    using NeighbourDataMapType = typename NeighbourCommunicatorType::NeighbourMapType             ;
    using NeighbourDataType = typename NeighbourDataMapType::DataType                          ;

    NeighbourMessageWrapperBodies(NeighbourCommunicatorType * nc):
        m_nc(nc),
        m_initialized(false),
        m_neighbourData(nullptr),
        m_bodyInfo(nullptr) {

        if(Logging::LogManager::getSingleton().existsLog("SimulationLog")) {
            m_pSimulationLog = Logging::LogManager::getSingleton().getLog("SimulationLog");
        } else {
            ERRORMSG("SimulationLog does not yet exist? Did you create it?")
        }

        if(Logging::LogManager::getSingleton().existsLog("MPISerializerLog")) {
            m_pSerializerLog =  Logging::LogManager::getSingleton().getLog("MPISerializerLog");
        } else {
            boost::filesystem::path filePath = FileManager::getSingleton().getLocalDirectoryPath();
            filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
            filePath /= "MPISerializer.log";
            m_pSerializerLog = Logging::LogManager::getSingleton().createLog("MPISerializerLog",false,true,filePath);
        }
    };


    /**
    * Set the rank if we want to reuse this instance and receive another message
    */
    void setRank( RankIdType neigbourRank) {
        m_neighbourRank = neigbourRank;
        if(!m_initialized) {
            m_initialized = true;
        }
    }
    /**
    * Set the flags which are sent to the neighbours
    */
    void setFlags(bool hasNoSimBodies){
        m_isEmpty = hasNoSimBodies;
    }

    enum class SubMessageFlag : char {
        NOTIFICATION = 1 << 0,
        UPDATE = 1 << 1,
        REMOVAL = 1 << 2
    };

    template<class Archive>
    void save(Archive & ar, const unsigned int version) const {


        LOGASSERTMSG( m_initialized, m_pSerializerLog, "The NeighbourMessageWrapperBodies is not correctly initialized, Rank not set!");

        //Message Content:
        /*
         - PREC simulationTime (to check if the message is the correct one
         - unsigned int number of notifications + removes + updates
         - char hasNotifactions / hasRemoves / hasUpdates
         - Update:
            - unsigned int size of body updates
            - UpdatePart for local overlapping Body 1
                - Body Id
                - ownerRank (to check if local or remote)
                - bool overlaps ( false -> remove from NeighbourData )
                  if(ownerRank != receiver && overlap = false){
                    - Remove body (Should not happen! Send remove!)
                  }else{
                    - q
                    - u
                  }
         - Remove:
            - Body Id of local body which should be removed in this neighbour splitting surface
         - Notification:
                - Body Id (to indetify where to overwrite the data if its already in the remote list)
                - RigidBody (full Data)
                     - id
                     - q
                     - u
                     - ...

        */

        // Simulation Time:

        ar & m_nc->m_currentSimTime;

        // Send all flags
        ar & m_isEmpty;

        // Number of submessages to send (for each local body 1)
        m_neighbourData = m_nc->m_nbDataMap.getNeighbourData(m_neighbourRank);
        LOGASSERTMSG( m_neighbourData, m_pSerializerLog, "There exists no NeighbourData for neighbourRank: " << m_neighbourRank << "in process rank: " << m_nc->m_rank << "!");

        unsigned int size = m_neighbourData->sizeLocal();
        ar & size;

        if(size>0) {
            LOGBC_SZ(m_pSerializerLog, "BodyComm=================================================================================="<< std::endl;)
            LOGBC_SZ(m_pSerializerLog, "SERIALIZE Message for neighbour rank: " << m_neighbourRank << std::endl;);
            LOGBC_SZ(m_pSerializerLog, "---> Timestamp: "<<  m_nc->m_currentSimTime << std::endl;);
            LOGBC_SZ(m_pSerializerLog, "---> Size: "<<  size << std::endl;);
        }
        //Loop over all localBodies in this NeighbourData
        int i = 0;

        typename NeighbourDataType::LocalIterator it, it_next;
        for(it = m_neighbourData->localBegin(), it_next = it ; it != m_neighbourData->localEnd(); it = it_next) {
            it_next++; // take care here, because, we might invalidate this iterator it because we might remove local bodies, thats why it_next is used above!

            m_bodyInfo = it->second.m_pBody->m_pBodyInfo;

            SubMessageFlag flag;
            if(it->second.m_commStatus == NeighbourDataType::LocalDataType::SEND_NOTIFICATION) {
                flag = SubMessageFlag::NOTIFICATION;
                ar & flag;
                LOGBC_SZ(m_pSerializerLog, "---> NotifictionSTART: " << i<<std::endl;);
                saveNotificationOrUpdate(ar, it->second.m_pBody, flag );
                LOGBC_SZ(m_pSerializerLog, "---> NotifictionEND: "<<std::endl;);

            } else if (it->second.m_commStatus == NeighbourDataType::LocalDataType::SEND_UPDATE) {
                flag = SubMessageFlag::UPDATE;
                ar & flag;
                LOGBC_SZ(m_pSerializerLog, "---> UpdateSTART: " << i<<std::endl;);
                saveNotificationOrUpdate(ar, it->second.m_pBody, flag);
                LOGBC_SZ(m_pSerializerLog, "---> UpdateEND: "<<std::endl;);

            } else if (it->second.m_commStatus == NeighbourDataType::LocalDataType::SEND_REMOVE) {
                flag = SubMessageFlag::REMOVAL;
                ar & flag;
                LOGBC_SZ(m_pSerializerLog, "---> RemovalSTART: " << i<<std::endl;);
                saveRemoval(ar, it->second.m_pBody);
                LOGBC_SZ(m_pSerializerLog, "---> RemovalEND: "<<std::endl;);
            }

            i++;
        }

        m_initialized = false;


    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version) const {
        LOGASSERTMSG( m_initialized, m_pSerializerLog, "The NeighbourMessageWrapperBodies is not correctly initialized, Rank not set!")

        // Simulation Time:
        PREC simulationTime;
        ar & simulationTime;
        LOGASSERTMSG( m_nc->m_currentSimTime == simulationTime, m_pSerializerLog, "The message from rank: "<< m_neighbourRank << " has timeStamp: " << simulationTime<<" which does not fit our current simulation Time: "<< m_nc->m_currentSimTime << " for rank: " << m_nc->m_rank <<" !")

        // All flags
        bool isEmpty;
        ar & isEmpty;
        if(isEmpty){
            //add rank to the empty list
            m_nc->m_nbRanksEmpty.insert(m_neighbourRank);

        }else{

            // Number of submessages
            unsigned int size;
            ar & size;

            if(size>0) {
                LOGBC_SZ(m_pSerializerLog, "BodyComm=================================================================================="<< std::endl;)
                LOGBC_SZ(m_pSerializerLog, "DESERIALIZE Message from neighbour rank: " << m_neighbourRank << std::endl;);
                LOGBC_SZ(m_pSerializerLog, "---> Timestamp: "<<  simulationTime << std::endl;);
                LOGBC_SZ(m_pSerializerLog, "---> Size: "<<  size << std::endl;);
            }

            m_neighbourData = m_nc->m_nbDataMap.getNeighbourData(m_neighbourRank);
            m_bodyInfo = nullptr;

            LOGASSERTMSG( m_neighbourData, m_pSerializerLog, "There exists no NeighbourData for neighbourRank: " << m_neighbourRank << "in process rank: " << m_nc->m_rank << "!");

            //Loop over all messages
            for(int i = 0; i < size; i++) {
                SubMessageFlag flag;
                ar & flag;
                if(flag == SubMessageFlag::NOTIFICATION) {
                    LOGBC_SZ(m_pSerializerLog, "---> NotifactionSTART: " << i<<std::endl;);
                    loadNotification(ar);
                    LOGBC_SZ(m_pSerializerLog, "---> NotifactionEND: "<<std::endl;);

                } else if (flag == SubMessageFlag::UPDATE) {
                    LOGBC_SZ(m_pSerializerLog, "---> UpdateSTART: " << i<<std::endl;);
                    loadUpdate(ar);
                    LOGBC_SZ(m_pSerializerLog, "---> UpdateEND: "<<std::endl;);

                } else if (flag == SubMessageFlag::REMOVAL) {
                    LOGBC_SZ(m_pSerializerLog, "---> RemovalSTART: " << i<<std::endl;);
                    loadRemoval(ar);
                    LOGBC_SZ(m_pSerializerLog, "---> RemovalEND: "<<std::endl;);
                } else {
                    LOGBC_SZ(m_pSerializerLog, "---> Received WRONG FLAG: "<< (int)flag << std::endl;);
                    LOGASSERTMSG( false, m_pSerializerLog, "Wrong flag received!")
                }
            }
        }

        m_initialized = false;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

private:

    template<class Archive>
    void saveRemoval(Archive & ar, RigidBodyType * body) const {
        // serialize (ONLY LOCAL BODIES)
        // send the id to remove

        ar & body->m_id;
        //ASSERTMSG(body->m_id != RigidBodyIdType(0)," ID zero!");

        LOGBC_SZ(m_pSerializerLog, "-----> body id: " << RigidBodyId::getBodyIdString(body) <<std::endl;);
        // remove local from this neighbour structure!
        bool res = m_neighbourData->eraseLocalBodyData(body);
        LOGASSERTMSG(res, m_pSerializerLog, "Could not delete local body with id: " << RigidBodyId::getBodyIdString(body) << " in neighbour structure rank: " << m_neighbourRank << " !" );

        // remove neighbour from body info!

        // test existence because m_neighbourRanks might have been cleared before! (LOCAL to REMOTE)
        auto it = m_bodyInfo->m_neighbourRanks.find(m_neighbourRank);
        if( it != m_bodyInfo->m_neighbourRanks.end()){
            LOGASSERTMSG(it->second.m_overlaps == false , m_pSerializerLog,
                     "Body with id: " << RigidBodyId::getBodyIdString(body)
                     << " overlaps (m_overlaps = true) neighbour rank: "
                     << m_neighbourRank << " which should not because we have send a removal!" );
        }


        m_bodyInfo->markNeighbourRankToRemove(m_neighbourRank); // Mark this rank to remove!

    }

    template<class Archive>
    void saveNotificationOrUpdate(Archive & ar, RigidBodyType * body, const SubMessageFlag & flag) const {
        // serialize (ONLY LOCAL BODIES)

        // id
        ar & body->m_id;
        LOGBC_SZ(m_pSerializerLog, "-----> body id: " << RigidBodyId::getBodyIdString(body)<<std::endl;);

        // owning rank

        ar & m_bodyInfo->m_ownerRank;
        LOGBC_SZ(m_pSerializerLog, "-----> owning rank: " << m_bodyInfo->m_ownerRank<<std::endl;);

        if(flag == SubMessageFlag::NOTIFICATION) {
            LOGBC_SZ(m_pSerializerLog, "-----> Serialize body (full)... "<< std::endl;);
            serializeBodyFull(ar,body);
            LOGBC_SZ(m_pSerializerLog, "-----> Serialize body (full): finished" << std::endl);
        } else if(flag == SubMessageFlag::UPDATE) {
            LOGBC_SZ(m_pSerializerLog, "-----> Serialize body (update)... "<< std::endl;);
            serializeBodyUpdate(ar,body);
            LOGBC_SZ(m_pSerializerLog, "-----> Serialize body (update): finished " << std::endl;);
        } else {
            ERRORMSG("flag not recognized!")
        }


        bool removeBody = false;
        if( m_bodyInfo->m_ownerRank == m_neighbourRank) { // the body belongs now to m_neighbourRank
            // send a list of all adjacent neighbours where the body overlaps
            // need to know where to send the update next time!
            saveOverlappingCommonNeighbours(ar,body);

            // send extended dynamic stuff (h vector) which is important for the neighbour which overtakes this body!
            if(flag == SubMessageFlag::UPDATE) {
                //LOGBC_SZ(m_pSerializerLog, "-----> Send h_term: " << body->m_h_term << std::endl;);
                serializeAdditionalDynamicsProperties(ar,body);
            }

            // FROM LOCAL to REMOTE!
            if( m_bodyInfo->m_overlapsThisRank) { // if it still overlaps our rank
                //Move local body to remote global list
                LOGBC_SZ(m_pSerializerLog, "-----> Changing LOCAL body to REMOTE in neighbour structure rank: " << m_neighbourRank << std::endl;);
                bool res = m_nc->m_globalRemote.addBody(body);
                LOGASSERTMSG(res, m_pSerializerLog, "Could not add body with id: " << RigidBodyId::getBodyIdString(body->m_id) << "in global remote list")

                res = m_nc->m_globalLocal.removeBody(body);
                LOGASSERTMSG(res, m_pSerializerLog, "Could not remove body with id: " << RigidBodyId::getBodyIdString(body->m_id) << "in global local list")


                // Change m_bodyInfo
                m_bodyInfo->m_isRemote = true;
                m_bodyInfo->m_receivedUpdate = true;
                m_bodyInfo->m_neighbourRanks.clear();
                m_bodyInfo->m_neighbourRanks[m_neighbourRank] = typename BodyInfoType::Flags(true);

                //Add into the neighbour structure!
                res = m_neighbourData->eraseLocalBodyData(body);
                LOGASSERTMSG(res, m_pSerializerLog, "Could not delete body with id: " << RigidBodyId::getBodyIdString(body->m_id) << "in neighbour structure rank: " << m_neighbourRank)
                auto resPair = m_neighbourData->addRemoteBodyData(body);
                LOGASSERTMSG( resPair.second == true, m_pSerializerLog, "Remote Body with id: " << RigidBodyId::getBodyIdString(body->m_id) << " could not be added in neighbour data structure rank: " << m_neighbourRank << "!");


            } else {
                // FROM LOCAL to REMOVE!
                // If it does not overlap anymore, it gets deleted by the delete list in BodyCommunicator
                LOGBC_SZ(m_pSerializerLog,"--->\t\t Body with id: " << RigidBodyId::getBodyIdString(body) <<" marked for deletion after send!" <<std::endl;)
                m_nc->m_localBodiesToDelete.insert(body);

                //Remove this local from this neighbour structure!
                LOGBC_SZ(m_pSerializerLog, "-----> Deleting LOCAL body in neighbour structure rank: " << m_neighbourRank << std::endl;);
                bool res = m_neighbourData->eraseLocalBodyData(body);
                LOGASSERTMSG( res == true, m_pSerializerLog, "Could not delete local body with id: " << RigidBodyId::getBodyIdString(body->m_id) <<"in neighbour structure rank: " << m_neighbourRank << "!");

                m_bodyInfo->markNeighbourRankToRemove(m_neighbourRank); // Mark this rank to remove!
            }

            m_nc->invokeAllRemoveBodyLocal(body);

        } else if(m_bodyInfo->m_ownerRank != m_nc->m_rank) { // if owner rank is not the sending neighbour and  not our rank!
            //Remove this local from this neighbour structure!
            LOGBC_SZ(m_pSerializerLog, "-----> Deleting LOCAL body in neighbour structure rank: " << m_neighbourRank << std::endl;);
            bool res = m_neighbourData->eraseLocalBodyData(body);
            LOGASSERTMSG( res == true, m_pSerializerLog, "Could not delete local body with id: " << RigidBodyId::getBodyIdString(body->m_id) <<"in neighbour structure rank: " << m_neighbourRank << "!");

            //Mark bodyinfo , dont remove this rank (only mark it) because we might send a notification which needs access to the overlap bool!
            m_bodyInfo->markNeighbourRankToRemove(m_neighbourRank);
        } else {
            // if owner rank is m_rank! LOCAL STAYS HERE!
            if(flag == SubMessageFlag::NOTIFICATION) {
                // Change the commStatus!
                m_neighbourData->getLocalBodyData(body)->m_commStatus = NeighbourDataType::LocalDataType::SEND_UPDATE;
            }
        }

    }


    template<class Archive>
    void loadUpdate(Archive & ar) const {
        // deserialize
        typename RigidBodyType::RigidBodyIdType id;
        ar & id;
        LOGBC_SZ(m_pSerializerLog, "-----> body id: " << RigidBodyId::getBodyIdString(id) <<std::endl;);
        //ASSERTMSG(id != RigidBodyIdType(0)," ID zero!");

        RankIdType owningRank;
        ar & owningRank;
        LOGBC_SZ(m_pSerializerLog, "-----> owning rank: " << owningRank<<std::endl;);


        // normal update
        LOGBC_SZ(m_pSerializerLog, "-----> Deserialize body (update)... "<<std::endl;);
        auto * remoteData = m_neighbourData->getRemoteBodyData(id);
        LOGASSERTMSG( remoteData, m_pSerializerLog, "There exists no RemoteData for body with id: " << RigidBodyId::getBodyIdString(id) << " for neighbourRank: " << m_neighbourRank << "in process rank: " << m_nc->m_rank << "!");
        LOGASSERTMSG( m_nc->m_globalRemote.find(id) != m_nc->m_globalRemote.end(), m_pSerializerLog, "m_globalRemote does not contain body with id: " << RigidBodyId::getBodyIdString(id) << " in process rank: " << m_nc->m_rank << "!");

        RigidBodyType * body = remoteData->m_pBody;
        serializeBodyUpdate(ar,body);
        LOGBC_SZ(m_pSerializerLog, "-----> Deserialize body (update): finished "  <<std::endl;);

        // Set flag that we received update
        body->m_pBodyInfo->m_receivedUpdate = true;

        // NEW BODY FROM REMOTE  to  LOCAL!!!
        if(owningRank == m_nc->m_rank) { // if the body is now our local body!
            LOGBC_SZ(m_pSerializerLog, "-----> Changing REMOTE to LOCAL" <<std::endl;);

            std::set<RankIdType> overlappingNeighbours; // these are only the common neighbours between m_neighbourRank and m_nc->m_rank
            ar & overlappingNeighbours; // all ranks where the body overlaps

            serializeAdditionalDynamicsProperties(ar,body);
            //LOGBC_SZ(m_pSerializerLog, "-----> GOT h_term: " << body->m_h_term << std::endl;);

            // Move the remote body to the locale ones and delete in remotes
            m_nc->m_globalRemote.removeBody(body);
            m_nc->m_globalLocal.addBody(body);

            // Make sure this new local( from a remote) is correctly initialized, set velocity of front buffer to zero
            RigidBodyFunctions::remoteToLocalBodyInitialization(body);

            // Move the remote out of the neighbour structure
            // (needs to be in the neighbour structure with rank m_neighbourRank, otherwise this update is somewhat stupid?
            bool res = m_neighbourData->eraseRemoteBodyData(body);
            LOGASSERTMSG( res, m_pSerializerLog, "Body with id: " << RigidBodyId::getBodyIdString(id) << "not deleted in neighbour structure (?)");

            // Change the body info
            body->m_pBodyInfo->m_isRemote = false;
            body->m_pBodyInfo->m_overlapsThisRank = true;
            body->m_pBodyInfo->m_ownerRank = owningRank;

            // Add all neighbours which need updates!
            body->m_pBodyInfo->m_neighbourRanks.clear();
            for(auto it = overlappingNeighbours.begin(); it != overlappingNeighbours.end(); ++it) {
                LOGASSERTMSG(  *it != m_nc->m_rank, m_pSerializerLog, "overlappingNeighbours should not contain own rank: " << m_nc->m_rank);

                if(m_nc->m_nbRanks.find(*it) !=  m_nc->m_nbRanks.end()) { // If rank is neighbour rank
                    body->m_pBodyInfo->m_neighbourRanks[*it] = typename BodyInfoType::Flags(true); // Overlaps this rank!
                } else {
                    ERRORMSG("This rank: " << *it << " is no neighbour of our process rank: " << m_nc->m_rank);
                }

                auto pairlocalData = m_nc->m_nbDataMap.getNeighbourData(*it)->addLocalBodyData(body);
                LOGASSERTMSG( pairlocalData.second, m_pSerializerLog, "Insert to neighbour data rank: " << *it << " in process rank: " << m_nc->m_rank << " failed!");
                // Change comm status!
                pairlocalData.first->m_commStatus = NeighbourDataType::LocalDataType::SEND_UPDATE;
            }

            // Notify all delegates which registered about the new local (StateRecorder)
            LOGBC_SZ(m_pSerializerLog, "-----> Invoke delegates for new LOCAL" <<std::endl;);
            m_nc->invokeAllAddBodyLocal(body);


            // FROM REMOTE to REMOTE
        } else if(owningRank != m_neighbourRank) { // if the body is not owned anymore by the sending process but by another of our neighbours (hoppfully)
            LOGBC_SZ(m_pSerializerLog, "-----> Changing remote body to REMOTE" <<std::endl;);
            LOGASSERTMSG(  m_nc->m_nbRanks.find(owningRank) != m_nc->m_nbRanks.end(), m_pSerializerLog, "Owner Rank: " << owningRank << " for body with id: "
                           <<RigidBodyId::getBodyIdString(id) << " is no neighbour in process rank: " << m_nc->m_rank << "!")

            // Move the remote out of the neighbour structure
            bool res = m_neighbourData->eraseRemoteBodyData(body);
            LOGASSERTMSG( res, m_pSerializerLog, "Body with id: " << RigidBodyId::getBodyIdString(id) << "not deleted in neighbour structure (?)");

            auto pairRes = m_nc->m_nbDataMap.getNeighbourData(owningRank)->addRemoteBodyData(body);
            LOGASSERTMSG( pairRes.second, m_pSerializerLog, "Insertion of body with id: " << RigidBodyId::getBodyIdString(body) << " in neighbour structure rank: " << owningRank << " failed!")

            // Change the body info
            body->m_pBodyInfo->m_isRemote = true; // No need to set! REMOTE BODY!!!!!!!!!!!!!
            body->m_pBodyInfo->m_receivedUpdate = true;
            body->m_pBodyInfo->m_overlapsThisRank = true; // No need to set!
            body->m_pBodyInfo->m_ownerRank = owningRank;
            body->m_pBodyInfo->m_neighbourRanks.clear();
            body->m_pBodyInfo->m_neighbourRanks[owningRank] = typename BodyInfoType::Flags(true);

        }
    }

    template<class Archive>
    void loadNotification(Archive & ar) const {

        //id
        typename RigidBodyType::RigidBodyIdType id;
        ar & id;
        LOGBC_SZ(m_pSerializerLog, "-----> body id: " << RigidBodyId::getBodyIdString(id) <<std::endl;);
        //ASSERTMSG(id != RigidBodyIdType(0)," ID zero!");
        // owner rankk
        RankIdType owningRank;
        ar & owningRank;
        LOGBC_SZ(m_pSerializerLog, "-----> owning rank: " << owningRank<<std::endl;);


        // normal update (make a new body!)
        LOGASSERTMSG( m_nc->m_globalRemote.find(id) == m_nc->m_globalRemote.end(), m_pSerializerLog, "m_globalRemote does contain body with id: " << RigidBodyId::getBodyIdString(id) << " in process rank: " << m_nc->m_rank << "!");
        LOGASSERTMSG( m_nc->m_globalLocal.find(id) == m_nc->m_globalLocal.end(), m_pSerializerLog, "m_globalLocal does contain body with id: " << RigidBodyId::getBodyIdString(id) << " in process rank: " << m_nc->m_rank << "!");

        //MAKE NEW BODY!!!
        RigidBodyType * body = new RigidBodyType(id);

        LOGBC_SZ(m_pSerializerLog, "-----> Deserialize body (full): ... "<< std::endl;);
        serializeBodyFull(ar,body);
        LOGBC_SZ(m_pSerializerLog, "-----> Deserialize body (full): finished " << std::endl;);

        if(owningRank == m_nc->m_rank) {
            // This is a new LOCAL body
            LOGBC_SZ(m_pSerializerLog, "-----> New body as LOCAL" <<std::endl;);
            std::set<RankIdType> overlappingNeighbours; // these are only the common neighbours between m_neighbourRank and m_nc->m_rank
            ar & overlappingNeighbours; // all ranks where the body overlaps

            // add to global Local list!
            m_nc->m_globalLocal.addBody(body);

            // add a new body info
            body->m_pBodyInfo = new BodyInfoType(owningRank,true,false);


            // Add all neighbours which need updates!
            body->m_pBodyInfo->m_neighbourRanks.clear();
            for(auto it = overlappingNeighbours.begin(); it != overlappingNeighbours.end(); ++it) {
                LOGASSERTMSG(  *it != m_nc->m_rank, m_pSerializerLog, "overlappingNeighbours should not contain own rank: " << m_nc->m_rank);
                if(m_nc->m_nbRanks.find(*it) !=  m_nc->m_nbRanks.end()) {
                    body->m_pBodyInfo->m_neighbourRanks[*it] = typename BodyInfoType::Flags(true); // Overlaps this rank!
                } else {
                    ERRORMSG("This rank: " << *it << " is no neighbour of our process rank: " << m_nc->m_rank);
                }
                // Change comm status!
                auto pairlocalData = m_nc->m_nbDataMap.getNeighbourData(*it)->addLocalBodyData(body);
                LOGASSERTMSG( pairlocalData.second, m_pSerializerLog, "Insert to neighbour data rank: " << *it << " in process rank: " << m_nc->m_rank << " failed!");
                pairlocalData.first->m_commStatus = NeighbourDataType::LocalDataType::SEND_UPDATE;
            }


            // Notify all delegates which registered about the new local
            LOGBC_SZ(m_pSerializerLog, "-----> Invoke delegates for new LOCAL" <<std::endl;);
            m_nc->invokeAllAddBodyLocal(body);


        } else {
            // This is a new remote body!
            LOGASSERTMSG(  m_nc->m_nbRanks.find(owningRank) != m_nc->m_nbRanks.end(), m_pSerializerLog, "Owner Rank: " << owningRank << " for body with id: "<<RigidBodyId::getBodyIdString(id) << " is no neighbour in process rank: " << m_nc->m_rank << "!")
            LOGBC_SZ(m_pSerializerLog, "-----> New body as REMOTE" <<std::endl;);
            // add in global
            m_nc->m_globalRemote.addBody(body);

            // add in neighbour structure
            auto pairRes = m_nc->m_nbDataMap.getNeighbourData(owningRank)->addRemoteBodyData(body);
            LOGASSERTMSG( pairRes.second, m_pSerializerLog, "Insertion of body with id: " << RigidBodyId::getBodyIdString(body) << " in neighbour structure rank: " << owningRank << " failed!")

            // add the body info
            body->m_pBodyInfo = new BodyInfoType(owningRank,true,true,true);
            body->m_pBodyInfo->m_neighbourRanks.clear();
            body->m_pBodyInfo->m_neighbourRanks[owningRank] = typename BodyInfoType::Flags(true);

        }
    }

    template<class Archive>
    void loadRemoval(Archive & ar) const {
        // deserialize (ONLY REMOTE BODIES)
        typename RigidBodyType::RigidBodyIdType id;
        ar & id;
        LOGBC_SZ(m_pSerializerLog, "-----> body id: " << RigidBodyId::getBodyIdString(id) <<std::endl;);
        //ASSERTMSG(id != RigidBodyIdType(0)," ID zero!");
        // Go into the neighbour data structure and remove the remote body
        bool res = m_neighbourData->eraseRemoteBodyData(id);
        LOGASSERTMSG(res, m_pSerializerLog, "Could not delete remote body with id: " << RigidBodyId::getBodyIdString(id) << " in neighbour structure rank: " << m_neighbourRank << " !" );

        // Remove and delete from global list, deletes also the body info data
        LOGBC_SZ(m_pSerializerLog, "--->\t Deleting body with id: "<< RigidBodyId::getBodyIdString(id) << std::endl;)
        res = m_nc->m_globalRemote.deleteBody(id);
        LOGASSERTMSG( res == true, m_pSerializerLog, "Remote Body with id: " << RigidBodyId::getBodyIdString(id) << " could not be deleted in m_globalRemote!");


    }

    template<class Archive>
    void serializeBodyUpdate(Archive & ar, RigidBodyType * body) const {
        //Position
        serializeEigen(ar,body->m_r_S);
        serializeEigen(ar,body->m_q_KI.coeffs());
        LOGBC_SZ(m_pSerializerLog, "----->  m_r_S: " << body->m_r_S.transpose()<<std::endl;);
        LOGBC_SZ(m_pSerializerLog, "----->  m_q_KI: " << body->m_q_KI.coeffs().transpose()<<std::endl;);

        if(Archive::is_loading::value) {
           body->m_A_IK =  body->m_q_KI.matrix(); //QuaternionHelpers::setRotFromQuaternion(body->m_q_KI , body->m_A_IK);
        }

        //Velocity
        LOGASSERTMSG( body->m_pSolverData, m_pSerializerLog, "No SolverData present in body with id: "<< RigidBodyId::getBodyIdString(body) << "!");

         if(Archive::is_loading::value) {
            //Reset solver data, we are updating a remote
            body->m_pSolverData->reset();
         }
        serializeEigen(ar,body->m_pSolverData->m_uBuffer.m_back);

        ar & body->m_pSolverData->m_t;
        LOGBC_SZ(m_pSerializerLog, "----->  m_t: " << body->m_pSolverData->m_t <<std::endl;);
        LOGBC_SZ(m_pSerializerLog, "----->  m_uBuffer.m_back: " << body->m_pSolverData->m_uBuffer.m_back.transpose() <<std::endl;);
    }

    template<class Archive>
    void serializeBodyFull(Archive & ar, RigidBodyType * body) const {

        // State
        ar & body->m_eMode;

        //Material id
        ar & body->m_eMaterial;

        //Geometry

        ar & body->m_globalGeomId;
        if( body->m_globalGeomId == 0) {
            //This geometry is not a global one! serialize too!
            serializeGeom(ar,body);
        } else {
            if(Archive::is_loading::value) {
                body->m_geometry = m_nc->m_globalGeometries.find(body->m_globalGeomId)->second;
            }
        }
        LOGBC_SZ(m_pSerializerLog, "-----> global geometry id: " << body->m_globalGeomId <<std::endl;);

        //Position
        serializeEigen(ar,body->m_r_S);
        serializeEigen(ar,body->m_q_KI.coeffs());
        if(Archive::is_loading::value) {
            body->m_A_IK =  body->m_q_KI.matrix();//QuaternionHelpers::setRotFromQuaternion(body->m_q_KI , body->m_A_IK);
        }
        LOGBC_SZ(m_pSerializerLog, "----->  m_r_S: " << body->m_r_S.transpose()<<std::endl;);
        LOGBC_SZ(m_pSerializerLog, "----->  m_q_KI: " << body->m_q_KI.coeffs().transpose()<<std::endl;);

        // Other Dynamics Stuff
        ar & body->m_mass;
        serializeEigen(ar,body->m_K_Theta_S);
        serializeEigen(ar,body->m_MassMatrix_diag);
        serializeEigen(ar,body->m_MassMatrixInv_diag);
        serializeEigen(ar,body->m_h_term);

        if(body->m_eMode == RigidBodyType::BodyMode::SIMULATED) {

            //Velocity
            if(Archive::is_loading::value) {
                if(body->m_pSolverData == nullptr) {
                    body->m_pSolverData = new typename RigidBodyType::RigidBodySolverDataType();
                } else {
                    ERRORMSG("There is a SolverData already present in body with id: " << body->m_id);
                }
            }
            LOGASSERTMSG( body->m_pSolverData, m_pSerializerLog, "There is no SolverData present in body with id: "<< RigidBodyId::getBodyIdString(body) << "! ?");
            serializeEigen(ar,body->m_pSolverData->m_uBuffer.m_back);

            //m_uBegin is initialized when receiving!
            if(Archive::is_loading::value) {
                body->m_pSolverData->m_uBegin = body->m_pSolverData->m_uBuffer.m_back;
            }

            ar & body->m_pSolverData->m_t;
            LOGBC_SZ(m_pSerializerLog, "----->  m_t: " << body->m_pSolverData->m_t <<std::endl;);
            LOGBC_SZ(m_pSerializerLog, "----->  m_uBuffer.m_back: " << body->m_pSolverData->m_uBuffer.m_back.transpose() <<std::endl;);
        } else {
            if(Archive::is_loading::value) {
                ERRORMSG("Deserializing a not simulated body should not happen?");
            } else {
                ERRORMSG("Serializing a not simulated body should not happen?");
            }

        }

    }

    template<class Archive>
    void serializeAdditionalDynamicsProperties(Archive & ar, RigidBodyType * body) const {
        serializeEigen(ar,body->m_h_term);
    }

    template<class Archive>
    void saveOverlappingCommonNeighbours(Archive & ar, RigidBodyType * body) const {
        std::set<RankIdType> overlappingNeighbours; // set of ranks where this body overlaps for m_neighbourRank!
        const typename ProcessTopologyType::NeighbourRanksListType & adjRanks = m_nc->m_pProcTopo->getAdjacentNeighbourRanks(m_neighbourRank);

        LOGBC_SZ(m_pSerializerLog, "-----> overlappingNeigbours: "<<std::endl;);
        for( auto it =  m_bodyInfo->m_neighbourRanks.begin(); it != m_bodyInfo->m_neighbourRanks.end(); ++it) {
            if(it->second.m_overlaps == true && adjRanks.find(it->first) != adjRanks.end() ) {
                // this body overlaps a rank which is adjacent to m_neighbourRank
                overlappingNeighbours.insert(it->first);

                LOGBC_SZ(m_pSerializerLog, "-------> " << it->first <<std::endl;);
            }
        }

        if( m_bodyInfo->m_overlapsThisRank) {
            overlappingNeighbours.insert(m_nc->m_rank);
            LOGBC_SZ(m_pSerializerLog, "-------> own: " << m_nc->m_rank <<std::endl;);
        }

        //serialize the set
        ar & overlappingNeighbours;
    }


    template<class Archive>
    void serializeGeom(Archive & ar, RigidBodyType * body) const {
        // take care this serialization replaces any shared_ptr if body->m_geometry is already filled!
        //ERRORMSG("Serialize Full GEOM")
        LOGBC_SZ(m_pSerializerLog,  "-----> Serializing full geometry!"<<std::endl;)

        GeomSerialization gs(body->m_geometry);
        ar & gs;
    }


    NeighbourCommunicatorType* m_nc;
    RankIdType m_neighbourRank;        ///< This is the neighbour rank where the message is send to or received from!

    bool m_isEmpty;

    mutable NeighbourDataType * m_neighbourData;
    mutable BodyInfoType * m_bodyInfo;

    mutable bool m_initialized;

    Logging::Log *  m_pSerializerLog;
    Logging::Log *  m_pSimulationLog;

};


template<typename TNeighbourCommunicator >
class NeighbourMessageWrapperInclusion {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    using NeighbourCommunicatorType = TNeighbourCommunicator;
    using RankIdType = typename NeighbourCommunicatorType::RankIdType                   ;

    NeighbourMessageWrapperInclusion(NeighbourCommunicatorType * nc):
        m_nc(nc),
        m_initialized(false) {

        if(Logging::LogManager::getSingleton().existsLog("SimulationLog")) {
            m_pSimulationLog = Logging::LogManager::getSingleton().getLog("SimulationLog");
        } else {
            ERRORMSG("SimulationLog does not yet exist? Did you create it?")
        }

        if(Logging::LogManager::getSingleton().existsLog("MPISerializerLog")) {
            m_pSerializerLog =  Logging::LogManager::getSingleton().getLog("MPISerializerLog");
        } else {
            boost::filesystem::path filePath = FileManager::getSingleton().getLocalDirectoryPath();
            filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
            filePath /= "MPISerializer.log";
            m_pSerializerLog = Logging::LogManager::getSingleton().createLog("MPISerializerLog",false,true,filePath);
        }
    };

    /**
    * Set the rank if we want to reuse this instance and receive another message
    */
    void setRank( RankIdType neigbourRank) {
        m_neighbourRank = neigbourRank;
        m_initialized = true;
    }

    void setTime(RigidBodyType::PREC time){
        m_time = time;
    }


protected:

    mutable RigidBodyType::PREC m_time;

    NeighbourCommunicatorType* m_nc;
    RankIdType m_neighbourRank;        ///< This is the neighbour rank where the message is send to or received from!


    mutable bool m_initialized;

    Logging::Log *  m_pSerializerLog;
    Logging::Log *  m_pSimulationLog;

};


/// CHECKED OK
template<typename TNeighbourCommunicator >
class NeighbourMessageWrapperInclusionContact : public NeighbourMessageWrapperInclusion<TNeighbourCommunicator>,
    public boost::serialization::traits< NeighbourMessageWrapperInclusion<TNeighbourCommunicator>,
    boost::serialization::object_serializable,
        boost::serialization::track_never> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    static const MPIMessageTag m_tag = MPIMessageTag::EXTERNALCONTACTS_MESSAGE;

    using NeighbourCommunicatorType = TNeighbourCommunicator;

    using RigidBodyIdType = typename RigidBodyType::RigidBodyIdType                          ;
    using BodyInfoType = typename RigidBodyType::BodyInfoType                             ;

    using ProcessCommunicatorType = typename NeighbourCommunicatorType::ProcessCommunicatorType      ;
    using ProcessInfoType = typename NeighbourCommunicatorType::ProcessInfoType              ;
    using RankIdType = typename NeighbourCommunicatorType::RankIdType                   ;
    using ProcessTopologyType = typename NeighbourCommunicatorType::ProcessTopologyType          ;
    using RigidBodyContainerType = typename NeighbourCommunicatorType::RigidBodyContainerType       ;
    using NeighbourDataMapType = typename NeighbourCommunicatorType::NeighbourMapType             ;
    using ContactGraphType = typename NeighbourCommunicatorType::ContactGraphType             ;

    using NeighbourDataType = typename NeighbourDataMapType::DataType                          ;


    NeighbourMessageWrapperInclusionContact(NeighbourCommunicatorType * nc):
        NeighbourMessageWrapperInclusion<NeighbourCommunicatorType>(nc),
        m_neighbourData(nullptr)
    {}

    //For remote-remote contacts
    enum class SubMessageFlag : char {
        SPLITBODYMESSAGE = 1 << 0,
        REMOTENODEMESSAGE = 1 << 1,
    };

    template<class Archive>
    void save(Archive & ar, const unsigned int version) const {

        LOGASSERTMSG( this->m_initialized, this->m_pSerializerLog, "The NeighbourMessageWrapperInclusionContact is not correctly initialized, Rank not set!");

        //Serialize all body ids which have contact
        m_neighbourData = this->m_nc->m_nbDataMap.getNeighbourData(this->m_neighbourRank);
        LOGASSERTMSG( m_neighbourData, this->m_pSerializerLog, "There exists no NeighbourData for neighbourRank: " << this->m_neighbourRank << "in process rank: " << this->m_nc->m_rank << "!");

        ar & this->m_time; //NOT_ESSENTIAL

        //Serialize all remote body ids which have contact
        unsigned int size = m_neighbourData->sizeRemote();

        ar & size;

        if(size>0) {
            LOGIC_SZ(this->m_pSerializerLog, "InclusionComm-Contact=============================================================================="<< std::endl;)
            LOGIC_SZ(this->m_pSerializerLog, "SERIALIZE Message for neighbour rank: " << this->m_neighbourRank << std::endl;);

            LOGIC_SZ(this->m_pSerializerLog, "---> # Remote Bodies (with Contacts): " << size << std::endl;);
            for(auto it = m_neighbourData->remoteBegin(); it != m_neighbourData->remoteEnd(); ++it) {
                ar & (it->first); //m_id, all these bodies have remote contacts
                LOGIC_SZ(this->m_pSerializerLog, "---->  body id: " << RigidBodyId::getBodyIdString((it->first)) << std::endl;);
            }

            // Add this rank to the sending list for further communication
            this->m_nc->m_nbRanksSendRecvRemote.insert( this->m_neighbourRank );
        }

        this->m_initialized = false;
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version) const {
        LOGASSERTMSG( this->m_initialized, this->m_pSerializerLog, "The NeighbourMessageWrapperInclusion is not correctly initialized, Rank not set!")

        RigidBodyType::PREC time;
        ar & time;
        LOGASSERTMSG(time == this->m_time, this->m_pSerializerLog, "Wrong message received! message time: " << time << ", current time: " << this->m_time)

        // for each body one bilateral node
        // for each received body , if no node in the bilateral set in ContactGraph exists , add one bilateral node
        // with this participating rank;
        unsigned int size;
        ar & size;

        if(size>0) {
            LOGIC_SZ(this->m_pSerializerLog, "InclusionComm-Contact=============================================================================="<< std::endl;)
            LOGIC_SZ(this->m_pSerializerLog, "DESERIALIZE Message for neighbour rank: " << this->m_neighbourRank << std::endl;);

            LOGIC_SZ(this->m_pSerializerLog, "---> # Remote Bodies (with Contacts): " << size << std::endl;);

            // Update m_neighbourData;
            m_neighbourData = this->m_nc->m_nbDataMap.getNeighbourData(this->m_neighbourRank);

            for(unsigned int i = 0; i < size ; i++) {
                RigidBodyIdType id;
                ar & id; // get all unique ids!
                LOGIC_SZ(this->m_pSerializerLog, "----> id: " << RigidBodyId::getBodyIdString(id) << std::endl;);

                auto * localData = this->m_nc->m_pBodyComm->getNeighbourMap()->getNeighbourData(this->m_neighbourRank)->getLocalBodyData(id);
                LOGASSERTMSG(localData, this->m_pSerializerLog, "There is no bodydata for local body id: " << RigidBodyId::getBodyIdString(id)
                             << " in body communicators neighbour data for rank:" << this->m_neighbourRank)

                RigidBodyType * body = localData->m_pBody;
                //add a local bodydata which connects to the billateral constraint
                LOGASSERTMSG(body, this->m_pSerializerLog,"Local body pointer null for id: "
                             << RigidBodyId::getBodyIdString(id) << " in body communicators neighbour data for rank:" << this->m_neighbourRank)
                auto pairAddLocal = m_neighbourData->addLocalBodyData(body);
                LOGASSERTMSG(pairAddLocal.second, this->m_pSerializerLog, "Could not add body with id: "
                             << RigidBodyId::getBodyIdString(id) << " to neighbour data, already added!")

                auto pairRes = this->m_nc->m_pContactGraph->addSplitBodyNode(body,this->m_neighbourRank);
                //first: SplitBodyNode * pointer / second: bool

                //Connect local body data (for sending and receiving updates later) to this splitBodyNode
                pairAddLocal.first->m_pSplitBodyNode = pairRes.first;
                LOGIC_SZ(this->m_pSerializerLog, "----> added rank: " << this->m_neighbourRank << " to SplitBodyNode for body id: "<< RigidBodyId::getBodyIdString(id) << std::endl);

                // Add this rank to the receving list for further communication
                this->m_nc->m_nbRanksSendRecvLocal.insert( this->m_neighbourRank );

            }
        }


        this->m_initialized = false;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

private:

    mutable NeighbourDataType * m_neighbourData;

};



template<typename TNeighbourCommunicator >
class NeighbourMessageWrapperInclusionMultiplicity : public NeighbourMessageWrapperInclusion<TNeighbourCommunicator>,
    public boost::serialization::traits< NeighbourMessageWrapperInclusion<TNeighbourCommunicator>,
    boost::serialization::object_serializable,
        boost::serialization::track_never> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    static const MPIMessageTag m_tag = MPIMessageTag::SPLITBODYFACTOR_MESSAGE;

    using NeighbourCommunicatorType = TNeighbourCommunicator;
    using RankIdType = typename NeighbourCommunicatorType::RankIdType                   ;

    using RigidBodyIdType = typename RigidBodyType::RigidBodyIdType                          ;
    using BodyInfoType = typename RigidBodyType::BodyInfoType                             ;

    using ProcessCommunicatorType = typename NeighbourCommunicatorType::ProcessCommunicatorType      ;
    using ProcessInfoType = typename NeighbourCommunicatorType::ProcessInfoType              ;
    using ProcessTopologyType = typename NeighbourCommunicatorType::ProcessTopologyType          ;
    using RigidBodyContainerType = typename NeighbourCommunicatorType::RigidBodyContainerType       ;
    using NeighbourDataMapType = typename NeighbourCommunicatorType::NeighbourMapType             ;
    using ContactGraphType = typename NeighbourCommunicatorType::ContactGraphType             ;

    using NeighbourDataType = typename NeighbourDataMapType::DataType                          ;


    NeighbourMessageWrapperInclusionMultiplicity(NeighbourCommunicatorType * nc):
        NeighbourMessageWrapperInclusion<NeighbourCommunicatorType>(nc),
        m_neighbourData(nullptr)
    {}


    template<class Archive>
    void save(Archive & ar, const unsigned int version) const {
        // LOCAL BODIES
        LOGASSERTMSG( this->m_initialized, this->m_pSerializerLog, "The NeighbourMessageWrapperInclusionMultiplicity is not correctly initialized, Rank not set!");


        m_neighbourData = this->m_nc->m_nbDataMap.getNeighbourData(this->m_neighbourRank);
        LOGASSERTMSG( this->m_neighbourData, this->m_pSerializerLog,
                      "There exists no NeighbourData for neighbourRank: " << this->m_neighbourRank << "in process rank: "
                      << this->m_nc->m_rank << "!");

        ar & this->m_time;

        //Serialize all local body mulitplicities which have contact
        unsigned int size = m_neighbourData->sizeLocal();

        ar & size;

        if(size>0) {
            LOGIC_SZ(this->m_pSerializerLog, "InclusionComm-Multiplicity=============================================================================="<< std::endl;)
            LOGIC_SZ(this->m_pSerializerLog, "SERIALIZE Message for neighbour rank: " << this->m_neighbourRank << std::endl;);

            LOGIC_SZ(this->m_pSerializerLog, "---> # Local Split Bodies (with external Contacts): " << size << std::endl;);

            auto itEnd = m_neighbourData->localEnd();
            for(auto it = m_neighbourData->localBegin(); it != itEnd; ++it) {
                LOGASSERTMSG(it->second.m_pSplitBodyNode, this->m_pSerializerLog, "m_pSplitBodyNode is null for body id: "
                             << RigidBodyId::getBodyIdString(it->first) <<std::endl)

                unsigned int multiplicity;
                PREC multiplicityWeight;
                it->second.m_pSplitBodyNode->getMultiplicityAndWeight(this->m_neighbourRank, multiplicity, multiplicityWeight );

                LOGIC_SZ(this->m_pSerializerLog, "----> id: " << RigidBodyId::getBodyIdString((it->first)) << std::endl <<
                      "----> multiplicity: " << multiplicity <<std::endl <<
                      "----> multiplicityWeight: " <<multiplicityWeight<<std::endl <<
                      "----> h_term: " << it->second.m_pBody->m_h_term.transpose() <<std::endl;)
                ar & (it->first);  // body id
                ar & multiplicity; // multiplicity
                ar & multiplicityWeight; //weight
                serializeEigen(ar,it->second.m_pBody->m_h_term); // send the current h_term , mass matrix is already in the remote on the neighbour
            }
        } else {
            ERRORMSG("We should send a message to neighbour " << this->m_neighbourRank << " which is non empty! This should not happen!")
            // We know to which neighbours we send receive a nonempty message
        }

        this->m_initialized = false;
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version) const {
        LOGASSERTMSG( this->m_initialized, this->m_pSerializerLog, "The NeighbourMessageWrapperInclusionMultiplicity is not correctly initialized, Rank not set!")

        RigidBodyType::PREC time;
        ar & time;
        LOGASSERTMSG(time == this->m_time, this->m_pSerializerLog, "Wrong message received!")


        // REMOTE BODIES
        unsigned int size;
        ar & size;

        if(size>0) {
            LOGIC_SZ(this->m_pSerializerLog, "InclusionComm-Multiplicity=============================================================================="<< std::endl;)
            LOGIC_SZ(this->m_pSerializerLog, "DESERIALIZE Message for neighbour rank: " << this->m_neighbourRank << std::endl;);

            LOGIC_SZ(this->m_pSerializerLog, "---> # Remote SplitBodies: " << size << std::endl;);

            // Update m_neighbourData;
            m_neighbourData = this->m_nc->m_nbDataMap.getNeighbourData(this->m_neighbourRank);

            unsigned int multiplicity;
            PREC multiplicityWeight;
            RigidBodyIdType id;
            static VectorUBody h_term;
            for(unsigned int i = 0; i < size ; i++) {
                ar & id;
                ar & multiplicity;
                ar & multiplicityWeight;
                serializeEigen(ar,h_term);

                // Save this multfactor in the REMOTE rigid body
                LOGASSERTMSG(multiplicity != 0, this->m_pSerializerLog, "multiplicity can not be zero!" )
                auto * remoteBodyData = m_neighbourData->getRemoteBodyData(id);

                LOGASSERTMSG( remoteBodyData, this->m_pSerializerLog,"remoteBodyData is null for body id: " << RigidBodyId::getBodyIdString(id) << " in neighbour data rank " << this->m_neighbourRank)
                LOGASSERTMSG( remoteBodyData->m_pBody->m_pSolverData , this->m_pSerializerLog,"m_pSolverData is null for body id: " << RigidBodyId::getBodyIdString(id));



                RigidBodyType * remoteBody = remoteBodyData->m_pBody;

                LOGASSERTMSG(remoteBody->m_pBodyInfo,this->m_pSerializerLog,"bodyInfo is null for body id: "
                             << RigidBodyId::getBodyIdString(id) << " in neighbour data rank " << this->m_neighbourRank);
                LOGASSERTMSG(remoteBody->m_pBodyInfo->m_isRemote,this->m_pSerializerLog,
                             "Body id: " << RigidBodyId::getBodyIdString(id) << " in neighbour data rank "
                             << this->m_neighbourRank << " is not remote!");


                //Set current h_term;
                remoteBody->m_h_term = h_term;

                LOGIC_SZ(this->m_pSerializerLog, "----> id: " << RigidBodyId::getBodyIdString(id) << std::endl <<
                      "----> multiplicity: " << multiplicity <<std::endl <<
                      "----> multiplicityWeight: " <<multiplicityWeight<<std::endl<<
                      "----> h_term current: " << h_term.transpose() << std::endl;  );

                // Apply weighting factors
                // Turn this remote into a split body
                // Scale the inverse mass matrix, and h_term
                LOGIC_SZ(this->m_pSerializerLog, "----> change weighting for remote body" <<std::endl;);
                RigidBodyFunctions::changeBodyToSplitWeighting( remoteBody, multiplicity, multiplicityWeight);


            }

        } else {
            ERRORMSG("We should send a message to neighbour " << this->m_neighbourRank << " which is non empty! This should not happen!")
            // We know from which neighbours we should receive a nonempty message
        }


        this->m_initialized = false;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

private:
    mutable NeighbourDataType * m_neighbourData;
};



template<typename TNeighbourCommunicator >
class NeighbourMessageWrapperInclusionSplitBodyUpdate : public NeighbourMessageWrapperInclusion<TNeighbourCommunicator>,
    public boost::serialization::traits< NeighbourMessageWrapperInclusion<TNeighbourCommunicator>,
    boost::serialization::object_serializable,
        boost::serialization::track_never> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    static const MPIMessageTag m_tag = MPIMessageTag::SPLITBODYUPDATE_MESSAGE;

    using NeighbourCommunicatorType = TNeighbourCommunicator;
    using RankIdType = typename NeighbourCommunicatorType::RankIdType                   ;

    using RigidBodyIdType = typename RigidBodyType::RigidBodyIdType                          ;
    using BodyInfoType = typename RigidBodyType::BodyInfoType                             ;

    using ProcessCommunicatorType = typename NeighbourCommunicatorType::ProcessCommunicatorType      ;
    using ProcessInfoType = typename NeighbourCommunicatorType::ProcessInfoType              ;
    using ProcessTopologyType = typename NeighbourCommunicatorType::ProcessTopologyType          ;
    using RigidBodyContainerType = typename NeighbourCommunicatorType::RigidBodyContainerType       ;
    using NeighbourDataMapType = typename NeighbourCommunicatorType::NeighbourMapType             ;
    using ContactGraphType = typename NeighbourCommunicatorType::ContactGraphType             ;

    using NeighbourDataType = typename NeighbourDataMapType::DataType                          ;


    NeighbourMessageWrapperInclusionSplitBodyUpdate(NeighbourCommunicatorType * nc):
        NeighbourMessageWrapperInclusion<NeighbourCommunicatorType>(nc),
        m_neighbourData(nullptr)
    {}

    void setStep(unsigned int step){
        m_step = step;
    }

    template<class Archive>
    void save(Archive & ar, const unsigned int version) const {
        // Remote SplitBodies
        LOGASSERTMSG( this->m_initialized, this->m_pSerializerLog, "The NeighbourMessageWrapperInclusionSplitBodyUpdate is not correctly initialized, Rank not set!");


        m_neighbourData = this->m_nc->m_nbDataMap.getNeighbourData(this->m_neighbourRank);
        LOGASSERTMSG( this->m_neighbourData, this->m_pSerializerLog,
                      "There exists no NeighbourData for neighbourRank: " << this->m_neighbourRank << "in process rank: "
                      << this->m_nc->m_rank << "!");


        ar & this->m_time;
        ar & m_step;

        //Serialize all velocities of all remote SplitBodies
        unsigned int size = m_neighbourData->sizeRemote();
        ar & size;

        if(size>0) {
            LOGIC_SZ(this->m_pSerializerLog, "InclusionComm-SplitBodyUpdate=============================================================================="<< std::endl;)
            LOGIC_SZ(this->m_pSerializerLog, "SERIALIZE Message for neighbour rank: " << this->m_neighbourRank << std::endl;);

            LOGIC_SZ(this->m_pSerializerLog, "---> # Remote SplitBody Updates: " << size << std::endl;);

            for(auto it = m_neighbourData->remoteBegin(); it != m_neighbourData->remoteEnd(); it++){
                ar & it->first; // id

                LOGASSERTMSG( it->second.m_pBody->m_pSolverData, this->m_pSerializerLog,
                             "SolverData for body id: " << RigidBodyId::getBodyIdString(it->first) << "zero!");

                serializeEigen(ar,it->second.m_pBody->m_pSolverData->m_uBuffer.m_front); // front velocity

                LOGIC_SZ(this->m_pSerializerLog, "----> id: " << RigidBodyId::getBodyIdString((it->first)) << std::endl <<
                          "----> uFront: " << it->second.m_pBody->m_pSolverData->m_uBuffer.m_front.transpose() <<std::endl)
            }
        }
        else{
            ERRORMSG("We should send a message to neighbour " << this->m_neighbourRank << " which is non empty! This should not happen!")
            // We know from which neighbours we should receive a nonempty message
        }

        this->m_initialized = false;
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version) const {
        LOGASSERTMSG( this->m_initialized, this->m_pSerializerLog, "The NeighbourMessageWrapperInclusionSplitBodyUpdate is not correctly initialized, Rank not set!")
        //Local SplitBodyUpdates

        m_neighbourData = this->m_nc->m_nbDataMap.getNeighbourData(this->m_neighbourRank);
        LOGASSERTMSG( this->m_neighbourData, this->m_pSerializerLog,
                      "There exists no NeighbourData for neighbourRank: " << this->m_neighbourRank << "in process rank: "
                      << this->m_nc->m_rank << "!");

        unsigned int step;
        RigidBodyType::PREC time;
        ar & time;
        ar & step;
        LOGASSERTMSG(time == this->m_time && step == m_step, this->m_pSerializerLog, "Wrong message received!")

        unsigned int size;
        ar & size;

        if(size>0) {
            LOGIC_SZ(this->m_pSerializerLog, "InclusionComm-SplitBodyUpdate=============================================================================="<< std::endl;)
            LOGIC_SZ(this->m_pSerializerLog, "DESERIALIZE Message for neighbour rank: " << this->m_neighbourRank << std::endl;);

            LOGIC_SZ(this->m_pSerializerLog, "---> # Local SplitBody Updates: " << size << std::endl;);

            RigidBodyIdType id;
            VectorUBody u;
            for(unsigned int i = 0; i < size ; i++) {
                ar & id;
                serializeEigen(ar,u);

                auto * localBodyData = m_neighbourData->getLocalBodyData(id);

                LOGASSERTMSG(localBodyData, this->m_pSerializerLog, "The local split body id: "
                             << RigidBodyId::getBodyIdString(id) << "is not in this neighbour rank: " << this->m_neighbourRank)

                localBodyData->m_pSplitBodyNode->updateVelocity(this->m_neighbourRank,u);
                LOGIC_SZ(this->m_pSerializerLog, "----> id: " << RigidBodyId::getBodyIdString(id) << std::endl <<
                          "----> uFront: " << u.transpose() <<std::endl)
            }
        }
        else{
            ERRORMSG("We should not receive an empty message from neighbour " << this->m_neighbourRank << std::endl);
        }

        this->m_initialized = false;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

private:
    mutable unsigned int m_step;
    mutable NeighbourDataType * m_neighbourData;
};


template<typename TNeighbourCommunicator >
class NeighbourMessageWrapperInclusionSplitBodySolution : public NeighbourMessageWrapperInclusion<TNeighbourCommunicator>,
    public boost::serialization::traits< NeighbourMessageWrapperInclusion<TNeighbourCommunicator>,
    boost::serialization::object_serializable,
        boost::serialization::track_never> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    static const MPIMessageTag m_tag = MPIMessageTag::SPLITBODYSOLUTION_MESSAGE;

    using NeighbourCommunicatorType = TNeighbourCommunicator;
    using RankIdType = typename NeighbourCommunicatorType::RankIdType                   ;

    using RigidBodyIdType = typename RigidBodyType::RigidBodyIdType                          ;
    using BodyInfoType = typename RigidBodyType::BodyInfoType                             ;

    using ProcessCommunicatorType = typename NeighbourCommunicatorType::ProcessCommunicatorType      ;
    using ProcessInfoType = typename NeighbourCommunicatorType::ProcessInfoType              ;
    using ProcessTopologyType = typename NeighbourCommunicatorType::ProcessTopologyType          ;
    using RigidBodyContainerType = typename NeighbourCommunicatorType::RigidBodyContainerType       ;
    using NeighbourDataMapType = typename NeighbourCommunicatorType::NeighbourMapType             ;
    using ContactGraphType = typename NeighbourCommunicatorType::ContactGraphType             ;

    using NeighbourDataType = typename NeighbourDataMapType::DataType                          ;


    NeighbourMessageWrapperInclusionSplitBodySolution(NeighbourCommunicatorType * nc):
        NeighbourMessageWrapperInclusion<NeighbourCommunicatorType>(nc),
        m_neighbourData(nullptr)
    {}


    void setStep(unsigned int step){
        m_step = step;
    }

    template<class Archive>
    void save(Archive & ar, const unsigned int version) const {
        // LOCAL SPLITBODIES
        LOGASSERTMSG( this->m_initialized, this->m_pSerializerLog, "The NeighbourMessageWrapperInclusionSplitBodySolution is not correctly initialized, Rank not set!");


        m_neighbourData = this->m_nc->m_nbDataMap.getNeighbourData(this->m_neighbourRank);
        LOGASSERTMSG( this->m_neighbourData, this->m_pSerializerLog,
                      "There exists no NeighbourData for neighbourRank: " << this->m_neighbourRank << "in process rank: "
                      << this->m_nc->m_rank << "!");


        ar & this->m_time;
        ar & m_step;

        //Serialize all solved velocities of all local SplitBodies
        unsigned int size = m_neighbourData->sizeLocal();

        ar & size;

        if(size>0) {
            LOGIC_SZ(this->m_pSerializerLog, "InclusionComm-SplitBodySolution=============================================================================="<< std::endl;)
            LOGIC_SZ(this->m_pSerializerLog, "SERIALIZE Message for neighbour rank: " << this->m_neighbourRank << std::endl;);

            for(auto it = m_neighbourData->localBegin(); it != m_neighbourData->localEnd(); it++){
                ar & it->first; // id

                LOGASSERTMSG( it->second.m_pBody->m_pSolverData, this->m_pSerializerLog,
                             "SolverData for body id: " << RigidBodyId::getBodyIdString(it->first) << "zero!");

                //Because all splitbody velocities in the splitNode are equal due to the billateral cosntraint,
                // we can just send the first velocity to all participating ranks
                // which is the local bodies new front velocity
                serializeEigen(ar,it->second.m_pBody->m_pSolverData->m_uBuffer.m_front); // front velocity

                LOGIC_SZ(this->m_pSerializerLog, "----> id: " << RigidBodyId::getBodyIdString((it->first)) << std::endl <<
                          "----> uFront: " << it->second.m_pBody->m_pSolverData->m_uBuffer.m_front.transpose() <<std::endl)
            }
        }
        else{
            ERRORMSG("We should send a message to neighbour " << this->m_neighbourRank << " which is non empty! This should not happen!")
            // We know from which neighbours we should receive a nonempty message
        }

        this->m_initialized = false;
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version) const {
        // REMOTE SPLITBODIES
        LOGASSERTMSG( this->m_initialized, this->m_pSerializerLog, "The NeighbourMessageWrapperInclusionSplitBodySolution is not correctly initialized, Rank not set!")

        m_neighbourData = this->m_nc->m_nbDataMap.getNeighbourData(this->m_neighbourRank);
        LOGASSERTMSG( this->m_neighbourData, this->m_pSerializerLog,
                      "There exists no NeighbourData for neighbourRank: " << this->m_neighbourRank << "in process rank: "
                      << this->m_nc->m_rank << "!");

        unsigned int step;
        RigidBodyType::PREC time;
        ar & time;
        ar & step;
        LOGASSERTMSG(time == this->m_time && step == m_step, this->m_pSerializerLog, "Wrong message received!")

        unsigned int size;
        ar & size;

        if(size>0) {
            LOGIC_SZ(this->m_pSerializerLog, "InclusionComm-SplitBodySolution=============================================================================="<< std::endl;)
            LOGIC_SZ(this->m_pSerializerLog, "DESERIALIZE Message for neighbour rank: " << this->m_neighbourRank << std::endl;);

            LOGIC_SZ(this->m_pSerializerLog, "---> # Remote SplitBodies Solutions: " << size << std::endl;);

            RigidBodyIdType id;
            VectorUBody u;
            for(unsigned int i = 0; i < size ; i++) {
                ar & id;
                serializeEigen(ar,u);

                auto * remoteBodyData = m_neighbourData->getRemoteBodyData(id);
                LOGASSERTMSG(remoteBodyData, this->m_pSerializerLog, "The remote split body id: "
                             << RigidBodyId::getBodyIdString(id) << "is not in this neighbour rank: " << this->m_neighbourRank)

                LOGASSERTMSG(remoteBodyData->m_pBody->m_pSolverData, this->m_pSerializerLog, "No solver data in remote split body id: "
                             << RigidBodyId::getBodyIdString(id) << " in this neighbour rank: " << this->m_neighbourRank)

                remoteBodyData->m_pBody->m_pSolverData->m_uBuffer.m_front = u;

                LOGIC_SZ(this->m_pSerializerLog, "----> id: " << RigidBodyId::getBodyIdString(id) << std::endl <<
                          "----> uFront: " << u.transpose() <<std::endl)
            }
        }
        else{
            ERRORMSG("We should not receive an empty message from neighbour " << this->m_neighbourRank << std::endl);
        }

        this->m_initialized = false;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

private:
    mutable unsigned int m_step;
    mutable NeighbourDataType * m_neighbourData;
};





template<typename TTopologyBuilder >
class TopologyBuilderMessageWrapperBodies : public boost::serialization::traits< TopologyBuilderMessageWrapperBodies<TTopologyBuilder>,
    boost::serialization::object_serializable,
        boost::serialization::track_never> {
public:

    DEFINE_LAYOUT_CONFIG_TYPES

    static const MPIMessageTag m_tag = MPIMessageTag::TOPOLOGYBUILDER_POINTGATHER;

    using TopologyBuilderType = TTopologyBuilder;
    using SettingsType = typename TopologyBuilderType::SettingsType;
    using RankIdType = typename TopologyBuilderType::RankIdType ;

    TopologyBuilderMessageWrapperBodies(TopologyBuilderType * topoBuilder):
            m_pTopoBuilder(topoBuilder)
    {};


    // broadcast
    template<class Archive>
    void save(Archive & ar, const unsigned int version) const {

        unsigned int size = m_pTopoBuilder->m_initStates.size() ;
        ar & size;

        // send all bodies state to master rank which build the topology
        for(auto & state :  m_pTopoBuilder->m_initStates ){
            ar & state.second.m_id;
            // Displacement
            serializeEigen(ar,state.second.m_q);
            //Velocity
            serializeEigen(ar,state.second.m_u);
        }





        // if BINET_TENSOR
        if(m_pTopoBuilder->m_settings.m_buildMode == SettingsType::BuildMode::BINET_TENSOR){
            // Local AABB
            ar & m_pTopoBuilder->m_I_aabb_loc;
            // Local Theta
            serializeEigen(ar,m_pTopoBuilder->m_I_theta_loc);
            // Local Center point
            ar & m_pTopoBuilder->m_countPoints_loc;
            serializeEigen(ar,m_pTopoBuilder->m_r_G_loc);
        }
        else if( m_pTopoBuilder->m_settings.m_buildMode == SettingsType::BuildMode::ALIGNED ){
            // Local AABB
            ar & m_pTopoBuilder->m_I_aabb_loc;
        }else if( m_pTopoBuilder->m_settings.m_buildMode == SettingsType::BuildMode::MVBB ){

        }

    }

    // Master reset
    void resetBeforLoad(){
        // Binet
        m_pTopoBuilder->m_I_theta_G_glo.setZero();
        m_pTopoBuilder->m_countPoints_glo = 0;
        m_pTopoBuilder->m_r_G_glo.setZero();
        // Aligned
        m_pTopoBuilder->m_rankAABBs.clear();

        m_pTopoBuilder->m_aabb_glo.reset();
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version) const {

        // MASTER rank receives here all points and AABB from m_rank

        // Size
        unsigned int size;
        ar & size;
        // States
        RigidBodyIdType id;
        for(unsigned int i = 0; i < size; i++){
            ar & id;
            // Pos/Vel

            auto res = m_pTopoBuilder->m_initStates.emplace(std::piecewise_construct,
                                                            std::make_tuple(id) ,
                                                            std::make_tuple(id));

            m_pTopoBuilder->m_massPoints_glo.emplace_back(&res.first->second); // link new masspoint to state
            auto & back = m_pTopoBuilder->m_massPoints_glo.back();

            serializeEigen(ar, back.m_state->m_q);
            serializeEigen(ar, back.m_state->m_u);
        }




        // Binet Tensor
        if(m_pTopoBuilder->m_settings.m_buildMode == SettingsType::BuildMode::BINET_TENSOR){

            // Insert a AABB
            auto insertedAABB = m_pTopoBuilder->m_rankAABBs.emplace(std::piecewise_construct, std::make_tuple(m_rank),std::make_tuple() );
            // Get AABB
            ar & insertedAABB.first->second;

            // Theta in I frame
            // Add Theta to the global one
            Vector6 theta; serializeEigen(ar,theta);
            m_pTopoBuilder->m_I_theta_G_glo += theta;

            // r_G
            // add to global r_G (massen gewichtung, masse=1)
            Vector3 r_G;
            unsigned int countPoints;
            ar & countPoints; // get number of local pred. points
            // add to global predicted points
            m_pTopoBuilder->m_countPoints_glo += countPoints;
            serializeEigen(ar,r_G);
            m_pTopoBuilder->m_r_G_glo += countPoints*r_G;
        }else if( m_pTopoBuilder->m_settings.m_buildMode == SettingsType::BuildMode::ALIGNED ){
            // Insert a AABB
            auto insertedAABB = m_pTopoBuilder->m_rankAABBs.emplace(std::piecewise_construct, std::make_tuple(m_rank),std::make_tuple() );
            // Get AABB
            ar & insertedAABB.first->second;

            // unite local AABB with total AABB
            m_pTopoBuilder->m_aabb_glo += insertedAABB.first->second;
        }

    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

    void setRank(const RankIdType & rank){
        m_rank = rank;
    }


private:
    TTopologyBuilder * m_pTopoBuilder;
    RankIdType m_rank;
};


template<typename TTopologyBuilder >
class TopologyBuilderMessageWrapperOrientation : public boost::serialization::traits< TopologyBuilderMessageWrapperBodies<TTopologyBuilder>,
    boost::serialization::object_serializable,
        boost::serialization::track_never> {
public:

    DEFINE_LAYOUT_CONFIG_TYPES

    static const MPIMessageTag m_tag = MPIMessageTag::TOPOLOGYBUILDER_POINTGATHER;

    using TopologyBuilderType = TTopologyBuilder;
    using RankIdType = typename TopologyBuilderType::RankIdType ;

    TopologyBuilderMessageWrapperOrientation(TopologyBuilderType * topoBuilder):
            m_pTopoBuilder(topoBuilder)
    {};

    // master sends orientation / ranks receive it
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) const {
        serializeEigen(ar,m_pTopoBuilder->m_A_IK);
    }

private:
    TTopologyBuilder * m_pTopoBuilder;
    RankIdType m_rank;
};



template<typename TTopologyBuilder >
class TopologyBuilderMessageWrapperResults: public boost::serialization::traits< TopologyBuilderMessageWrapperResults<TTopologyBuilder>,
    boost::serialization::object_serializable,
        boost::serialization::track_never> {
public:

    DEFINE_LAYOUT_CONFIG_TYPES

    static const MPIMessageTag m_tag = MPIMessageTag::TOPOLOGYBUILDER_RESULTS;

    using TopologyBuilderType = TTopologyBuilder;
    using RankIdType = typename TopologyBuilderType::RankIdType ;

    TopologyBuilderMessageWrapperResults(TopologyBuilderType * topoBuilder):
            m_pTopoBuilder(topoBuilder)
    {};


    // master sends all data to neighbours
    template<class Archive>
    void save(Archive & ar, const unsigned int version) const {

        // Send grid data
        ar & m_pTopoBuilder->m_aabb_glo;
        serializeEigen(ar,m_pTopoBuilder->m_settings.m_processDim);

        ar & m_pTopoBuilder->m_settings.m_aligned;

        if( m_pTopoBuilder->m_settings.m_aligned){
            serializeEigen(ar,m_pTopoBuilder->m_A_IK);
        }

        // Send states of bodies
        unsigned int nStates = 0;
        // Get states for this rank if any!
        auto stateListIt = m_pTopoBuilder->m_bodiesPerRank.find(m_rank);
        if(stateListIt ==  m_pTopoBuilder->m_bodiesPerRank.end()){
            ar & nStates;
        }else{
            nStates = stateListIt->second.size();
            ar & nStates;
            // Serialize states
            for( auto & state : stateListIt->second){
                ar & state->m_id;
                serializeEigen(ar,state->m_q);
                serializeEigen(ar,state->m_u);
            }

            // clear the entry for this rank
            m_pTopoBuilder->m_bodiesPerRank.erase(stateListIt);
        }
    }

    // receive broadcast all ranks except master
    void resetBeforLoad(){
        m_pTopoBuilder->m_pDynSys->m_bodiesInitStates.clear();
    }
    template<class Archive>
    void load(Archive & ar, const unsigned int version) const {

        // Recv grid data
        ar & m_pTopoBuilder->m_aabb_glo;
        serializeEigen(ar,m_pTopoBuilder->m_settings.m_processDim);

        ar & m_pTopoBuilder->m_settings.m_aligned;

        if( m_pTopoBuilder->m_settings.m_aligned){
            serializeEigen(ar,m_pTopoBuilder->m_A_IK);
        }


        unsigned int nStates;
        ar & nStates;
        RigidBodyIdType id;

        // this does only add the states to the dynamic system, this class is not responsible for clearing it!
        auto & bodiesInitStates = m_pTopoBuilder->m_pDynSys->m_bodiesInitStates;
        for(unsigned int i = 0; i< nStates;i++){
            ar & id;
            auto res = bodiesInitStates.emplace(id,id);
            ASSERTMSG(res.second, "Body with id: " << id << " has already been added in list!?");
            serializeEigen(ar, res.first->second.m_q);
            serializeEigen(ar , res.first->second.m_u);
        }
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

    void setRank(const RankIdType & rank){
        m_rank = rank;
    }


private:
    TTopologyBuilder * m_pTopoBuilder;
    RankIdType m_rank;
};




}; // MPILayer



#endif
