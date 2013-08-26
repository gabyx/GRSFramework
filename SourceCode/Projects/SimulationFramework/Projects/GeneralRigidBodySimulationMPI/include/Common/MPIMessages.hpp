#ifndef MPIMessages_hpp
#define MPIMessages_hpp


#include "AssertionDebug.hpp"
#include "AssertionDebugMPI.hpp"
#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include <boost/tuple/tuple.hpp>
#include <boost/type_traits.hpp>

#include <boost/filesystem/path.hpp>
#include <boost/serialization/level.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
//#include <boost/serialization/split_member.hpp>

#include "MPISerializationHelpers.hpp"

#include "FileManager.hpp"
#include "SimpleLogger.hpp"



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

//template<class Archive, typename T >
//void serialize(Archive& ar, T * p, const unsigned int version)
//{
//     if(Archive::is_saving::value){
//         ar & *p;
//     }else{
//         if(!p){
//            p = new T();
//         }
//         ar & *p;
//     }
//}

}
}

namespace MPILayer {

//#define CONCAT(a,b) a ## b
//
//#define SERIALIZE_TUPLE_ELEMS(z,nargs,unused) \
//        ar  &  m_data.template get< nargs >();\

//#define GENERATE_GENERICMESSAGE_CLASS(z,nargs,unused) \


template<typename T1>
class GenericMessage1 {
public:

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & m_data.template get<0>();
    }

    boost::tuple<T1> m_data;
};

template<typename T1, typename T2>
class GenericMessage2 {
public:

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & m_data.template get<0>();
        ar & m_data.template get<1>();
    }

    boost::tuple<T1,T2> m_data;
};

//BOOST_PP_REPEAT_FROM_TO(2,3,GENERATE_GENERICMESSAGE_CLASS,nothing);



template<typename TNeighbourCommunicator >
class NeighbourMessageWrapper : public boost::serialization::traits< NeighbourMessageWrapper<TNeighbourCommunicator>,
    boost::serialization::object_serializable,
        boost::serialization::track_never> {
public:


    typedef TNeighbourCommunicator NeighbourCommunicatorType;
    typedef typename NeighbourCommunicatorType::DynamicsSystemType DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(NeighbourCommunicatorType::DynamicsSystemConfig);

    typedef typename NeighbourCommunicatorType::ProcessCommunicatorType       ProcessCommunicatorType;
    typedef typename NeighbourCommunicatorType::ProcessInfoType               ProcessInfoType;
    typedef typename NeighbourCommunicatorType::RankIdType                    RankIdType;
    typedef typename NeighbourCommunicatorType::ProcessTopologyType           ProcessTopologyType;
    typedef typename NeighbourCommunicatorType::RigidBodyContainerType        RigidBodyContainerType;


    typedef typename NeighbourCommunicatorType::BodyInfoMapType               BodyInfoMapType;
    typedef typename BodyInfoMapType::DataType                                BodyInfoType;

    typedef typename NeighbourCommunicatorType::NeighbourMapType              NeighbourDataMapType;
    typedef typename NeighbourDataMapType::DataType                           NeighbourDataType ;

    NeighbourMessageWrapper(NeighbourCommunicatorType * nc): m_nc(nc), m_initialized(false) {

        if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
            m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
        } else {
            ERRORMSG("SimulationLog does not yet exist? Did you create it?")
        }

        boost::filesystem::path filePath = FileManager::getSingletonPtr()->getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "MPISerializer.log";
        m_pSerializerLog = Logging::LogManager::getSingletonPtr()->createLog("MPISerializerLog",false,true,filePath);
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

    enum class SubMessageFlag : char {
        NOTIFICATION = 1 << 0,
        UPDATE = 1 << 1,
        REMOVAL = 1 << 2
    };

    template<class Archive>
    void save(Archive & ar, const unsigned int version) const {


        ASSERTMSG(m_initialized, "The NeighbourMessageWrapper is not correctly initialized, Rank not set!");
        LOGSZ(m_pSerializerLog, "=========================================================================================="<< std::endl;)
        LOGSZ(m_pSerializerLog, "SERIALIZE Message for neighbour rank: " << m_neighbourRank << std::endl;);

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
        LOGSZ(m_pSerializerLog, "---> Timestamp: "<<  m_nc->m_currentSimTime << std::endl;);
        ar & m_nc->m_currentSimTime;

        // Number of submessages to send (for each local body 1)
        NeighbourDataType * neighbourData = m_nc->m_nbDataMap.getNeighbourData(m_neighbourRank);
        unsigned int size = neighbourData->sizeLocal();
        ar & size;
        LOGSZ(m_pSerializerLog, "---> Size: "<<  size << std::endl;);
        //Loop over all localBodies in this NeighbourData
        int i = 0;
        for(typename NeighbourDataType::LocalIterator it = neighbourData->localBegin(); it != neighbourData->localEnd(); it++) {
            SubMessageFlag flag;
            if(it->second->m_commStatus == NeighbourDataType::LocalData::SEND_NOTIFICATION) {
                flag = SubMessageFlag::NOTIFICATION;
                ar & flag;
                LOGSZ(m_pSerializerLog, "---> NotifactionSTART: " << i<<std::endl;);
                serializeNotification(ar, it->second->m_body,version);
                LOGSZ(m_pSerializerLog, "---> NotifactionEND: "<<std::endl;);

            } else if (it->second->m_commStatus == NeighbourDataType::LocalData::SEND_UPDATE) {
                flag = SubMessageFlag::UPDATE;
                ar & flag;
                LOGSZ(m_pSerializerLog, "---> UpdateSTART: " << i<<std::endl;);
                serializeUpdate(ar, it->second->m_body,version);
                LOGSZ(m_pSerializerLog, "---> UpdateEND: "<<std::endl;);

            } else if (it->second->m_commStatus == NeighbourDataType::LocalData::SEND_REMOVE) {
                flag = SubMessageFlag::REMOVAL;
                ar & flag;
                LOGSZ(m_pSerializerLog, "---> RemovalSTART: " << i<<std::endl;);
                serializeRemoval(ar, it->second->m_body,version);
                LOGSZ(m_pSerializerLog, "---> RemovalEND: "<<std::endl;);
            }

            i++;
        }

        m_initialized = false;
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version) const {
        ASSERTMSG(m_initialized, "The NeighbourMessageWrapper is not correctly initialized, Rank not set!")
        LOGSZ(m_pSerializerLog, "=========================================================================================="<< std::endl;)
        LOGSZ(m_pSerializerLog, "DESERIALIZE Message for neighbour rank: " << m_neighbourRank << std::endl;);


        // Simulation Time:
        PREC simulationTime;
        ar & simulationTime; LOGSZ(m_pSerializerLog, "---> Timestamp: "<<  simulationTime << std::endl;);
        ASSERTMSG(m_nc->m_currentSimTime == simulationTime, "The message from rank: "<< m_neighbourRank << " has timeStamp: " << simulationTime<<" which does not fit our current simulation Time: "<< m_nc->m_currentSimTime << " for rank: " << m_nc->m_rank <<" !")


        // Number of submessages
        unsigned int size;
        ar & size; LOGSZ(m_pSerializerLog, "---> Size: "<<  size << std::endl;);

        //Loop over all messages
        for(int i = 0; i < size; i++) {
            SubMessageFlag flag;
            ar & flag;
            if(flag == SubMessageFlag::NOTIFICATION) {
                LOGSZ(m_pSerializerLog, "---> NotifactionSTART: " << i<<std::endl;);
                //deserializeNotification(ar,version);
                LOGSZ(m_pSerializerLog, "---> NotifactionEND: "<<std::endl;);

            } else if (flag == SubMessageFlag::UPDATE) {
                LOGSZ(m_pSerializerLog, "---> UpdateSTART: " << i<<std::endl;);
                deserializeUpdate(ar,version);
                LOGSZ(m_pSerializerLog, "---> UpdateEND: "<<std::endl;);

            } else if (flag == SubMessageFlag::REMOVAL) {
                LOGSZ(m_pSerializerLog, "---> RemovalSTART: " << i<<std::endl;);
                //deserializeRemoval(ar,version);
                LOGSZ(m_pSerializerLog, "---> RemovalEND: "<<std::endl;);
            }
        }

        m_initialized = false;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

private:

    template<class Archive>
    void serializeRemoval(Archive & ar, RigidBodyType * body, const unsigned int version) const {
        // serialize (ONLY LOCAL BODIES)
        // send the id to remove
        ar & body->m_id;

    }

    template<class Archive>
    void deserializeRemoval(Archive & ar, const unsigned int version) const {
        // deserialize
        typename RigidBodyType::RigidBodyIdType id;
        ar & id;
        // Go into the neighbour data structure and remove the remote body
        m_nc->m_nbDataMap.find(m_neighbourRank)->second->removeRemoteBody(id);
        // Remove from bodyToInfoList
        typename BodyInfoMapType::iterator it = m_nc->m_bodyToInfo.erase(id);
        ASSERTMSG(it !=  m_nc->m_bodyToInfo.end(), "Remote Body with id: " << id << " is not contained in m_bodyToInfo!");
        //Remove and delete from global list
        bool res = m_nc->m_globalRemote.removeAndDeleteBody(id);
        ASSERTMSG(res == true, "Remote Body with id: " << id << " could not be deleted in m_globalRemote!");


    }

    template<class Archive>
    void serializeUpdate(Archive & ar, RigidBodyType * body, const unsigned int version) const {
        // serialize (ONLY LOCAL BODIES)

        // id
        ar & body->m_id; LOGSZ(m_pSerializerLog, "-----> body id: " << body->m_id<<std::endl;);

        // owning rank
        BodyInfoType * bodyInfo = m_nc->m_bodyToInfo.getBodyInfo(body);


        ar & bodyInfo->m_ownerRank; LOGSZ(m_pSerializerLog, "-----> owning rank: " << bodyInfo->m_ownerRank<<std::endl;);
        //Position
        serializeEigen(ar,body->m_r_S,version);
        serializeEigen(ar,body->m_q_KI,version);

        //TODO

        //Velocity
        ASSERTMSG(body->m_pSolverData,"No SolverData present in body with id: "<< body->m_id << "!");
        serializeEigen(ar,body->m_pSolverData->m_uBuffer.m_back,version);

        // Additional info if body belongs to neighbour!
        if( bodyInfo->m_ownerRank == m_neighbourRank){ // the body belongs now to m_neighbourRank
            // send a list of all adjacent neighbours where the body overlaps
            // need to know where to send the update next time!
            std::set<RankIdType> overlappingNeighbours; // set of ranks where this body overlaps for m_neighbourRank!
            const typename ProcessTopologyType::NeighbourRanksListType & adjRanks = m_nc->m_pProcTopo->getAdjacentNeighbourRanks(m_neighbourRank);

            LOGSZ(m_pSerializerLog, "-----> overlappingNeigbours: "<<std::endl;);
            for( auto it =  bodyInfo->m_neighbourRanks.begin(); it != bodyInfo->m_neighbourRanks.end(); it++){
                if(it->second.m_bOverlaps == true && adjRanks.find(it->first) != adjRanks.end() ){
                    // this body overlaps a rank which is adjacent to m_neighbourRank
                    overlappingNeighbours.insert(it->first);

                    LOGSZ(m_pSerializerLog, "-------> " << it->first <<std::endl;);
                }
            }

            if( bodyInfo->m_overlapsThisRank){
                overlappingNeighbours.insert(m_nc->m_rank);
                LOGSZ(m_pSerializerLog, "-------> own: " << m_nc->m_rank <<std::endl;);
            }

            //serialize the set
            ar & overlappingNeighbours;
        }
    }

    template<class Archive>
    void deserializeUpdate(Archive & ar, const unsigned int version) const {
        // deserialize
        typename RigidBodyType::RigidBodyIdType id;
        ar & id;
        RankIdType owningRank;
        ar & owningRank;

        ASSERTMSG( m_nc->m_nbRanks.find(owningRank) != m_nc->m_nbRanks.end(), "Owner Rank: " << owningRank << " for body with id: "<<id << " is no neighbour in process rank: " << m_nc->m_rank << "!")

        // normal update
        auto * neighbourData = m_nc->m_nbDataMap.getNeighbourData(m_neighbourRank);
        ASSERTMSG(neighbourData,"There exists no NeighbourData for body with id: " << id << " for neighbourRank: " << m_neighbourRank << "in process rank: " << m_nc->m_rank << "!");

        auto * remoteData = neighbourData->getRemoteBodyData(id);
        ASSERTMSG(remoteData,"There exists no RemoteData for body with id: " << id << " for neighbourRank: " << m_neighbourRank << "in process rank: " << m_nc->m_rank << "!");
        ASSERTMSG(m_nc->m_globalRemote.find(id) != m_nc->m_globalRemote.end(),"m_globalRemote does not contain body with id: " << id << " in process rank: " << m_nc->m_rank << "!");

        RigidBodyType * body = remoteData->m_body;

        //Update the body:
        // Position
        serializeEigen(ar, body->m_r_S,version);
        serializeEigen(ar,body->m_q_KI,version);
        // TODO Update A_IK!!!!!
        // Velocity
        ASSERTMSG(body->m_pSolverData, "No SolverData present in body with id: "<< id << " in process rank: " << m_nc->m_rank << "!");
        serializeEigen(ar,body->m_pSolverData->m_uBuffer.m_back,version);


        if(owningRank == m_nc->m_rank){ // if the body is now our local body!

            std::set<RankIdType> overlappingNeighbours; // these are only the common neighbours between m_neighbourRank and m_nc->m_rank
            ar & overlappingNeighbours; // all ranks where the body overlaps

            // Move the remote body to the locale ones and delete in remotes
            m_nc->m_globalRemote.removeBody(body);
            m_nc->m_globalLocal.addBody(body); // NEW BODY AS LOCAL!!!

            // Move the remote out of the neighbour structure
            // (needs to be in the neighbour structure with rank m_neighbourRank, otherwise this update is somewhat stupid?
            bool res = neighbourData->deleteRemoteBodyData(body);
            ASSERTMSG(res,"Body with id: " << id << "not deleted in neighbour structure (?)");

            // Change the body info
            auto * bodyInfo = m_nc->m_bodyToInfo.getBodyInfo(id);
            bodyInfo->m_isRemote = false;
            bodyInfo->m_overlapsThisRank = true;
            bodyInfo->m_ownerRank = m_nc->m_rank;

            // Add all neighbours which need updates!
            bodyInfo->m_neighbourRanks.clear();
            for(auto it = overlappingNeighbours.begin(); it != overlappingNeighbours.end(); it++){
                ASSERTMSG( *it != m_nc->m_rank, "overlappingNeighbours should not contain own rank: " << m_nc->m_rank);
                ASSERTMSG(m_nc->m_nbRanks.find(*it) !=  m_nc->m_nbRanks.end(), "This rank: " << *it << " is no neighbour of our process rank: " << m_nc->m_rank);
                bodyInfo->m_neighbourRanks[*it] = typename BodyInfoType::Flags(true); // Overlaps this rank!

                auto localDataIt = m_nc->m_nbDataMap.getNeighbourData(*it)->addLocalBodyData(body);
                localDataIt->second->m_commStatus = NeighbourDataType::LocalData::SEND_UPDATE;
            }




        }else if(owningRank != m_neighbourRank) { // if the body is not owned anymore by the sending process

            // Move the remote out of the neighbour structure
            // (needs to be in the neighbour structure with rank m_neighbourRank, otherwise this update is somewhat stupid?
            bool res = neighbourData->deleteRemoteBodyData(body);
            ASSERTMSG(res,"Body with id: " << id << "not deleted in neighbour structure (?)");

            m_nc->m_nbDataMap.getNeighbourData(owningRank)->addRemoteBodyData(body);

            // Change the body info
            auto * bodyInfo = m_nc->m_bodyToInfo.getBodyInfo(id);
            bodyInfo->m_isRemote = true; // No need to set!
            bodyInfo->m_overlapsThisRank = true; // No need to set!
            bodyInfo->m_ownerRank = owningRank;
            bodyInfo->m_neighbourRanks.clear();
            bodyInfo->m_neighbourRanks[owningRank] = typename BodyInfoType::Flags(true);

        }
    }

    template<class Archive>
    void serializeNotification(Archive & ar, RigidBodyType * body, const unsigned int version) const {
        if( Archive::is_saving::value ) {
            // serialize (ONLY LOCAL BODIES)

        } else {
            // deserialize
        }
    }



    NeighbourCommunicatorType* m_nc;
    RankIdType m_neighbourRank; ///< This is the neighbour rank where the message is send to or received from!
    mutable bool m_initialized;

    Logging::Log *  m_pSerializerLog;
    Logging::Log *  m_pSimulationLog;

};


}; // MPILayer


#endif
