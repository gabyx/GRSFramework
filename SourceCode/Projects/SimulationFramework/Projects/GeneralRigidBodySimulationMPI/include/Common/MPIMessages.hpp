#ifndef MPIMessages_hpp
#define MPIMessages_hpp


#include "AssertionDebug.hpp"
#include "AssertionDebugMPI.hpp"
#include "TypeDefs.hpp"


#include <boost/tuple/tuple.hpp>
#include <boost/type_traits.hpp>

#include <boost/filesystem/path.hpp>
#include <boost/serialization/level.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
//#include <boost/serialization/split_member.hpp>

#include "MPISerializationHelpers.hpp"

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

    NeighbourMessageWrapper(NeighbourCommunicatorType * nc, RankIdType neigbourRank): m_nc(nc), m_neighbourRank(neigbourRank), m_initialized(true) {};
    NeighbourMessageWrapper(NeighbourCommunicatorType * nc): m_nc(nc), m_initialized(false) {};

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

//        // Simulation Time:
        ar & m_nc->m_currentSimTime;

        // Number of submessages to send (for each local body 1)
        NeighbourDataType * neighbourData = m_nc->m_nbDataMap.getNeighbourData(m_neighbourRank);
        unsigned int size = (unsigned int)neighbourData->m_localBodies.size();
        ar & size;

        //Loop over all localBodies in this NeighbourData

        for(typename NeighbourDataType::LocalBodiesMapType::iterator it = neighbourData->m_localBodies.begin();
                it != neighbourData->m_localBodies.end(); it++) {
            SubMessageFlag flag;
            if(it->second.m_commStatus == NeighbourDataType::LocalOverlapData::SEND_NOTIFICATION) {
                flag = SubMessageFlag::NOTIFICATION;
                ar & flag;
                serializeNotification(ar, it->second.m_body,version);

            } else if (it->second.m_commStatus == NeighbourDataType::LocalOverlapData::SEND_UPDATE) {
                flag = SubMessageFlag::UPDATE;
                ar & flag;
                serializeUpdate(ar, it->second.m_body,version);

            } else if (it->second.m_commStatus == NeighbourDataType::LocalOverlapData::SEND_REMOVE) {
                flag = SubMessageFlag::REMOVAL;
                ar & flag;
                serializeRemoval(ar, it->second.m_body,version);
            }

        }

    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version) const {
        ASSERTMSG(m_initialized, "The NeighbourMessageWrapper is not correctly initialized, Rank not set!")
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
    void deserializeRemoval(Archive & ar, const unsigned int version) {
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
        ar & body->m_id;
        // owning rank
        BodyInfoType * bodyInfo = m_nc->m_bodyToInfo.getBodyInfo(body);

        ar & bodyInfo->m_ownerRank;

        if( bodyInfo->m_ownerRank == m_neighbourRank){ // the body belongs now to m_neighbourRank
            // send a list of all adjacent neighbours where the body overlaps
            // need to know where to send the update next time!
            std::set<RankIdType> overlappingNeighbours; // set of ranks where this body overlaps for m_neighbourRank!
            const typename ProcessTopologyType::NeighbourRanksListType & adjRanks =
            m_nc->m_pProcTopo->getAdjacentNeighbourRanks(m_neighbourRank);

            for( typename BodyInfoType::RankToFlagsType::iterator it =  bodyInfo->m_neighbourRanks.begin();
                  it != bodyInfo->m_neighbourRanks.end(); it++){
                if(it->second.m_bOverlaps == true && adjRanks.find(it->first) != adjRanks.end() ){
                    // this body overlaps a rank which is adjacent to m_neighbourRank
                    overlappingNeighbours.insert(it->first);
                }
            }

            //serialize the set
            ar & overlappingNeighbours;
        }

        //Position
        serializeEigen(ar,body->m_r_S,version);
        serializeEigen(ar,body->m_q_KI,version);
        //Velocity
        ASSERTMSG(body->m_pSolverData,"No SolverData present in body with id: "<< body->m_id << "!");
        serializeEigen(ar,body->m_pSolverData->m_uBuffer.m_back,version);

    }

    template<class Archive>
    void deserializeUpdate(Archive & ar, const unsigned int version) {
        // deserialize
        typename RigidBodyType::RigidBodyIdType id;
        ar & id;
        RankIdType owningRank;
        ar & owningRank;
        bool overlapsNeighbour;
        ar & overlapsNeighbour;

        if( owningRank == m_neighbourRank || owningRank == m_nc->m_rank ) {
            // normal update

            typename RigidBodyContainerType::iterator bodyIt = m_nc->m_globalRemote.find(id);
            ASSERTMSG(bodyIt != m_nc->m_globalRemote.end(),"m_globalRemote does not contain body with id: " << id << "!");

            RigidBodyType * body = (*bodyIt);

            //Update the body:
            serializeEigen(ar, body->m_r_S,version);
            serializeEigen(ar,body->m_q_KI,version);
            ASSERTMSG(body->m_pSolverData, "No SolverData present in body with id: "<< id << "!");
            serializeEigen(ar,body->m_pSolverData.m_buffer.m_back);


            if(owningRank == m_nc->m_rank){
                // its our local body!
                // Move the remote body to the locale ones and delete in remotes
                m_nc->m_globalRemote.removeBody(body->m_id);
                m_nc->m_globalLocal.addBody(body);

                // Move the remote out of the neighbour structure
                m_nc->m_nbDataMap.find(m_neighbourRank)->second->removeRemoteBody(body->m_id);
                if( overlapsNeighbour == true ){
                   typename NeighbourDataType::LocalBodiesMapType::iterator it =  m_nc->m_nbDataMap.find(m_neighbourRank)->second->addLocalBody(body);

                }
            }
        } else {
            // the owner changed, its no more the sender
            // check if the body belongs to one of our neighbour, IT MUST! otherwise something went wrong
            // we assume that the bodies fits in a ball which encloses all 3x3x3 process cells

        }

        if(overlapsNeighbour == false){
                ASSERTMSG(owningRank != m_neighbourRank, " Body with id: "
                          << id << "does not overlap sender rank: "
                          << m_neighbourRank << " but the owner is this rank, -> impossible!");

                //remove from this neighbour data
                m_nc->m_nbDataMap.find(m_neighbourRank)->second->removeRemoteBody(id);
        }else{
                m_nc->m_nbDataMap.find(m_neighbourRank)->second->removeRemoteBody(id);
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
    bool m_initialized;
};


}; // MPILayer


#endif
