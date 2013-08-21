#ifndef MPIMessages_hpp
#define MPIMessages_hpp


#include "AssertionDebug.hpp"
#include "AssertionDebugMPI.hpp"
#include "TypeDefs.hpp"


#include <boost/tuple/tuple.hpp>

#include <boost/type_traits.hpp>

#include <boost/filesystem/path.hpp>
#include <boost/serialization/level.hpp>
//#include <boost/serialization/split_member.hpp>

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
class NeighbourMessageWrapper {
public:


    typedef TNeighbourCommunicator NeighbourCommunicatorType;
    typedef typename NeighbourCommunicatorType::DynamicsSystemType DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(NeighbourCommunicatorType::DynamicsSystemConfig)

    typedef typename NeighbourCommunicatorType::ProcessCommunicatorType       ProcessCommunicatorType;
    typedef typename NeighbourCommunicatorType::ProcessInfoType               ProcessInfoType;
    typedef typename NeighbourCommunicatorType::RankIdType                    RankIdType;
    typedef typename NeighbourCommunicatorType::ProcessTopologyType           ProcessTopologyType;
    typedef typename DynamicsSystemType::RigidBodySimContainer                RigidBodyContainerType;

    typedef typename NeighbourCommunicatorType::BodyProcessInfoType           BodyProcessInfoType;
    typedef typename NeighbourCommunicatorType::BodyToInfoMapType             BodyToInfoMapType;

    typedef typename NeighbourCommunicatorType::NeighbourMapType              NeighbourMapType;
    typedef typename NeighbourMapType::NeighbourDataType                      NeighbourDataType ;

    NeighbourMessageWrapper(NeighbourCommunicatorType * nc, RankIdType neigbourRank): m_nc(nc), m_neigbourRank(neigbourRank), m_initialized(true) {};
    NeighbourMessageWrapper(NeighbourCommunicatorType * nc): m_nc(nc), m_initialized(false) {};

    /**
    * Set the rank if we want to reuse this instance and receive another message
    */
    void setRank( RankIdType neigbourRank) {
        m_neigbourRank = neigbourRank;
        if(!m_initialized){m_initialized = true;}
    }

    enum SubMessageFlag {
        NOTIFICATION = 1 << 0,
        UPDATE = 1 << 1,
        REMOVAL = 1 << 2
    };

    template<class Archive>
    void save(Archive & ar, const unsigned int version) const {


        ASSERTMSG(m_initialized, "The NeighbourMessageWrapper is not correctly initialized, Rank not set!")

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
        // Number of submessages to send (for each local body 1)
        unsigned int size = (unsigned int)m_nc->m_nbDataMap[m_neigbourRank]->m_localBodies.size();
        ar & size;

        //Loop over all localBodies in this NeighbourData
        typename NeighbourDataType::LocalBodiesMapType & localBodies = m_nc->m_nbDataMap[m_neigbourRank]->m_localBodies;

        for(typename NeighbourDataType::LocalBodiesMapType::iterator it = localBodies.begin();
                it != localBodies.end(); it++) {
            SubMessageFlag flag;
            if(it->second.m_commStatus == NeighbourDataType::LocalOverlapData::SEND_NOTIFICATION) {
                flag = NOTIFICATION;
                ar & flag;

            } else if (it->second.m_commStatus == NeighbourDataType::LocalOverlapData::SEND_UPDATE) {
                flag = UPDATE;
                ar & flag;

            } else if (it->second.m_commStatus == NeighbourDataType::LocalOverlapData::SEND_REMOVE) {
                flag = REMOVAL;
                ar & flag;

            }

        }

    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version) const {
        ASSERTMSG(m_initialized, "The NeighbourMessageWrapper is not correctly initialized, Rank not set!")
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

private:
    NeighbourCommunicatorType* m_nc;
    RankIdType m_neigbourRank;
    bool m_initialized;
};

}; // MPILayer


#endif
