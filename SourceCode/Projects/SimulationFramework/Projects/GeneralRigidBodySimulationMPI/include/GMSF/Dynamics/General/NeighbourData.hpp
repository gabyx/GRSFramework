#ifndef GMSF_Dynamics_General_NeighbourData_hpp
#define GMSF_Dynamics_General_NeighbourData_hpp


#include <unordered_map>

#include "TypeDefs.hpp"

#include RigidBody_INCLUDE_FILE


template<typename TLocalData, typename TRemoteData = TLocalData>
class NeighbourData{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    using LocalDataType = TLocalData;
    using RemoteDataType = TRemoteData;

    using RemoteDataMapType = std::unordered_map<typename RigidBodyType::RigidBodyIdType, RemoteDataType >;
    using RemoteIterator = typename RemoteDataMapType::iterator;

    using LocalDataMapType = std::unordered_map<typename RigidBodyType::RigidBodyIdType, LocalDataType >;
    using LocalIterator = typename LocalDataMapType::iterator;


    NeighbourData(const RankIdType & neighbourRank): m_neighbourRank(neighbourRank){};
    ~NeighbourData(){
        m_remoteDataMap.clear();
        m_localDataMap.clear();
    }

    inline unsigned int sizeLocal(){
        return m_localDataMap.size();
    }
    inline unsigned int sizeRemote(){
        return m_remoteDataMap.size();
    }

    inline LocalIterator localBegin(){ return m_localDataMap.begin(); }
    inline RemoteIterator remoteBegin(){ return m_remoteDataMap.begin(); }
    inline LocalIterator localEnd(){ return m_localDataMap.end(); }
    inline RemoteIterator remoteEnd(){ return m_remoteDataMap.end(); }

    inline std::pair<LocalDataType*, bool> addLocalBodyData(RigidBodyType * body){

        auto resPair = m_localDataMap.insert(
                       typename LocalDataMapType::value_type(body->m_id, LocalDataType(body) )
                    );
        return std::pair<LocalDataType *, bool>(&resPair.first->second, resPair.second);
    }


    inline std::pair<RemoteDataType*, bool> addRemoteBodyData(RigidBodyType * body){
        // <iterator,bool>
        auto resPair = m_remoteDataMap.insert(
                    typename RemoteDataMapType::value_type(body->m_id, RemoteDataType(body) )
                    );

        return std::pair<RemoteDataType *, bool>(&resPair.first->second, resPair.second);
    }

    inline RemoteDataType * getRemoteBodyData(RigidBodyType * body){
        return  getRemoteBodyData(body->m_id);
    }

    inline RemoteDataType * getRemoteBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_remoteDataMap.find(id);
        if(it != m_remoteDataMap.end()){
           return (&it->second);
        }else{
           //ASSERTMSG(false,"There is no RemoteDataType for body with id: " << id << "!")
           return NULL;
        }
    }

    inline LocalDataType * getLocalBodyData(RigidBodyType * body){
        return  getLocalBodyData(body->m_id);
    }

    inline LocalDataType * getLocalBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_localDataMap.find(id);
        if(it != m_localDataMap.end()){
           return (&it->second);
        }else{
           //ASSERTMSG(false,"There is no LocalDataType for body with id: " << id << "!")
           return NULL;
        }
    }

    inline bool eraseLocalBodyData(RigidBodyType * body){
        return eraseLocalBodyData(body->m_id);
    }

    inline bool eraseLocalBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_localDataMap.find(id);
        if(it != m_localDataMap.end()){
            m_localDataMap.erase(it);
            return true;
        }
        return false;
    }

    inline bool eraseRemoteBodyData(RigidBodyType * body){
        return eraseRemoteBodyData(body->m_id);
    }

    inline bool eraseRemoteBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_remoteDataMap.find(id);
        if(it != m_remoteDataMap.end()){
            m_remoteDataMap.erase(it);
            return true;
        }
        return false;
    }

    void clear(){
        m_localDataMap.clear();
        m_remoteDataMap.clear();
    }


protected:
    RankIdType m_neighbourRank; // This is the rank to which this data structure belongs!

    /** Private because we do not want that the operator[] is called anywhere in these maps!
    * Not good behaviour if oeprator [] is used, as new element get inserted easily!
    */
    RemoteDataMapType m_remoteDataMap;

    LocalDataMapType m_localDataMap;
};

#endif
