#ifndef NeighbourMap_hpp
#define NeighbourMap_hpp

#include <vector>
#include <list>
#include <type_traits>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"



template<typename TData>
class NeighbourMap {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    typedef typename RigidBodyType::RigidBodyIdType RigidBodyIdType;
    typedef typename RigidBodyType::BodyInfoType BodyInfoType;
    typedef typename RigidBodyType::BodyInfoType::RankToFlagsType RankToFlagsType;


    // Neighbour data definitions
    typedef TData DataType;
    typedef std::map<RankIdType, DataType > Type;
    typedef typename Type::iterator iterator;

    NeighbourMap(RankIdType rank): m_rank(rank){};

    ~NeighbourMap(){
        m_nbDataMap.clear();
    }


    std::pair<DataType *, bool> insert(const RankIdType & rank){
        auto resPair = m_nbDataMap.insert( typename Type::value_type(rank, DataType(rank) ));
        return std::pair<DataType*, bool>(&resPair.first->second, resPair.second);
    }

    inline DataType * getNeighbourData(const RankIdType & rank){
        auto it = m_nbDataMap.find(rank);
        if(it != m_nbDataMap.end()){
           return (&it->second);
        }else{
           ASSERTMSG(false,"There is no NeighbourData for rank: " << rank << "!")
           return NULL;
        }
    }

    /** Executes clear on all neighbour datas*/
    inline void emptyAllNeighbourData(){
        for(auto it = m_nbDataMap.begin(); it != m_nbDataMap.end();it++){
            it->second.clear();
        }
    }

private:
    RankIdType m_rank;
    Type m_nbDataMap;

};





#endif
