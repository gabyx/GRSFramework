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

    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    // Neighbour data definitions
    using DataType = TData;
    using Type = std::unordered_map<RankIdType, DataType >;
    using iterator = typename Type::iterator;

    NeighbourMap(RankIdType rank): m_rank(rank){};

    ~NeighbourMap(){
        m_nbDataMap.clear();
    }

    inline typename Type::iterator begin(){return m_nbDataMap.begin();}
    inline typename Type::iterator end(){return m_nbDataMap.end();}


    inline std::pair<DataType *, bool> insert(const RankIdType & rank){
        auto resPair = m_nbDataMap.insert( typename Type::value_type(rank, DataType(rank) ));
        return std::pair<DataType*, bool>(&resPair.first->second, resPair.second);
    }

    inline DataType * getNeighbourData(const RankIdType & rank){
        auto it = m_nbDataMap.find(rank);
        if(it != m_nbDataMap.end()){
           return (&it->second);
        }else{
           ASSERTMSG(false,"There is no NeighbourData for rank: " << rank << "!")
           return nullptr;
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
