#ifndef BodyInfoMap_hpp
#define BodyInfoMap_hpp


template<typename TRigidBody, typename TRankId>
class BodyProcessInfo{
public:
        typedef TRankId RankIdType;
        typedef TRigidBody RigidBodyType;

        BodyProcessInfo(RigidBodyType *body,
                        RankIdType ownRank,
                        bool overlapsThisRank = true,
                        bool isRemote = false, bool receivedUpdate = false):
                            m_body(body),
                            m_ownerRank(ownRank),
                            m_overlapsThisRank(overlapsThisRank),
                            m_isRemote(isRemote),
                            m_receivedUpdate(receivedUpdate)
                            {};
        /**
        * Data structure in the Map: Rank -> Flags, Flags define the behaviour what needs to be done with this Body.
        * m_overlaps: Used to decide if body is removed from the corresponding neigbourS
        */
        struct Flags{
            Flags(bool overlap = true, bool inNeighbourMap = true):m_overlaps(overlap), m_inNeighbourMap(inNeighbourMap){};
            bool m_overlaps;         ///< If this body overlaps this neighbour in this timestep
            bool m_inNeighbourMap;   ///< If this body is contained in the neighbourmap or not!
        };

        typedef std::map<RankIdType, Flags> RankToFlagsType;
        RankToFlagsType m_neighbourRanks;

        RankIdType m_ownerRank;   ///< The process rank to which this body belongs (changes during simulation, if change -> send to other process)
        bool m_overlapsThisRank; ///< True if body overlaps this process!, if false

        RigidBodyType * m_body;

        bool m_isRemote;
        bool m_receivedUpdate;

        void resetNeighbourFlags(){
            for(auto it = m_neighbourRanks.begin(); it != m_neighbourRanks.end();){
                it->second.m_overlaps = false;
                if(it->second.m_inNeighbourMap == false){
                    it=m_neighbourRanks.erase(it);
                }else{
                    ++it;
                }
            }
        }

        bool markNeighbourRankToRemove(RankIdType rank){
            auto it = m_neighbourRanks.find(rank);
            if(it!=m_neighbourRanks.end()){
                it->second.m_inNeighbourMap = false;
                return true;
            }
            return false;
        }
};




template<typename TDynamicsSystem, typename TRankId>
class BodyInfoMap {
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef TRankId RankIdType;

    // BodyInfoMap data definitions
    typedef BodyProcessInfo<RigidBodyType,RankIdType> DataType;
    typedef std::map<RankIdType, DataType *> Type;
    typedef typename Type::iterator iterator;

    // Body info definitions
    typedef typename DataType::RankToFlagsType RankToFlagsType;


    BodyInfoMap(){};
    ~BodyInfoMap(){
        for(auto it = m_map.begin(); it != m_map.end(); it++){
            delete it->second;
        }
        m_map.clear();
    }

    bool erase(RigidBodyType * body){
        return erase(body->m_id);
    }

    bool erase(const typename RigidBodyType::RigidBodyIdType & id){
        auto it = m_map.find(id);
        if(it != m_map.end()){
            delete it->second;
            m_map.erase(it);
            return true;
        }
        return false;
    }


    std::pair<DataType *, bool> insert(RigidBodyType * body,
                                       RankIdType ownerRank,
                                       bool overlapsThisRank = true,
                                       bool isRemote = false,
                                       bool receivedUpdate = false){
        std::pair<typename Type::iterator,bool> res = m_map.insert(
                                                                   typename Type::value_type(body->m_id, (DataType*)NULL) );
        if(res.second){
             res.first->second = new DataType(body, ownerRank, overlapsThisRank, isRemote,receivedUpdate);
        }
        return std::pair<DataType *, bool>(res.first->second, res.second);
    }

    inline DataType * getBodyInfo(const RigidBodyType * body){
        return getBodyInfo(body->m_id);
    }

    inline DataType * getBodyInfo(typename RigidBodyType::RigidBodyIdType id){
        typename Type::iterator it = m_map.find(id);
        if(it != m_map.end()){
           return (it->second);
        }else{
           //ASSERTMSG(false,"There is no BodyInfo for body with id: " << RigidBodyId::getBodyIdString(id) << "!")
           return NULL;
        }
    }

private:
    Type m_map;
};


#endif
