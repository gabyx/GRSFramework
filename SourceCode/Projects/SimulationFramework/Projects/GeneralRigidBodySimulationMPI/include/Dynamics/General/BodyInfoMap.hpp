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
                        bool isRemote = false): m_body(body), m_ownerRank(ownRank), m_overlapsThisRank(true), m_isRemote(isRemote){};
        /**
        * Data structure in the Map: Rank -> Flags, Flags define the behaviour what needs to be done with this Body.
        * m_bOverlaps: Used to decide if body is removed from the corresponding neigbourS
        */
        struct Flags{
            Flags(bool overlap = false):m_bOverlaps(overlap){};
            bool m_bOverlaps; ///< Remove flag from this ranks neighbour data
        };

        typedef std::map<RankIdType, Flags> RankToFlagsType;
        RankToFlagsType m_neighbourRanks;

        RankIdType m_ownerRank;   ///< The process rank to which this body belongs (changes during simulation, if change -> send to other process)
        bool m_overlapsThisRank; ///< True if body overlaps this process!, if false

        RigidBodyType * m_body;

        bool m_isRemote;
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
        auto it = m_map.find(body->m_id);
        if(it != m_map.end()){
            delete it->second;
            m_map.erase(it);
            return true;
        }
        return false;
    }


    std::pair<iterator,bool> insert(RigidBodyType * body, RankIdType ownerRank){
        std::pair<typename Type::iterator,bool> res = m_map.insert( typename Type::value_type(body->m_id, (DataType*)NULL) );
        if(res.second){
             res.first->second = new DataType(body,body->m_id, ownerRank);
        }
        return res;
    }

    inline DataType * getBodyInfo(const RigidBodyType * body){
        return getBodyInfo(body->m_id);
    }

    inline DataType * getBodyInfo(typename RigidBodyType::RigidBodyIdType id){
        typename Type::iterator it = m_map.find(id);
        if(it != m_map.end()){
           return (it->second);
        }else{
           ASSERTMSG(false,"There is no BodyInfo for body with id: " << id << "!")
           return NULL;
        }
    }

private:
    Type m_map;
};


#endif
