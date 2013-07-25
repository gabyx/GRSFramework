#ifndef RigidBodyList_hpp
#define RigidBodyList_hpp

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index/tag.hpp>
#include <boost/multi_index/key_extractors.hpp>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "AssertionDebug.hpp"


template<typename TRigidBodyIdType, typename TRigidBodyType >
class RigidBodyContainer {
private:

    // Tags for accesing the index
    struct by_insertion{};
    struct by_id{};

public:
    typedef TRigidBodyIdType RigidBodyIdType;
    typedef TRigidBodyType RigidBodyType;

    //This container grants only const access with its iterators to RigidBodyType*
    typedef boost::multi_index::multi_index_container<
        RigidBodyType * ,
        boost::multi_index::indexed_by<
            boost::multi_index::random_access<
                boost::multi_index::tag<by_insertion>
            >, // this index represents insertion order
            boost::multi_index::hashed_unique<
                boost::multi_index::tag<by_id>,
                boost::multi_index::member<RigidBodyType, const RigidBodyIdType, &RigidBodyType::m_id>
            >
        >
    > MapType;

private:

    MapType m_map;

    typedef  typename MapType::template index<by_insertion>::type MapByInsertionType;
    typedef  typename MapType::template index<by_id>::type        MapByIdType;

    MapByIdType & m_mapById;
    MapByInsertionType & m_mapByInsertion;

public:

     RigidBodyContainer():
        m_mapById(m_map.template get<by_id>()),
        m_mapByInsertion( m_map.template get<by_insertion>())
    {}

    typedef typename MapByInsertionType::iterator iterator;
    typedef typename MapByInsertionType::const_iterator const_iterator;


    inline iterator begin(){return m_mapByInsertion.begin();}
    inline iterator end(){return m_mapByInsertion.end();}
    inline const_iterator begin() const {return m_mapByInsertion.begin();}
    inline const_iterator end() const {return m_mapByInsertion.end();}


    inline bool addBody(RigidBodyType* ptr){
        std::pair<typename MapByIdType::iterator,bool> res=  m_mapById.insert(ptr);
        return res.second;
    }
    inline bool removeBody(RigidBodyIdType const & id){
        typename MapByIdType::size_type a = m_mapById.erase(id);
        return (a>0);
    }

    inline void clear(){
        m_map.clear();
    }

    inline typename MapType::size_type size() const{
        return m_map.size();
    }

};

#endif
