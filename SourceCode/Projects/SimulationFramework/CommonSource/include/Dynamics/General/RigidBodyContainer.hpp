#ifndef RigidBodyList_hpp
#define RigidBodyList_hpp

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index/tag.hpp>
#include <boost/multi_index/key_extractors.hpp>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include RigidBody_INCLUDE_FILE

#include "AssertionDebug.hpp"


class RigidBodyContainer {
private:

    // Tags for accesing the index
    struct by_insertion{};
    struct by_hashed_id{};
    struct by_ordered_id{};

public:
    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef RigidBodyType::RigidBodyIdType RigidBodyIdType;

    //This container grants only const access with its iterators with (RigidBodyType* const)
    //So we are able to change these rigidbodies, but cant change the m_id because it is const in the rigidbody class
    //If we could accidentally change m_id, this container will still find  with the old hashes the right bucket, but will
    // fail to retrieve it because the check by boost::multi_index::member will fail because we changed id!
    typedef boost::multi_index::multi_index_container<
        RigidBodyType * ,
        boost::multi_index::indexed_by<
            boost::multi_index::random_access<
                boost::multi_index::tag<by_insertion>
            >, // this index represents insertion order
            boost::multi_index::hashed_unique<
                boost::multi_index::tag<by_hashed_id>,
                boost::multi_index::member<RigidBodyType, const RigidBodyIdType, &RigidBodyType::m_id>
            >,
            boost::multi_index::ordered_unique<
                boost::multi_index::tag<by_ordered_id>,
                boost::multi_index::member<RigidBodyType, const RigidBodyIdType, &RigidBodyType::m_id>
            >
        >
    > MapType;

private:

    MapType m_map;

    typedef  typename MapType::index<by_insertion>::type    MapByInsertionType;
    typedef  typename MapType::index<by_hashed_id>::type    MapByHashedIdType;
    typedef  typename MapType::index<by_ordered_id>::type   MapByOrderedIdType;
    MapByHashedIdType & m_mapByHashedId;
    MapByInsertionType & m_mapByInsertion;
    MapByOrderedIdType & m_mapByOrderedId;

public:

     RigidBodyContainer():
        m_mapByHashedId(m_map.get<by_hashed_id>()),
        m_mapByInsertion( m_map.get<by_insertion>()),
        m_mapByOrderedId( m_map.get<by_ordered_id>())
    {}

    typedef typename MapByInsertionType::iterator iterator;
    typedef typename MapByInsertionType::const_iterator const_iterator;

    typedef typename MapByOrderedIdType::iterator iterator_ordered;
    typedef typename MapByOrderedIdType::const_iterator const_iterator_ordered;

    // Ordered by id
    iterator_ordered beginOrdered(){ return m_mapByOrderedId.begin(); }
    iterator_ordered endOrdered(){ return m_mapByOrderedId.end(); }
    const_iterator_ordered beginOrdered() const{ return m_mapByOrderedId.begin(); }
    const_iterator_ordered endOrdered() const { return m_mapByOrderedId.end(); }

    // Ordered by insertion
    iterator begin(){return m_mapByInsertion.begin();}
    iterator end(){return m_mapByInsertion.end();}
    const_iterator begin() const {return m_mapByInsertion.begin();}
    const_iterator end() const {return m_mapByInsertion.end();}


    bool addBody(RigidBodyType* ptr){
        std::pair<typename MapByHashedIdType::iterator,bool> res=  m_mapByHashedId.insert(ptr);
        return res.second;
    }

    bool removeBody(RigidBodyType* ptr){
        return m_mapByHashedId.erase(ptr->m_id);
    }

    bool removeAndDeleteBody(RigidBodyType* ptr){
        return removeAndDeleteBody(ptr->m_id);
    }

    bool removeAndDeleteBody(RigidBodyIdType const & id){
        typename MapByHashedIdType::iterator it = m_mapByHashedId.find(id);
        if(it != m_mapByHashedId.end()){
            delete *it; // Delete body!
            m_mapByHashedId.erase(it);
            return true;
        }
        return false;
    }

    bool removeAndDeleteAllBodies(){
        iterator it;
        for(it = begin(); it != end(); it++){
            delete (*it);
        }
        m_map.clear();
        return true;
    }

    iterator find(RigidBodyIdType id){
        typename MapByHashedIdType::iterator it = m_mapByHashedId.find(id);
        return m_map.project<by_insertion>(it);
    }
    iterator find(RigidBodyType * body){
        typename MapByHashedIdType::iterator it = m_mapByHashedId.find(body->m_id);
        return m_map.project<by_insertion>(it);
    }

    void clear(){
        m_map.clear();
    }

    typename MapType::size_type size() const{
        return m_map.size();
    }

};

//template<typename TIterator, typename TRigidBody>
//class RigidBodyWrapperIterator{
//public:
//    RigidBodyWrapperIterator(TIterator & it): m_bodyIt(it){};
//
//    typename TRigidBody::VectorQObj get_q();
//    typename TRigidBody::VectorUObj get_u();
//
//    TIterator & m_bodyIt;
//};

//template<typename TRigidBodyContainer>
//class RigidBodyContainerWrapper{
//public:
//    RigidBodyContainerWrapper( TRigidBodyContainer & ref ): m_ref(ref){}
//
//    TRigidBodyContainer::iterator begin(){
//        return ref.begin();
//    }
//
//    TRigidBodyContainer::iterator end(){
//        return ref.end();
//    }
//
//};




#endif
