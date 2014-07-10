#ifndef RigidBodyList_hpp
#define RigidBodyList_hpp

#include <boost/iterator/transform_iterator.hpp>
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
                boost::multi_index::member<RigidBodyType::AbsoluteBaseType, const RigidBodyIdType, &RigidBodyType::AbsoluteBaseType::m_id>
            >,
            boost::multi_index::ordered_unique<
                boost::multi_index::tag<by_ordered_id>,
                boost::multi_index::member<RigidBodyType::AbsoluteBaseType, const RigidBodyIdType, &RigidBodyType::AbsoluteBaseType::m_id>
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


    template <typename Iter>
    struct KeyGetter : std::unary_function< const typename Iter::value_type, const RigidBodyIdType &>
    {
         const RigidBodyIdType & operator()(const typename Iter::value_type & pBody) const{ return pBody->m_id; }
    };

public:

     RigidBodyContainer():
        m_mapByHashedId(m_map.get<by_hashed_id>()),
        m_mapByInsertion( m_map.get<by_insertion>()),
        m_mapByOrderedId( m_map.get<by_ordered_id>())
    {}

    /** Body iterator ordered by id */
    typedef typename MapByInsertionType::iterator iterator;
    typedef typename MapByInsertionType::const_iterator const_iterator;
    /** Body iterator ordered by insertion */
    typedef typename MapByOrderedIdType::iterator iterator_ordered;
    typedef typename MapByOrderedIdType::const_iterator const_iterator_ordered;
    /** Key ierator ordered by id */
    typedef boost::transform_iterator< KeyGetter<iterator>, iterator> key_iterator_ordered;
    /** Key ierator ordered by insertion */
    typedef boost::transform_iterator< KeyGetter<iterator>, iterator> key_iterator;

    /** Get body iterators ordered by id */
    iterator_ordered beginOrdered(){ return m_mapByOrderedId.begin(); }
    iterator_ordered endOrdered(){ return m_mapByOrderedId.end(); }
    const_iterator_ordered beginOrdered() const{ return m_mapByOrderedId.begin(); }
    const_iterator_ordered endOrdered() const { return m_mapByOrderedId.end(); }

    /** Get body Iterators ordered by insertion (random access) */
    iterator begin(){return m_mapByInsertion.begin();}
    iterator end(){return m_mapByInsertion.end();}
    const_iterator begin() const {return m_mapByInsertion.begin();}
    const_iterator end() const {return m_mapByInsertion.end();}

    /** Get key iterators ordered by insertion (random access) */
    key_iterator beginKey(){ return key_iterator(m_mapByInsertion.begin(), KeyGetter<iterator>());}
    key_iterator endKey(){return key_iterator(m_mapByInsertion.end(), KeyGetter<iterator>());}

    template<typename Iterator>
    bool addBodies(Iterator beginIt, Iterator endIt){
        for( auto it = beginIt; it!= endIt; ++it){
           auto res =  m_mapByHashedId.insert(*it);
           if( res.second == false){
                return false;
           }
        }
        return true;
    }

    /** Similiar to std::map::insert*/
    inline bool addBody(RigidBodyType* ptr){
        ASSERTMSG(ptr != nullptr, "Null pointer added!")
        std::pair<typename MapByHashedIdType::iterator,bool> res =  m_mapByHashedId.insert(ptr);
        return res.second;
    }

    /** Similiar to std::map::erase, does not delete the pointer*/
    inline bool removeBody(RigidBodyType* ptr){
        return m_mapByHashedId.erase(ptr->m_id); // Returns size_type integer
    }

    /** Similiar to std::map::erase, but also deletes the underlying body*/
    iterator deleteBody(iterator it){
        if(it != this->end()){
            ASSERTMSG(*it != nullptr, " Pointer in map is null!")
            delete *it; // Delete body!
            it = m_mapByInsertion.erase(it);
        }
        return it;
    }

    bool deleteBody(RigidBodyType* ptr){
        return deleteBody(ptr->m_id);
    }

    bool deleteBody(RigidBodyIdType const & id){
        typename MapByHashedIdType::iterator it = m_mapByHashedId.find(id);
        if(it != m_mapByHashedId.end()){
            ASSERTMSG(*it != nullptr, " Pointer in map is null!")
            delete *it; // Delete body!
            m_mapByHashedId.erase(it);
            return true;
        }
        return false;
    }


    inline bool deleteAllBodies(){
        iterator it;
        for(it = begin(); it != end(); it++){
            ASSERTMSG(*it != nullptr, " Pointer in map is null!")
            delete (*it);
        }
        m_map.clear();
        return true;
    }

    inline iterator find(RigidBodyIdType id){
        typename MapByHashedIdType::iterator it = m_mapByHashedId.find(id);
        return m_map.project<by_insertion>(it);
    }

    inline iterator find(RigidBodyType * body){
        typename MapByHashedIdType::iterator it = m_mapByHashedId.find(body->m_id);
        return m_map.project<by_insertion>(it);
    }

    inline void clear(){
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
//    typename TRigidBody::VectorQBody get_q();
//    typename TRigidBody::VectorUBody get_u();
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
