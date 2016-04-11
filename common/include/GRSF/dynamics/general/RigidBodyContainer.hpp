// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_RigidBodyContainer_hpp
#define GRSF_dynamics_general_RigidBodyContainer_hpp

#include <boost/iterator/transform_iterator.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index/tag.hpp>
#include <boost/multi_index/key_extractors.hpp>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"

#include RigidBody_INCLUDE_FILE

#include "GRSF/common/AssertionDebug.hpp"


class RigidBodyContainer {
private:

    // Tags for accesing the index
    struct by_insertion{};
    struct by_hashed_id{};
    struct by_ordered_id{};

public:
    DEFINE_RIGIDBODY_CONFIG_TYPES

    using RigidBodyIdType = RigidBodyType::RigidBodyIdType;

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

    using MapByInsertionType = typename MapType::index<by_insertion>::type   ;
    using MapByHashedIdType  = typename MapType::index<by_hashed_id>::type   ;
    using MapByOrderedIdType = typename MapType::index<by_ordered_id>::type  ;
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
    using iterator = typename MapByInsertionType::iterator;
    using const_iterator = typename MapByInsertionType::const_iterator;
    /** Body iterator ordered by insertion */
    using iterator_ordered = typename MapByOrderedIdType::iterator;
    using const_iterator_ordered = typename MapByOrderedIdType::const_iterator;
    /** Key ierator ordered by id */
    using key_iterator_ordered = boost::transform_iterator< KeyGetter<iterator>, iterator>;
    /** Key ierator ordered by insertion */
    using key_iterator = boost::transform_iterator< KeyGetter<iterator>, iterator>;

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
        GRSF_ASSERTMSG(ptr != nullptr, "Null pointer added!")
        std::pair<typename MapByHashedIdType::iterator,bool> res =  m_mapByHashedId.insert(ptr);
        return res.second;
    }

    /** Similiar to std::map::erase, does not delete the pointer*/
    inline bool removeBody(RigidBodyType* ptr){
        return m_mapByHashedId.erase(ptr->m_id); // Returns size_type integer
    }

    bool deleteBody(RigidBodyType* ptr){
        return deleteBody(ptr->m_id);
    }

    bool deleteBody(RigidBodyIdType const & id){
        typename MapByHashedIdType::iterator it = m_mapByHashedId.find(id);
        if(it != m_mapByHashedId.end()){
            GRSF_ASSERTMSG(*it != nullptr, " Pointer of body: " << id << " in map is null!")
            auto * body = *it; // save ptr
            m_mapByHashedId.erase(it); // delete in map (might rehash, so we are not allowed to delete the body above!)
            delete body;
            return true;
        }
        return false;
    }


    inline bool deleteAllBodies(){
        iterator it;
        for(it = begin(); it != end(); it++){
            GRSF_ASSERTMSG(*it != nullptr, " Pointer in map is null!")
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




#endif
