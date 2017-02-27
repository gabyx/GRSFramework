// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <unordered_map>
#include <vector>

#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/key_extractors.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index/tag.hpp>
#include <boost/multi_index_container.hpp>

struct A
{
    A(const unsigned int i) : id(i)
    {
    }
    const unsigned int id;
    double             b;
    float              c;
};

class Container
{
    private:
    // Tags for accesing the index
    struct by_insertion
    {
    };
    struct by_hashed_id
    {
    };
    // struct by_ordered_id {};

    public:
    typedef A Type;

    // This container grants only const access with its iterators with (RigidBodyType* const)
    // So we are able to change these rigidbodies, but cant change the m_id because it is const in the rigidbody class
    // If we could accidentally change m_id, this container will still find  with the old hashes the right bucket, but
    // will
    // fail to retrieve it because the check by boost::multi_index::member will fail because we changed id!
    typedef boost::multi_index::multi_index_container<
        A,
        boost::multi_index::indexed_by<
            boost::multi_index::random_access<boost::multi_index::tag<by_insertion>>,  // this index represents
                                                                                       // insertion order
            boost::multi_index::hashed_unique<boost::multi_index::tag<by_hashed_id>,
                                              boost::multi_index::member<Type, const unsigned int, &A::id>>>>
        MapType;

    MapType m_map;

    typedef typename MapType::index<by_insertion>::type MapByInsertionType;
    typedef typename MapType::index<by_hashed_id>::type MapByHashedIdType;
    // typedef  typename MapType::index<by_ordered_id>::type   MapByOrderedIdType;
    MapByHashedIdType&  m_mapByHashedId;
    MapByInsertionType& m_mapByInsertion;
    // MapByOrderedIdType & m_mapByOrderedId;

    Container() : m_mapByHashedId(m_map.get<by_hashed_id>()), m_mapByInsertion(m_map.get<by_insertion>())  //,
    /*m_mapByOrderedId( m_map.get<by_ordered_id>())*/ {
    }

    typedef typename MapByInsertionType::iterator       iterator;
    typedef typename MapByInsertionType::const_iterator const_iterator;

    //    typedef typename MapByOrderedIdType::iterator iterator_ordered;
    //    typedef typename MapByOrderedIdType::const_iterator const_iterator_ordered;

    //    // Ordered by id
    //    iterator_ordered beginOrdered() {
    //        return m_mapByOrderedId.begin();
    //    }
    //    iterator_ordered endOrdered() {
    //        return m_mapByOrderedId.end();
    //    }
    //    const_iterator_ordered beginOrdered() const {
    //        return m_mapByOrderedId.begin();
    //    }
    //    const_iterator_ordered endOrdered() const {
    //        return m_mapByOrderedId.end();
    //    }

    // Ordered by insertion
    iterator begin()
    {
        return m_mapByInsertion.begin();
    }
    iterator end()
    {
        return m_mapByInsertion.end();
    }
    const_iterator begin() const
    {
        return m_mapByInsertion.begin();
    }
    const_iterator end() const
    {
        return m_mapByInsertion.end();
    }

    /** Similiar to std::map::insert*/
    inline bool add(const A& a)
    {
        std::pair<typename MapByHashedIdType::iterator, bool> res = m_mapByHashedId.insert(a);
        return res.second;
    }

    /** Similiar to std::map::erase, does not delete the pointer*/
    inline bool remove(unsigned int const id)
    {
        return m_mapByHashedId.erase(id);  // Returns size_type integer
    }

    inline iterator find(unsigned int const id)
    {
        typename MapByHashedIdType::iterator it = m_mapByHashedId.find(id);
        return m_map.project<by_insertion>(it);
    }

    inline void clear()
    {
        m_map.clear();
    }

    typename MapType::size_type size() const
    {
        return m_map.size();
    }
};

#define INIT_TIMER auto start = std::chrono::high_resolution_clock::now();
#define START_TIMER start     = std::chrono::high_resolution_clock::now();
#define STOP_TIMER(name)                                                                                      \
    double count =                                                                                            \
        std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count(); \
    std::cout << "RUNTIME of " << name << ": " << count << " ms " << std::endl;

void runTest()
{
    srand(time(NULL));
    int            max = 100000;
    Container      c;                                      // sorted to insertion, sorted to id , and hashed
    std::vector<A> vec;                                    // unsorted to id (
    std::map<unsigned int, A>                          m;  // sorted to id
    std::unordered_map<unsigned int, unsigned int>     idToidx;
    std::vector<std::pair<unsigned int, unsigned int>> idToidxVec;
    std::unordered_map<unsigned int, A*>               idToA;
    std::vector<unsigned int> sortedIds;    // used to iterate over some ids
    std::vector<unsigned int> insertedIds;  // used to iterate over some ids
    unsigned int              ins = 0;
    while (m.size() < max)
    {
        unsigned int i   = rand();
        auto         res = m.insert(std::make_pair(i, A(i)));
        if (res.second)
        {
            vec.push_back(A(i));
            c.add(A(i));
            idToA.insert(std::make_pair(i, &vec.back()));
            idToidx.insert(std::make_pair(i, ins));
            idToidxVec.push_back(std::make_pair(i, ins));
            sortedIds.push_back(i);
            insertedIds.push_back(i);
        }
        ins++;
    }
    std::sort(sortedIds.begin(), sortedIds.end());

    sort(idToidxVec.begin(),
         idToidxVec.end(),
         [](const std::pair<unsigned int, unsigned int>& lhs, const std::pair<unsigned int, unsigned int>& rhs) {
             return lhs.first < rhs.first;
         });

    std::cout << "Size: vector: " << vec.size() << std::endl
              << "Size: map: " << m.size() << std::endl
              << "Size: multiidx: " << c.size() << std::endl;

    unsigned int n;
    INIT_TIMER

    {
        START_TIMER
        auto itEnd = c.end();
        for (auto it = c.begin(); it != itEnd; ++it)
        {
            n += it->id;
        }
        STOP_TIMER("Iteration: [multiidx, byinsertion]")
    }

    {
        START_TIMER
        auto itEnd = c.size();
        for (int i = 0; i < itEnd; i++)
        {
            n += c.m_map[i].id;
        }
        STOP_TIMER("Iteration: [multiidx, byinsertion [i] ]")
    }

    {
        START_TIMER
        auto itEnd = c.size();
        for (auto& a : c.m_map)
        {
            n += a.id;
        }
        STOP_TIMER("Iteration: [multiidx, byinsertion rangebased ]")
    }

    //    {
    //        START_TIMER
    //        auto itEnd = c.endOrdered();
    //        for(auto it=c.beginOrdered(); it!=itEnd; ++it) {
    //            n+= it->id;
    //        }
    //        STOP_TIMER("Iteration: [multiidx, ordered]")
    //    }

    {
        START_TIMER
        auto itEnd = vec.size();
        for (int i = 0; i < itEnd; i++)
        {
            n += vec[i].id;
        }
        STOP_TIMER("Iteration: [vector, iterator]")
    }

    {
        START_TIMER
        auto itEnd = vec.end();
        for (auto it = vec.begin(); it != itEnd; ++it)
        {
            n += it->id;
        }
        STOP_TIMER("Iteration: [vector, [i] ]")
    }

    {
        START_TIMER
        auto itEnd = m.end();
        for (auto it = m.begin(); it != itEnd; ++it)
        {
            n += it->second.id;
        }
        STOP_TIMER("Iteration: [map]")
    }

    {
        START_TIMER
        auto itEnd = vec.end();
        for (auto it = vec.begin(); it != itEnd; ++it)
        {
            n += c.m_mapByHashedId.find(it->id)->id;
        }
        STOP_TIMER("Iteration: [multiidx, find(id) ]")
    }

    {
        START_TIMER
        auto itEnd = sortedIds.size();
        for (int i = 0; i < itEnd; i++)
        {
            auto it = idToidx.find(insertedIds[i]);
            n += vec[it->second].id;
        }
        STOP_TIMER("Iteration: [vector, accessed random ids by find ]")
    }

    {
        START_TIMER
        auto itEnd = sortedIds.size();

        for (int i = 0; i < itEnd; i++)
        {
            auto id = sortedIds[i];
            auto it = std::find_if(idToidxVec.begin(),
                                   idToidxVec.end(),
                                   [&id](const std::pair<unsigned int, unsigned int>& v) { return v.first == id; });
            if (it == idToidxVec.end())
            {
                std::cout << "FAIL" << std::endl;
                exit(-1);
            }
            n += vec[it->second].id;
        }
        STOP_TIMER("Iteration: [vector, accessed random ids by find_if ]")
    }

    {
        START_TIMER
        auto itEnd = insertedIds.size();
        for (int i = 0; i < itEnd; i++)
        {
            auto it = idToidx.find(insertedIds[i]);
            n += vec[it->second].id;
        }
        STOP_TIMER("Iteration: [vector, accessed random (but in order) ids by find ]")
    }

    {
        START_TIMER
        auto itEnd = sortedIds.size();
        for (int i = 0; i < itEnd; i++)
        {
            auto id = sortedIds[i];
            n += c.m_mapByHashedId.find(id)->id;
        }
        STOP_TIMER("Iteration: [multidx, accessed random ids by find]")
    }

    {
        START_TIMER
        auto itEnd = insertedIds.size();
        for (int i = 0; i < itEnd; i++)
        {
            auto id = insertedIds[i];
            n += c.m_mapByHashedId.find(id)->id;
        }
        STOP_TIMER("Iteration: [multidx, accessed random (but in order) ids by find]")
    }

    std::cout << "value:" << n << std::endl;
};
