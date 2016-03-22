#ifndef TemplateTemplateParameters_hpp
#define TemplateTemplateParameters_hpp

#include <vector>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index/tag.hpp>
#include <boost/multi_index/key_extractors.hpp>

struct Data{
    int a;
};

struct by_insertion{};
struct by_id{};


typedef boost::multi_index::multi_index_container<
        Data,
        boost::multi_index::indexed_by<
            boost::multi_index::random_access<
                boost::multi_index::tag<by_insertion>
            >, // this index represents insertion order
            boost::multi_index::hashed_unique<
                boost::multi_index::tag<by_id>,
                boost::multi_index::member<Data, int, &Data::a>
            >
        >
    > MapType;


MapType::index<by_insertion>::type::iterator projectFoo(MapType &map, MapType::index<by_id>::type::iterator it){
    return map.project<by_insertion>(it);
}

void multiIndexContainerTest(){


    MapType map;
    Data data; data.a = 2;
//    map.get<by_insertion>().insert(data);   no insertion over random_access ()not assignable)
    map.get<by_id>().insert(data);
    MapType::index<by_id>::type::iterator it = map.get<by_id>().find(2);
//
    MapType::index<by_insertion>::type::iterator it2 = projectFoo(map, it);




}


#endif
