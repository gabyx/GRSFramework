#ifndef MPIMessages_hpp
#define MPIMessages_hpp


#include "AssertionDebug.hpp"
#include "AssertionDebugMPI.hpp"
#include "TypeDefs.hpp"


#include <boost/tuple/tuple.hpp>

#include <boost/type_traits.hpp>

#include <boost/filesystem/path.hpp>
#include <boost/serialization/level.hpp>

namespace boost { namespace serialization {

template<class Archive >
void serialize(Archive& ar, boost::filesystem::path & p,
                const unsigned int version)
{
     boost::filesystem::path::string_type s;
     if(Archive::is_saving::value)
         s = p.string();
     ar & s;
     if(Archive::is_loading::value)
         p = s;
}

//template<class Archive, typename T >
//void serialize(Archive& ar, T * p, const unsigned int version)
//{
//     if(Archive::is_saving::value){
//         ar & *p;
//     }else{
//         if(!p){
//            p = new T();
//         }
//         ar & *p;
//     }
//}

}}

namespace MPILayer{

//#define CONCAT(a,b) a ## b
//
//#define SERIALIZE_TUPLE_ELEMS(z,nargs,unused) \
//        ar  &  m_data.template get< nargs >();\

//#define GENERATE_GENERICMESSAGE_CLASS(z,nargs,unused) \


    template<typename T1>
    class GenericMessage1{
        public:

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & m_data.template get<0>();
        }

        boost::tuple<T1> m_data;
    };

    template<typename T1, typename T2>
    class GenericMessage2{
        public:

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & m_data.template get<0>();
            ar & m_data.template get<1>();
        }

        boost::tuple<T1,T2> m_data;
    };



    //BOOST_PP_REPEAT_FROM_TO(2,3,GENERATE_GENERICMESSAGE_CLASS,nothing);

};


#endif
