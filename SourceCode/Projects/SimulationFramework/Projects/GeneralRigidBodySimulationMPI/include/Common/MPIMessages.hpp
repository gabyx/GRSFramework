#ifndef MPIMessages_hpp
#define MPIMessages_hpp


#include "AssertionDebug.hpp"
#include "AssertionDebugMPI.hpp"
#include "TypeDefs.hpp"


#include <boost/tuple/tuple.hpp>

#include <boost/type_traits.hpp>

#include <boost/filesystem/path.hpp>
#include <boost/serialization/level.hpp>
//#include <boost/serialization/split_member.hpp>

namespace boost {
    namespace serialization {

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

    }
}

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


    template<typename TNeighbourCommunicator >
    class NeighbourMessageWrapper{
        public:


            typedef TNeighbourCommunicator NeighbourCommunicatorType;
            typedef typename NeighbourCommunicatorType::DynamicsSystemType DynamicsSystemType;
            DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(NeighbourCommunicatorType::DynamicsSystemConfig)

            typedef typename NeighbourCommunicatorType::ProcessCommunicatorType       ProcessCommunicatorType;
            typedef typename NeighbourCommunicatorType::ProcessInfoType               ProcessInfoType;
            typedef typename NeighbourCommunicatorType::RankIdType                    RankIdType;
            typedef typename NeighbourCommunicatorType::ProcessTopologyType           ProcessTopologyType;
            typedef typename DynamicsSystemType::RigidBodySimContainer                RigidBodyContainerType;

            typedef typename NeighbourCommunicatorType::BodyProcessInfoType           BodyProcessInfoType;
            typedef typename NeighbourCommunicatorType::BodyToInfoMapType             BodyToInfoMapType;

            NeighbourMessageWrapper(NeighbourCommunicatorType * nc): m_nc(nc){};

            template<class Archive>
            void save(Archive & ar, const unsigned int version)
            {

                //Message Content:
                /*
                 - PREC simulationTime (to check if the message is the correct one
                 - bool hasUpdates
                    - unsigned int size of BodyUpdates
                    - UpdatePart for local overlapping Body 1
                    - UpdatePart for local overlapping Body 1
                 - bool hasRemoves
                    - unsigned int size of Removes
                    - Body Id of local body which should be removed in the nieghbour
                    - Body Id ....
                    - ....
                 - bool hasMoves
                    - unsigned int number of moving bodies
                    - MoveBodyPart
                        - Body Id (to indetify where to overwrite the data if its already in the remote list)
                        - RigidBody (full Data)
                        - RigidBody (full Data)

                */

            }

            template<class Archive>
            void load(Archive & ar, const unsigned int version)
            {

            }

            BOOST_SERIALIZATION_SPLIT_MEMBER();

        private:
            NeighbourCommunicatorType* m_nc;
    };

}; // MPILayer


#endif
