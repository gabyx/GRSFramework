#ifndef MPICommunicationFunctions_hpp
#define MPICommunicationFunctions_hpp

#include <mpi.h>

#include "AssertionDebug.hpp"
#include "AssertionDebugMPI.hpp"
#include "TypeDefs.hpp"

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/version.hpp>

namespace MPILayer{


    template<typename T>
    void MPISendBroadcast(const T & t, unsigned int rank, MPI_Comm comm){
            static std::string serial_str;

            //Save data to archive, -> goes into serial_str!
            {
                serial_str.clear(); //Clear serializable string!
                boost::iostreams::back_insert_device<std::string> inserter(serial_str);
                boost::iostreams::stream< boost::iostreams::back_insert_device<std::string> > s(inserter);
                boost::archive::binary_oarchive oa(s);
                oa << t;
            }
        int size = serial_str.size();
        int error = MPI_Bcast(&size, 1 , MPI_INT, rank, comm); // First send size, because we cannot probe on the other side!! Collective Communication
        ASSERTMPIERROR(error,"MPISendBroadcast failed!");
        MPI_Bcast(const_cast<char*>(serial_str.data()), serial_str.size(), MPI::CHAR, rank, comm);
        ASSERTMPIERROR(error,"MPISendBroadcast failed!");
    };

    template<typename T>
    void MPIReceiveBroadcast(T & t, unsigned int rank, MPI_Comm comm){

            char * buf;
            int message_length;

            int error = MPI_Bcast(&message_length, 1 , MPI_INT, rank, comm);
            ASSERTMPIERROR(error,"MPISendBroadcast failed!");
            buf=(char*)malloc(message_length*sizeof(char));
            error = MPI_Bcast(buf, message_length, MPI_CHAR, rank, comm);
            ASSERTMPIERROR(error,"MPISendBroadcast failed!");

            {
                boost::iostreams::basic_array_source<char> device(buf, message_length);
                boost::iostreams::stream<boost::iostreams::basic_array_source<char> > s(device);
                boost::archive::binary_iarchive ia(s);
                // read class state from archive
                ia >> t;
            }

            delete buf;
    };


    /**
        Important struct to define all MPI message tags used in this framework!
    */
    class MPIMessageTags{
        public:

        template<typename T> struct type{};

        template<typename T> unsigned int getTag(){
            getTag(type<T>());
        };


        private:
            unsigned int getTag(type<std::string>){
                return STDSTRING;
            }


            enum {
                RIGIDBODY_NUMBER_CHECK,
                RIGIDBODY_UPDATE_MESSAGE,
                RIGIDBODY_MESSAGE,
                CONTACT_UPDATE_MESSAGE,
                STDSTRING
            };
    };

};

#endif
