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


    /**
    *    Important struct to define all MPI message tags used in this framework!
    */
    class MPIMessageTags{
        public:

        template<typename T>
        static unsigned int getTag(){
            getTag(type<T>());
        };

        private:
            template<typename T> struct type{};

            static unsigned int getTag(type<std::string>){
                return STDSTRING;
            }

            enum {
                RIGIDBODY_NUMBER_CHECK,
                RIGIDBODY_UPDATE_MESSAGE,
                RIGIDBODY_MESSAGE,
                CONTACT_UPDATE_MESSAGE,
                STDSTRING,
                GENERIC_STRING_MESSAGE,
                CURRENT_SIMFOLDER_MESSAGE
            };
    };


    /**
    * Composer which serilaizes a message: take care sending bytes and receiving them need to make sure that the same endianess is used in the network!
    */
    template<typename T>
    class MessageComposer{
        public:

            MessageComposer(std::size_t reserve_bytes){
                m_serial_str.reserve(reserve_bytes);
            }

            MessageComposer & operator<<(const T & t){
                m_serial_str.clear(); //Clear serializable string, no allocation if we push less or equal as much into the string next time!
                boost::iostreams::back_insert_device<std::string> inserter(m_serial_str);
                boost::iostreams::stream< boost::iostreams::back_insert_device<std::string> > s(inserter);
                boost::archive::binary_oarchive oa(s);
                oa << t;
                s.flush();
            };



            MPI_Datatype getMPIDataType(){return MPI_CHAR;}
            const  char * getDataPtr(){return m_serial_str.data();}
            std::size_t getSize(){return m_serial_str.size();}
            unsigned int getMPITag(){return MPIMessageTags::getTag<T>();}

        private:
        std::string m_serial_str;
    };

    /**
    * Decomposer for serialized Messages: take care sending bytes and receiving them need to make sure that the same endianess is used in the network!
    */
     template<typename T>
     class MessageDecomposer{
        public:

            MessageDecomposer(std::size_t reserve_bytes){
              m_sizeMessage = 0;
              m_sizeTotal = 0;
              reserveMessageSize(reserve_bytes);
            }


           MessageDecomposer & operator>>(T & t){

                boost::iostreams::basic_array_source<char> device(m_buffer, m_sizeMessage);
                boost::iostreams::stream<boost::iostreams::basic_array_source<char> > s(device);
                boost::archive::binary_iarchive ia(s);
                // read class state from archive
                ia >> t;
             };

            MPI_Datatype getMPIDataType(){return MPI_CHAR;}
            char * getDataPtr(){return m_buffer;}

            void reserveMessageSize(std::size_t bytes){
                if(bytes > m_sizeTotal){
                    delete m_buffer;
                    m_buffer = new char[bytes];
                    m_sizeMessage = bytes;
                    m_sizeTotal = bytes;
                }else{
                   m_sizeMessage = bytes;
                }
            }

              std::size_t getSize(){return m_sizeMessage;}

            unsigned int getMPITag(){return MPIMessageTags::getTag<T>();}

        private:
        std::size_t m_sizeTotal, m_sizeMessage;
        char * m_buffer;
    };



    template<>
    class MessageComposer<std::string>{
        public:

            MessageComposer(std::size_t reserve_bytes){
            }

            MessageComposer & operator<<(const std::string & t){
                m_serial_str = &t;
            };

            MPI_Datatype getMPIDataType(){return MPI_CHAR;}
            const  char * getDataPtr(){return m_serial_str->data();}
            std::size_t getSize(){return m_serial_str->size();}
            unsigned int getMPITag(){return MPIMessageTags::getTag<std::string>();}

        private:
        const std::string * m_serial_str;
    };

    template<>
     class MessageDecomposer<std::string>{
        public:

            MessageDecomposer(std::size_t reserve_bytes){
              m_sizeMessage = 0;
              m_sizeTotal = 0;
              reserveMessageSize(reserve_bytes);
            }


           MessageDecomposer & operator>>(std::string & t){
              t = std::string(m_buffer,m_sizeMessage);
           };

            MPI_Datatype getMPIDataType(){return MPI_CHAR;}
            char * getDataPtr(){return m_buffer;}

            void reserveMessageSize(std::size_t bytes){
                if(bytes > m_sizeTotal){
                    delete m_buffer;
                    m_buffer = new char[bytes];
                    m_sizeMessage = bytes;
                    m_sizeTotal = bytes;
                }else{
                   m_sizeMessage = bytes;
                }
            }

              std::size_t getSize(){return m_sizeMessage;}

            unsigned int getMPITag(){return MPIMessageTags::getTag<std::string>();}

        private:
        std::size_t m_sizeTotal, m_sizeMessage;
        char * m_buffer;
    };


    template<typename T>
    void MPISendBroadcast(const T & t, unsigned int rank, MPI_Comm comm){

        static MessageComposer<T> message(256);

        message << t ;

        int size = message.getSize();

        int error = MPI_Bcast(&(size), 1 , MPI_INT, rank, comm); // First send size, because we cannot probe on the other side!! Collective Communication
        ASSERTMPIERROR(error,"MPISendBroadcast failed!");
        MPI_Bcast(const_cast<char*>(message.getDataPtr()), message.getSize(), message.getMPIDataType(), rank, comm);
        ASSERTMPIERROR(error,"MPISendBroadcast failed!");
    };

    template<typename T>
    void MPIReceiveBroadcast(T & t, unsigned int rank, MPI_Comm comm){

            static MessageDecomposer<T> message(256);
            int message_length;
            int error = MPI_Bcast(&message_length, 1 , MPI_INT, rank, comm);
            ASSERTMPIERROR(error,"MPISendBroadcast failed!");
            message.reserveMessageSize(message_length);
            error = MPI_Bcast(message.getDataPtr(), message.getSize(), message.getMPIDataType(), rank, comm);
            ASSERTMPIERROR(error,"MPISendBroadcast failed!");
            message >> t;
    };



};

#endif
