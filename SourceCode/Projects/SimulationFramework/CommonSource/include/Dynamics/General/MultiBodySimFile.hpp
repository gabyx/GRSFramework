#ifndef MultiBodySimFile_hpp
#define MultiBodySimFile_hpp

#include <type_traits>
#include <set>
#include <fstream>

#include <boost/filesystem.hpp>

#include "StaticAssert.hpp"
#include "RigidBodyId.hpp"
#include "DynamicsState.hpp"

#include "MultiBodySimFileIOHelpers.hpp"

/**
* @ingroup Common
* @defgroup MultiBodySimFile
* @{
*/

/**
* @brief Defines for the signature length of the .sim file.
*/
#define SIM_FILE_SIGNATURE_LENGTH 4
/**
* @brief Defines for the signature of the .sim file.
*/
#define SIM_FILE_SIGNATURE {'M','B','S','F'}

#define SIM_FILE_VERSION 2

/**
* @brief Defines the extentsion of the file.
*/
#define SIM_FILE_EXTENSION ".sim"

/**
* @brief Defines the extentsion of the file for an initial sim file.
*/
#define SIM_INIT_FILE_EXTENSION ".sim"

class SimFileJoiner; ///< Friend declaration inside of MultiBodySimFile

/**
* @brief This is the input output class for writting a binary multi body system file to the disk.
* It is used to save all simulated data.
* The layout of the document is as follows:
* - The first #SIM_FILE_SIGNATURE_LENGTH bytes are for the signature of the file.
* - 3 doubles: nSimBodies, nDOFqBody, nDofuBody.
* - DynamicsState:
*     - double --> Time in [seconds]
*       - all states for all nSimBodies
*         - ...
*         - nDOFqBody doubles --> generalized coordinates of all bodies.
*         - nDOFuBody doubles --> generalized velocities of all bodies.
*         - ...
*/
class MultiBodySimFile {
    friend class SimFileJoiner;
public:


    MultiBodySimFile(unsigned int bufferSize = 1<<14);
    ~MultiBodySimFile();

    /**
    * @brief Opens a .sim file for read only.
    * @param file_path The path to the file to open.
    * @param nSimBodies The number of bodies which should be included in the file.
    * @return true if the file is successfully opened and readable and false if not.
    * @detail if nSimBodies = 0, the sim file does not need to match nSimBodies
    */
    bool openRead(const boost::filesystem::path & file_path,
                  unsigned int nDOFqBody = 0,
                  unsigned int nDOFuBody = 0,
                  unsigned int nSimBodies = 0,
                  bool readFullState = true);
    /**
    * @brief Opens a .sim file for write only.
    * @param file_path The path to the file to open.
    * @param nSimBodies The number of bodies which should be included in the file.
    * @return true if the file is successfully opened and writable and false if not.
    */
    bool openWrite(const boost::filesystem::path & file_path,
                   unsigned int nDOFqBody,
                   unsigned int nDOFuBody,
                   unsigned int nSimBodies,
                   bool truncate = true);

    /**
    * @brief Closes the .sim file which was opened by an openWrite or openRead command.
    */
    void close();

    /**
    * @brief Checks if there are still dynamics states to read or not.
    * @return true if there are still dynamics states to read, false if the file end has been reached.
    */
    bool isGood();

    bool writeOutAllStateTimes();

    /**
    * @brief Write all states of the bodies to the file, writes position and velocity!
    */
    template<typename TRigidBodyContainer>
    inline void write(double time, const TRigidBodyContainer & bodyList);

    /**
    * @brief Reads in the state at the given time and if the id in states is found, the state with id in states is overwritten.
    * @param which is 0 for begin state, 1 for current state given at time, 2 for end state
    * @param time is the input time if variable which is 1, and the output time if variable which is 0 or 2
    * @tparam TBodyStateMap, is a std::map or any hashed map with mapped_type: std::pair<key,value>
    * @detail If time is in between two states in the sim file, then the next bigger is taken!
    *         If the time is greater then the maximum time, then this time is taken.
    */
    template<typename TBodyStateMap>
    bool read(TBodyStateMap & states,
              double & stateTime,
              bool readPos = true,
              bool readVel= true,
              short which = 2,
              bool onlyUpdate = true);


    /**
    * @brief Operator to write a state to the file, writes position and velocity!
    */

    inline MultiBodySimFile & operator << (const DynamicsState* state);
    /**
    * @brief Overlaod!
    */

    inline MultiBodySimFile & operator << (const DynamicsState& state) {
        return this->operator<<(&state);
    }

    /**
    * @brief Operator to read a state from a file.
    */
    inline MultiBodySimFile & operator >> (DynamicsState* state);
    /**
    * @brief Overlaod!
    */

    inline MultiBodySimFile &  operator>>( DynamicsState & state ) {
        return this->operator>>(&state);
    }

    /**
    * @brief Dumps all data from file to this sim file.
    */
    MultiBodySimFile & operator << (MultiBodySimFile& file);


    //  /**
    //  * @brief Gets the state at the time t.
    //  */
    //bool getStateAt(DynamicsState& state, PREC t);
    /**
    * @brief Gets the end state.
    */
    inline void getEndState(DynamicsState& state);

    std::string getErrorString() {
        m_errorString << " strerror: " << std::strerror(errno) <<std::endl;
        return m_errorString.str();
    }

    unsigned int getNDOFq() {
        return m_nDOFqBody;
    }
    unsigned int getNDOFu() {
        return m_nDOFuBody;
    }
    unsigned int getNSimBodies() {
        return m_nSimBodies;
    }
    unsigned int getNStates() {
        return m_nStates;
    }

    std::streamsize getBytesPerState() {
        return m_nBytesPerState;
    }

private:

    bool openWrite_impl(const boost::filesystem::path & file_path,
                   unsigned int nDOFqBody,
                   unsigned int nDOFuBody,
                   unsigned int nSimBodies,
                   unsigned int additionalBytesType,
                   std::streamsize additionalBytesPerBody,
                   bool truncate);

    /**
    * @brief Operator to write a generic value to the file as binary data.
    */
    template<typename T>
    inline MultiBodySimFile & operator << (const T &value);
    /**
    * @brief Operator to read a generic value from a .sim file as binary data.
    */
    template<typename T>
    inline MultiBodySimFile & operator >> (T &value);


    std::fstream m_file_stream;                      ///< The file stream which represents the binary data.
    unsigned int m_buf_size;                         ///< The internal buffer size.
    char * m_Buffer;                                 ///< The buffer.

    static const char m_simFileSignature[SIM_FILE_SIGNATURE_LENGTH]; ///< The .sim file header.

    /**
    * @brief Writes the header to the file which has been opened.
    */
    void writeHeader();
    /**
    * @brief Reads the header from .sim file.
    * @return true if header has been read successfully.
    */
    bool readHeader();
    /**
    * @brief Reads in all the length of the .sim file.
    * @{
    */
    bool readLength();
    std::streamsize m_nBytes;
    std::streamsize m_nStates;
    /** @}*/

    /** @brief File path */
    boost::filesystem::path m_filePath;

    /** @brief Header length */
    static const  std::streamsize m_headerLength = SIM_FILE_SIGNATURE_LENGTH*sizeof(char)
            + sizeof(unsigned int)
            +3*sizeof(unsigned int) +2*sizeof(unsigned int) ; ///< 'MBSF' + nBodies, NDOFq, NDOFu, additionalBytesType (0=nothing, 1 = + process rank, etc.), additionalBytesPerBody



    /** @brief Determined from number of bodies! @{*/
    void setByteLengths();
    std::streamsize m_nBytesPerState; ///< m_nSimBodies*(q,u) + time
    unsigned int m_nSimBodies;
    /** @}*/

    std::streampos m_beginOfStates;

    unsigned int m_nDOFuBody, m_nDOFqBody;
    std::streamsize m_nBytesPerQBody;
    std::streamsize m_nBytesPerUBody;

    // Write mode, Write addditional bytes, not yet implemented, but the type is written in the header
    unsigned int m_additionalBytesType;
    std::streamsize getAdditionalBytes();
    std::streamsize m_nAdditionalBytesPerBody;


    std::streamsize m_nBytesPerBody;

    bool m_bReadFullState;

    std::stringstream m_errorString;

    // Copy constructor is private, we should never copy the file, because fstream is not copiable
    MultiBodySimFile & operator =(const MultiBodySimFile & file);


};
/** @} */





template<typename T>
MultiBodySimFile & MultiBodySimFile::operator<<( const T &value ) {
    m_file_stream.write(
        reinterpret_cast<char const *>(&value),
        sizeof(value)
    );
    return *this;
};


template<typename T>
MultiBodySimFile & MultiBodySimFile::operator>>( T &value ) {
    m_file_stream.read(
        reinterpret_cast<char *>(&value),
        sizeof(value)
    );
    return *this;
};

template<typename TRigidBodyContainer>
void MultiBodySimFile::write(double time, const TRigidBodyContainer & bodyList) {
    *this << time;
    ASSERTMSG(m_nSimBodies == bodyList.size(),"You try to write "<<bodyList.size()
              <<"bodies into a file which was instanced to hold "<<m_nSimBodies);
    STATIC_ASSERT2((std::is_same<double, typename TRigidBodyContainer::PREC>::value),"OOPS! TAKE CARE if you compile here, SIM files can only be read with the PREC precision!")
    auto itEnd = bodyList.endOrdered();
    for(auto it = bodyList.beginOrdered(); it != itEnd; ++it) {
        *this << (*it)->m_id;
        IOHelpers::writeBinary(m_file_stream, (*it)->get_q());
        IOHelpers::writeBinary(m_file_stream, (*it)->get_u());

//        AddBytes::write<m_additionalBytesType>(m_file_stream);
    }
}


template<typename TBodyStateMap>
bool MultiBodySimFile::read(TBodyStateMap & states,
                            double & stateTime,
                            bool readPos,
                            bool readVel,
                            short which,
                            bool onlyUpdate) {
    m_errorString.str("");

    std::set<RigidBodyIdType> updatedStates;
    double currentTime = 0;
    double lastTime =-1;

    RigidBodyState * pState = nullptr;
    RigidBodyIdType id;
    bool timeFound = false;

    m_file_stream.seekg(m_beginOfStates);

    if(stateTime < 0) {
        stateTime = 0;
    }

    if(which == 0) {
        *this >> stateTime;

        timeFound = true;
    } else if(which == 1) {
        //Scan all states
        for( std::streamoff stateIdx = 0; stateIdx < m_nStates; stateIdx++) {
            *this >> currentTime;
            if( ( lastTime > stateTime && stateTime <= currentTime )|| stateTime <= 0.0) {
                timeFound = true;
                break;
            } else {
                lastTime = currentTime;
                if(stateIdx < m_nStates-1) {
                    m_file_stream.seekg( m_nBytesPerState - sizeof(double) ,std::ios_base::cur);
                }
            }
        }
        stateTime = currentTime;
    } else if(which == 2) {
        m_file_stream.seekg( (m_nStates-1)*m_nBytesPerState ,std::ios_base::cur);
        *this >> stateTime;
        timeFound = true;
    }

    if(timeFound) {

        ASSERTMSG(stateTime>=0.0, " Found state time is " << stateTime);
        // Current time found!
        for(unsigned int body = 0; body < m_nSimBodies; body++) {
            *this >> id;

            //std::cout << RigidBodyId::getBodyIdString(id) << std::endl;

            if(onlyUpdate) {
                auto res = states.find(id);
                if(res != states.end()) {
                    pState = &res->second;
                } else {
                    pState = nullptr;
                }
            } else {
                auto res = states[id];
                pState = &res;
            }

            // State found
            if(pState != nullptr ) {
                // Read in state:
                IOHelpers::readBinary(m_file_stream, pState->m_q );
                if(m_bReadFullState) {
                    IOHelpers::readBinary(m_file_stream, pState->m_u );
                } else {
                    m_file_stream.seekg(m_nBytesPerUBody,std::ios_base::cur);
                }

                updatedStates.insert(id);

            } else {
                // State not found
                // Jump body (u and q)
                m_file_stream.seekg( m_nBytesPerQBody+ m_nBytesPerUBody ,std::ios_base::cur);
            }

            // Jump additional bytes
            m_file_stream.seekg(m_nAdditionalBytesPerBody,std::ios_base::cur);
        }

        m_file_stream.seekg(m_beginOfStates);

        if(onlyUpdate) {
            if(updatedStates.size() != states.size()) {
                m_errorString << "Some states have not been updated!; updated states: " << updatedStates.size() <<  std::endl;

                return false;
            }
        }
        return true;
    }

    m_file_stream.seekg(m_beginOfStates);

    m_errorString << "The time type: " << which << " (time: " <<time <<") was not found" << std::endl;
    return false;
}

MultiBodySimFile &  MultiBodySimFile::operator<<( const DynamicsState* state ) {


    ASSERTMSG(m_nSimBodies == state->getNSimBodies(),
              "You try to write "<<state->getNSimBodies()<<"bodies into a file which was instanced to hold "<<m_nSimBodies);

    // write time
    *this << (double)state->m_t;
    // write states
    for(auto & b : state->m_SimBodyStates) {
        STATIC_ASSERT2((std::is_same<double, typename DynamicsState::PREC>::value),
                       "OOPS! TAKE CARE if you compile here, SIM files can only be read with the PREC precision!")
        *this << b.m_id;
        IOHelpers::writeBinary(m_file_stream, b.m_q );
        IOHelpers::writeBinary(m_file_stream, b.m_u );

//        AddBytes::write<m_additionalBytesType>(m_file_stream);
    }

    m_nStates++;

    return *this;
}


MultiBodySimFile &  MultiBodySimFile::operator>>( DynamicsState* state ) {
    auto nSimBodies = state->getNSimBodies();
    ASSERTMSG(m_nSimBodies == nSimBodies,
              "You try to read "<<m_nSimBodies<<"bodies into a state which was instanced to hold "<<nSimBodies);

    // write time
    *this >> (double &)state->m_t;
    //std::cout << "m_t:"<< state->m_t <<std::endl;

    state->m_StateType = DynamicsState::NONE;
    // write states
    RigidBodyIdType id;
    for(unsigned int i=0 ; i< nSimBodies; i++) {

        //m_file_stream >> state->m_SimBodyStates[i].m_q; //ADL fails
        *this >> id;
        unsigned int bodyNr = RigidBodyId::getBodyNr(id);

        ASSERTMSG(bodyNr >=0 && bodyNr < state->m_SimBodyStates.size(), "BodyNr: " << bodyNr << " is out of bound!")
        if(bodyNr < state->m_SimBodyStates.size()) {
            state->m_SimBodyStates[bodyNr].m_id = id;

            IOHelpers::readBinary(m_file_stream, state->m_SimBodyStates[bodyNr].m_q );
            //std::cout<< state->m_SimBodyStates[i].m_q.transpose()  << std::endl;
            if(m_bReadFullState) {
                //m_file_stream >> state->m_SimBodyStates[i].m_u;
                IOHelpers::readBinary(m_file_stream, state->m_SimBodyStates[bodyNr].m_u );
                //std::cout<< state->m_SimBodyStates[i].m_u.transpose()  << std::endl;
            } else {
                //Dont read in velocities, its not needed!
                m_file_stream.seekg(m_nBytesPerUBody,std::ios_base::cur);
            }
        }

        // Dont read in additional bytes
        m_file_stream.seekg(m_nAdditionalBytesPerBody,std::ios_base::cur);

    }
    return *this;
}


void MultiBodySimFile::getEndState(DynamicsState& state) {
    m_file_stream.seekg(m_beginOfStates);
    m_file_stream.seekg( (m_nStates-1)*m_nBytesPerState ,std::ios_base::cur);
    this->operator>>(&state);
    m_file_stream.seekg(m_beginOfStates);
}





#endif

