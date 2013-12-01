#ifndef MultiBodySimFile_hpp
#define MultiBodySimFile_hpp

#include <boost/filesystem.hpp>
#include <StaticAssert.hpp>

#include <type_traits>
#include <fstream>

#include "DynamicsState.hpp"
#include "RigidBodyContainer.hpp"
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

/**
* @brief Defines the extentsion of the file.
*/
#define SIM_FILE_EXTENSION ".sim"

/**
* @brief Defines the extentsion of the file for an initial sim file.
*/
#define SIM_INIT_FILE_EXTENSION ".sim"

/**
* @brief This is the input output class for writting a binary multi body system file to the disk.
* It is used to save all simulated data.
* The layout of the document is as follows:
* - The first #SIM_FILE_SIGNATURE_LENGTH bytes are for the signature of the file.
* - 3 doubles: nSimBodies, nDOFqObj, nDofuObj.
* - DynamicsState:
*     - double --> Time in [seconds]
*       - all states for all nSimBodies
*         - ...
*         - nDOFqObj doubles --> generalized coordinates of all bodies.
*         - nDOFuObj doubles --> generalized velocities of all bodies.
*         - ...
*/
class MultiBodySimFile {
public:


    MultiBodySimFile(unsigned int nDOFqObj, unsigned int nDOFuObj,unsigned int bufferSize = 1<<14);
    ~MultiBodySimFile();

    /**
    * @brief Opens a .sim file for read only.
    * @param file_path The path to the file to open.
    * @param nSimBodies The number of bodies which should be included in the file.
    * @return true if the file is successfully opened and readable and false if not.
    */
    bool openRead( const boost::filesystem::path & file_path,   const unsigned int nSimBodies, bool readFullState = true);
    /**
    * @brief Opens a .sim file for write only.
    * @param file_path The path to the file to open.
    * @param nSimBodies The number of bodies which should be included in the file.
    * @return true if the file is successfully opened and writable and false if not.
    */
    bool openWrite( const boost::filesystem::path & file_path,   const unsigned int nSimBodies,  bool truncate = true);

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
    * @brief Operator to write all states of the bodies to the file, writes position and velocity!
    */
    template<typename TRigidBody>
    inline void write(double time, const RigidBodyContainer<TRigidBody> & bodyList);

    /**
    * @brief Operator to write a state to the file, writes position and velocity!
    */

    inline MultiBodySimFile & operator << (const DynamicsState* state);
    /**
    * @brief Overlaod!
    */

    MultiBodySimFile & operator << (const DynamicsState& state) {
        return this->operator<<(&state);
    }

    /**
    * @brief Operator to read a state from a file.
    */
    inline MultiBodySimFile & operator >> (DynamicsState* state);
    /**
    * @brief Overlaod!
    */

    MultiBodySimFile &  operator>>( DynamicsState & state ) {
        return this->operator>>(&state);
    }
    //  /**
    //  * @brief Gets the state at the time t.
    //  */
    //bool getStateAt(DynamicsState& state, PREC t);
    /**
    * @brief Gets the end state.
    */
    inline void getEndState(DynamicsState& state);

    unsigned int getNStates(); ///< Gets the number of states in a read only .sim file.

    std::string getErrorString() {
        m_errorString << " error: " << std::strerror(errno) <<std::endl;
        return m_errorString.str();
    }

private:
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
    std::streamoff m_nBytes;
    std::streamoff m_nStates;
    /** @}*/

    /** brief File path */
    boost::filesystem::path m_filePath;

    /** @brief Determined from number of bodies! @{*/
    void setByteLengths(const unsigned int nSimBodies);
    std::streamoff m_nBytesPerState; ///< m_nSimBodies*(q,u) + time
    std::streamoff m_nBytesPerQ, m_nBytesPerU;
    unsigned long long int m_nSimBodies;
    /** @}*/

    std::streampos m_beginOfStates;

    unsigned int m_nDOFuObj, m_nDOFqObj;
    const  std::streamoff m_nBytesPerQObj ;
    const  std::streamoff m_nBytesPerUObj ;

    static const  std::streamoff m_headerLength = (3*sizeof(unsigned int) + SIM_FILE_SIGNATURE_LENGTH*sizeof(char));

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


template<typename TRigidBody>
void MultiBodySimFile::write(double time, const RigidBodyContainer<TRigidBody> & bodyList){
    *this << time;
    ASSERTMSG(m_nSimBodies == bodyList.size(),"You try to write "<<bodyList.size()
              <<"bodies into a file which was instanced to hold "<<m_nSimBodies);
    STATIC_ASSERT2((std::is_same<double, typename TRigidBody::PREC>::value),"OOPS! TAKE CARE if you compile here, SIM files can only be read with the PREC precision!")
    for(auto it = bodyList.beginOrdered(); it != bodyList.endOrdered(); it++){
        IOHelpers::writeBinary(m_file_stream, (*it)->get_q());
        IOHelpers::writeBinary(m_file_stream, (*it)->get_u());
    }
}

//
MultiBodySimFile &  MultiBodySimFile::operator<<( const DynamicsState* state ) {


    ASSERTMSG(m_nSimBodies == state->m_nSimBodies,
              "You try to write "<<state->m_nSimBodies<<"bodies into a file which was instanced to hold "<<m_nSimBodies);

    // write time
    *this << (double)state->m_t;
    // write states
    for(unsigned int i=0 ; i< state->m_nSimBodies; i++) {
        STATIC_ASSERT2((std::is_same<double, typename DynamicsState::PREC>::value),
                       "OOPS! TAKE CARE if you compile here, SIM files can only be read with the PREC precision!")

//        for(int k=0; k < m_nDOFqObj; k++) {
//            *this << (double)(state->m_SimBodyStates[i].m_q(k));
//        }
//        m_file_stream << state->m_SimBodyStates[i].m_q; //ADL fails because Eigen has an overload already for fstream
//        m_file_stream << state->m_SimBodyStates[i].m_u;

        IOHelpers::writeBinary(m_file_stream, state->m_SimBodyStates[i].m_q );
        IOHelpers::writeBinary(m_file_stream, state->m_SimBodyStates[i].m_u );
    }

    m_nStates++;

    return *this;
}


MultiBodySimFile &  MultiBodySimFile::operator>>( DynamicsState* state ) {


    // write time
    *this >> (double &)state->m_t;
    //std::cout << "m_t:"<< state->m_t <<std::endl;

    state->m_StateType = DynamicsState::NONE;
    // write states
    for(unsigned int i=0 ; i< state->m_nSimBodies; i++) {

        //m_file_stream >> state->m_SimBodyStates[i].m_q; //ADL fails
        IOHelpers::readBinary(m_file_stream, state->m_SimBodyStates[i].m_q );
        //std::cout<< state->m_SimBodyStates[i].m_q.transpose()  << std::endl;
        if(m_bReadFullState) {
        //m_file_stream >> state->m_SimBodyStates[i].m_u;
        IOHelpers::readBinary(m_file_stream, state->m_SimBodyStates[i].m_u );
            //std::cout<< state->m_SimBodyStates[i].m_u.transpose()  << std::endl;
        } else {
            //Dont read in velocities, its not needed!
            m_file_stream.seekg(m_nBytesPerUObj,std::ios_base::cur);
        }

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

