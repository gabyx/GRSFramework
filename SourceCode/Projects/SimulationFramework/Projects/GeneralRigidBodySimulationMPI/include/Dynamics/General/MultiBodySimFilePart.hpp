#ifndef MultiBodySimFilePart_hpp
#define MultiBodySimFilePart_hpp

#include <boost/filesystem.hpp>
#include <StaticAssert.hpp>

#include <type_traits>
#include <fstream>

#include "StaticAssert.hpp"

#include "MultiBodySimFileIOHelpers.hpp"

#include "RigidBodyContainer.hpp"

/**
* @ingroup Common
* @defgroup MultiBodySimFilePart
* @{
*/

/**
* @brief Defines for the signature length of the .sim file.
*/
#define SIM_FILE_PART_SIGNATURE_LENGTH 4
/**
* @brief Defines for the signature of the .sim file.
*/
#define SIM_FILE_PART_SIGNATURE {'M','B','S','P'}

#define SIM_FILE_PART_VERSION 2

/**
* @brief Defines the extentsion of the file.
*/
#define SIM_FILE_PART_EXTENSION ".simpart"


/**
* @brief This is the input output class for writting a binary multi body system file to the disk.
* It is used to save partial simulated data for one process.
* The layout of the document is as follows:
* - The first #SIM_FILE_SIGNATURE_LENGTH bytes are for the signature of the file.
* - 2 doubles: nDOFqBody, nDofuBody.
*     - double --> Time in [seconds]
*       - nDOFqBody doubles --> generalized coordinates of all bodies.
*       - nDOFuBody doubles --> generalized velocities of all bodies.
*/
class MultiBodySimFilePart {
public:


    MultiBodySimFilePart(unsigned int bufferSize = 1<<14);
    ~MultiBodySimFilePart();

    /**
    * @brief Opens a .sim file for read only.
    * @param file_path The path to the file to open.
    * @return true if the file is successfully opened and readable and false if not.
    */
    //bool openRead( const boost::filesystem::path & file_path,   const unsigned int nSimBodies, bool readVelocities = true);
    /**
    * @brief Opens a .sim file for write only.
    * @param file_path The path to the file to open.
    * @return true if the file is successfully opened and writable and false if not.
    */
    bool openWrite(const boost::filesystem::path &file_path,
                                         unsigned int nDOFqBody,
                                         unsigned int nDOFuBody,
                                         bool truncate);

    /**
    * @brief Write all states of the bodies to the file, writes position and velocity!
    */
    template<typename TRigidBodyContainer>
    inline void write(double time, const TRigidBodyContainer & bodyList);

    /**
    * @brief Writes all bodies from begin to end (Iterator  points to a RigidBody pointer) to the file!
    */
    template<typename TBodyIterator>
    void write(double time, TBodyIterator begin, TBodyIterator end);

    /**
    * @brief Closes the .sim file which was opened by an openWrite or openRead command.
    */
    void close();


    std::string getErrorString() {
        m_errorString << " error: " << std::strerror(errno) <<std::endl;
        return m_errorString.str();
    }

private:

    void setByteLengths();

     /**
    * @brief Operator to write a generic value to the file as binary data.
    */
    template<typename T>
    inline MultiBodySimFilePart & operator << (const T &value);
    /**
    * @brief Operator to read a generic value from a .sim file as binary data.
    */
    template<typename T>
    inline MultiBodySimFilePart & operator >> (T &value);


    std::fstream m_file_stream;                      ///< The file stream which represents the binary data.
    unsigned int m_buf_size;                         ///< The internal buffer size.
    char * m_Buffer;                                 ///< The buffer.

    static const char m_simFileSignature[SIM_FILE_PART_SIGNATURE_LENGTH]; ///< The .sim file header.

    /**
    * @brief Writes the header to the file which has been opened.
    */
    void writeHeader();


    std::streamoff m_nBytes;
    /** @}*/

    /** brief File path */
    boost::filesystem::path m_filePath;


    std::streampos m_beginHeader;
    std::streampos m_beginOfStates;

    unsigned int m_nDOFuBody, m_nDOFqBody, m_nStates;
    std::streamoff m_nBytesPerBody ;
    std::streamoff m_nBytesPerQBody ;
    std::streamoff m_nBytesPerUBody ;
    static const  std::streamoff m_headerLength = (2*sizeof(unsigned int) + SIM_FILE_PART_SIGNATURE_LENGTH*sizeof(char));

    // Write addditional bytes, not yet implemented, but the type is written in the header
    unsigned int m_additionalBytesPerBodyType;
    std::streamsize getAdditionalBytesPerBody();
    std::streamsize m_nAdditionalBytesPerBody;

    std::stringstream m_errorString;

    // Copy constructor is private, we should never copy the file, because fstream is not copiable
    MultiBodySimFilePart & operator =(const MultiBodySimFilePart & file);


};
/** @} */




template<typename T>
MultiBodySimFilePart & MultiBodySimFilePart::operator<<( const T &value ) {
    m_file_stream.write(
        reinterpret_cast<char const *>(&value),
        sizeof(value)
    );
    return *this;
};


template<typename T>
MultiBodySimFilePart & MultiBodySimFilePart::operator>>( T &value ) {
    m_file_stream.read(
        reinterpret_cast<char *>(&value),
        sizeof(value)
    );
    return *this;
};

template<typename TRigidBodyContainer>
void MultiBodySimFilePart::write(double time, const TRigidBodyContainer & bodyList){
    write(time,bodyList.beginOrdered(),bodyList.endOrdered());
}


template<typename TBodyIterator>
void MultiBodySimFilePart::write(double time, TBodyIterator begin, TBodyIterator end) {
    *this << time;
    *this << (unsigned int) std::distance(begin,end);
    using BodyType = typename std::remove_reference<decltype(*(*begin))>::type;

    STATIC_ASSERTM((std::is_same<double, typename BodyType::PREC>::value),
                   "OOPS! TAKE CARE if you compile here, SIM files can only be read with the PREC precision!");
    auto itEnd = end;
    for(auto it = begin; it != itEnd; ++it) {
        *this << (*it)->m_id;
        IOHelpers::writeBinary(m_file_stream, (*it)->get_q());
        IOHelpers::writeBinary(m_file_stream, (*it)->get_u());

//        AddBytes::write<m_additionalBytesPerBodyType>(m_file_stream);
    }
    m_nStates++;
}



#endif


