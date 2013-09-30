#ifndef MultiBodySimFilePart_hpp
#define MultiBodySimFilePart_hpp

#include <boost/filesystem.hpp>
#include <StaticAssert.hpp>

#include <type_traits>
#include <fstream>

#include "MultiBodySimFileIOHelpers.hpp"

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
#define SIM_FILE_PART_SIGNATURE_PART {'M','B','S','P'}

/**
* @brief Defines the extentsion of the file.
*/
#define SIM_FILE_PART_EXTENSION ".simpart"


/**
* @brief This is the input output class for writting a binary multi body system file to the disk.
* It is used to save partial simulated data for one process.
* The layout of the document is as follows:
* - The first #SIM_FILE_SIGNATURE_LENGTH bytes are for the signature of the file.
* - 2 doubles: nDOFqObj, nDofuObj.
*     - double --> Time in [seconds]
*       - nDOFqObj doubles --> generalized coordinates of all bodies.
*       - nDOFuObj doubles --> generalized velocities of all bodies.
*/
class MultiBodySimFilePart {
public:


    MultiBodySimFilePart(unsigned int nDOFqObj, unsigned int nDOFuObj,unsigned int bufferSize = 1<<14);
    ~MultiBodySimFilePart();

    /**
    * @brief Opens a .sim file for read only.
    * @param file_path The path to the file to open.
    * @return true if the file is successfully opened and readable and false if not.
    */
    //bool openRead( const boost::filesystem::path & file_path,   const unsigned int nSimBodies, bool readFullState = true);
    /**
    * @brief Opens a .sim file for write only.
    * @param file_path The path to the file to open.
    * @return true if the file is successfully opened and writable and false if not.
    */
    bool openWrite( const boost::filesystem::path & file_path,  bool truncate = true);

    /**
    * @brief Closes the .sim file which was opened by an openWrite or openRead command.
    */
    void close();

    /**
    * @brief Checks if there are still dynamics states to read or not.
    * @return true if there are still dynamics states to read, false if the file end has been reached.
    */
    bool isGood();



    unsigned int getNStates(); ///< Gets the number of states in a read only .sim file.

    std::string getErrorString() {
        m_errorString << " error: " << std::strerror(errno) <<std::endl;
        return m_errorString.str();
    }

private:


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
    /** @}*/

    /** brief File path */
    boost::filesystem::path m_filePath;


    std::streampos m_beginHeader;
    std::streampos m_beginOfStates;

    unsigned int m_nDOFuObj, m_nDOFqObj, unsigned int m_nStates;
    const  std::streamoff m_nBytesPerQObj ;
    const  std::streamoff m_nBytesPerUObj ;
    static const  std::streamoff m_headerLength = (2*sizeof(unsigned int) + SIM_FILE_SIGNATURE_LENGTH*sizeof(char));

    std::stringstream m_errorString;

    // Copy constructor is private, we should never copy the file, because fstream is not copiable
    MultiBodySimFilePart & operator =(const MultiBodySimFilePart & file);


};
/** @} */



#endif


