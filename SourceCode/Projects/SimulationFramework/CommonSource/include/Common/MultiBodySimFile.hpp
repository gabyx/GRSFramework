#ifndef MultiBodySimFile_hpp
#define MultiBodySimFile_hpp

#include <boost/filesystem.hpp>
#include <boost/static_assert.hpp>

#include "DynamicsState.hpp"

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
* @brief This is the input output class for writting a binary multi body system file to the disk.
* It is used to save all simulated data.
* The layout of the document is as follows:
* - The first #SIM_FILE_SIGNATURE_LENGTH bytes are for the signature of the file.
* - 3 doubles: nSimBodies, nDOFqObj, nDofuObj.
* - DynamicsState:
*     - double --> Time in [seconds]
*     - nSimBodies*nDOFqObj doubles --> generalized coordinates of all bodies.
*     - nSimBodies*nDOFuObj doubles --> generalized velocities of all bodies.
*/
template<typename TLayoutConfig>
class MultiBodySimFile {
public:

  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

  MultiBodySimFile();
  ~MultiBodySimFile();

  /**
  * @brief Opens a .sim file for read only.
  * @param file_path The path to the file to open.
  * @param nSimBodies The number of bodies which should be included in the file.
  * @return true if the file is successfully opened and readable and false if not.
  */
  bool openSimFileRead( const boost::filesystem::path & file_path,   const unsigned int nSimBodies, bool readFullState = true);
  /**
  * @brief Opens a .sim file for write only.
  * @param file_path The path to the file to open.
  * @param nSimBodies The number of bodies which should be included in the file.
  * @return true if the file is successfully opened and writable and false if not.
  */
  bool openSimFileWrite( const boost::filesystem::path & file_path,   const unsigned int nSimBodies,  bool truncate = true);

  /**
  * @brief Closes the .sim file which was opened by an openSimFileWrite or openSimFileRead command.
  */
  void closeSimFile();

  /**
  * @brief Checks if there are still dynamics states to read or not.
  * @return true if there are still dynamics states to read, false if the file end has been reached.
  */
  bool isGood();

  bool writeOutAllStateTimes();

  /**
  * @brief Operator to write a state to the file, writes position and velocity!
  */
  MultiBodySimFile<TLayoutConfig> & operator << (const DynamicsState<TLayoutConfig>* state);
  /**

  * @brief Operator to read a state from a file, only reads position!
  */
  MultiBodySimFile<TLayoutConfig> & operator >> (DynamicsState<TLayoutConfig>* state);
  /**
  * @brief Operator to read a state from a file, reads position and velocity!
  */
  MultiBodySimFile<TLayoutConfig> & operator >> (DynamicsState<TLayoutConfig>& state);

  /**
  * @brief Operator to read a state from a file, reads position and velocity!
  */
  MultiBodySimFile<TLayoutConfig> & operator >> (boost::shared_ptr<DynamicsState<TLayoutConfig> > &state);


  /**
  * @brief Gets the state at the time t.
  */
  //bool getStateAt(DynamicsState<TLayoutConfig>& state, PREC t);
  void getEndState(DynamicsState<TLayoutConfig>& state);

  unsigned int getNStates(); ///< Gets the number of states in a read only .sim file.

  std::string getErrorString(){ return m_errorString.str();}

private:
   /**
   * @brief Operator to write a generic value to the file as binary data.
   */
  template<typename T>
  MultiBodySimFile<TLayoutConfig> & operator << (const T &value);
   /**
   * @brief Operator to read a generic value from a .sim file as binary data.
   */
  template<typename T>
  MultiBodySimFile<TLayoutConfig> & operator >> (T &value);

  std::fstream m_file_stream;                      ///< The file stream which represents the binary data.
  enum BuffSize{ BUF_SIZE = 1<<14 };               ///< The internal buffer size.
  char * m_Buffer;                                 ///< The buffer.

  static const char m_simHeader[SIM_FILE_SIGNATURE_LENGTH]; ///< The .sim file header.



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
  std::streamoff m_nBytesPerState, m_nBytesPerQ, m_nBytesPerU;
  unsigned long long int m_nSimBodies;
  /** @}*/

  std::streampos m_beginOfStates;

  static const std::streamoff m_nBytesPerQObj = NDOFqObj*sizeof(double);
  static const  std::streamoff m_nBytesPerUObj = NDOFuObj*sizeof(double);
  static const  std::streamoff m_headerLength = (3*sizeof(unsigned int) + SIM_FILE_SIGNATURE_LENGTH*sizeof(char));

  bool m_bReadFullState;

  std::stringstream m_errorString;

  // Copy constructor is private, we should ne copy the file, because fstream is not copiable
  MultiBodySimFile<TLayoutConfig> & operator =(const MultiBodySimFile<TLayoutConfig> & file);


};
/** @} */

template<typename TLayoutConfig>
template<typename T>
MultiBodySimFile<TLayoutConfig> & MultiBodySimFile<TLayoutConfig>::operator<<( const T &value )
{
  m_file_stream.write(
    reinterpret_cast<char const *>(&value),
    sizeof(value)
    );
  return *this;
};

template<typename TLayoutConfig>
template<typename T>
MultiBodySimFile<TLayoutConfig> & MultiBodySimFile<TLayoutConfig>::operator>>( T &value )
{
  m_file_stream.read(
    reinterpret_cast<char *>(&value),
    sizeof(value)
    );
  return *this;
};

template<typename TLayoutConfig> const char MultiBodySimFile<TLayoutConfig>::m_simHeader[SIM_FILE_SIGNATURE_LENGTH] = SIM_FILE_SIGNATURE;



// Implementation

template<typename TLayoutConfig>
 MultiBodySimFile<TLayoutConfig>::MultiBodySimFile()
{
  m_nStates = 0;
  m_nBytes = 0;
  m_nBytesPerState =0;
  m_nBytesPerU =0;
  m_nBytesPerQ = 0;
  m_nSimBodies = 0;

  m_filePath = boost::filesystem::path();

  m_Buffer = new char[BUF_SIZE];

  m_errorString.str("");
}
template<typename TLayoutConfig>
 MultiBodySimFile<TLayoutConfig>::~MultiBodySimFile()
{
   closeSimFile();
   delete[] m_Buffer;
}

 template<typename TLayoutConfig>
 bool MultiBodySimFile<TLayoutConfig>::isGood()
 {
   if(m_file_stream.good())
   {
      if( (m_nBytes - m_file_stream.tellg() ) >= ( m_nBytesPerState )  ){
       return true;
     }
   }
 return false;
 }

template<typename TLayoutConfig>
bool MultiBodySimFile<TLayoutConfig>::writeOutAllStateTimes()
{
   using namespace std;
   m_errorString.str("");

   std::fstream file;
   file.close();
   boost::filesystem::path new_path = m_filePath.parent_path();
   new_path /= "SimFileInfo.txt";
   file.open(new_path.string().c_str(), std::ios::trunc | std::ios::out);
   if(file.good()){
      file << "# Simfile Information for Path: " << m_filePath.string() <<endl;

      m_file_stream.seekg(m_beginOfStates);
      file << "# The following state times are in the sim file: " << m_filePath.string() <<endl;
      while(isGood()){
        double t;
        *this >> (double &)t;
        file << "m_t: " << t <<endl;
        m_file_stream.seekg(m_nBytesPerQ,ios_base::cur);
        m_file_stream.seekg(m_nBytesPerU,ios_base::cur);
      }
      file.close();
      return true;
   }

   m_errorString << "Could not open text file: " << new_path.string()<<endl;

   return false;
}

template<typename TLayoutConfig>
void  MultiBodySimFile<TLayoutConfig>::setByteLengths(const unsigned int nSimBodies){
  m_nBytesPerState = (nSimBodies*(NDOFqObj+NDOFuObj) + 1) *sizeof(double);
  m_nBytesPerU = nSimBodies*NDOFuObj*sizeof(double);
  m_nBytesPerQ = nSimBodies*NDOFqObj*sizeof(double);
  m_nSimBodies = nSimBodies;
}

template<typename TLayoutConfig>
bool  MultiBodySimFile<TLayoutConfig>::openSimFileWrite(const boost::filesystem::path &file_path, const unsigned int nSimBodies, bool truncate)
{
  m_errorString.str("");

  closeSimFile();

  setByteLengths(nSimBodies);

  if(truncate){

      m_file_stream.open(file_path.string().c_str(), std::ios_base::trunc | std::ios_base::binary | std::ios_base::out);
      m_file_stream.rdbuf()->pubsetbuf(m_Buffer, BUF_SIZE);
      if(m_file_stream.good())
      {
         writeHeader();
         m_filePath = file_path;
         return true;
      }

      m_errorString << "Could not open and truncate sim file: " << file_path.string();

  }else{
      //Here we need to check that, we set the put position exactly at the last state, we might have overhanging bits  because user has cancled and some binary stuff is hanging at the end!!!
     if(openSimFileRead(file_path,nSimBodies)){
         // Reopen only in write mode!
         m_file_stream.close();
         m_file_stream.clear();
         //TODO Why here ::in mode?? Reason?
         m_file_stream.open(file_path.string().c_str() , (std::ios_base::binary | std::ios_base::out  | std::ios_base::in ) );
         if(m_file_stream.good())
         {
            //Set the put pointer!
            m_file_stream.seekp(m_beginOfStates);
            m_file_stream.seekp( m_nStates*m_nBytesPerState ,std::ios_base::cur);
            m_filePath = file_path;

            return true;
         }
     }

     m_errorString << "Could not open sim file: " << file_path.string() <<" for appending data"<<std::endl;
  }

  closeSimFile();

  return false;
}

template<typename TLayoutConfig>
void  MultiBodySimFile<TLayoutConfig>::writeHeader()
{

  for(int i=0;i<SIM_FILE_SIGNATURE_LENGTH;i++){
    *this << m_simHeader[i];
  }

  *this << (unsigned int)m_nSimBodies << (unsigned int)NDOFqObj << (unsigned int)NDOFuObj; // Precision output is always double!


  m_beginOfStates = m_file_stream.tellp();
}


template<typename TLayoutConfig>
 MultiBodySimFile<TLayoutConfig> &  MultiBodySimFile<TLayoutConfig>::operator<<( const DynamicsState<TLayoutConfig>* state )
{
  // write time
  *this << (double)state->m_t;
  // write states
  for(unsigned int i=0 ; i< state->m_nSimBodies; i++){
    for(int k=0; k < NDOFqObj; k++){
      *this << (double)(state->m_SimBodyStates[i].m_q(k));
    }
    for(int k=0; k < NDOFuObj; k++){
      *this << (double)(state->m_SimBodyStates[i].m_u(k));
    }
  }

  m_nStates++;

  return *this;
}

template<typename TLayoutConfig>
 MultiBodySimFile<TLayoutConfig> &  MultiBodySimFile<TLayoutConfig>::operator>>( DynamicsState<TLayoutConfig>* state )
{
  // write time
  *this >> (double &)state->m_t;
  //std::cout << "m_t:"<< state->m_t <<std::endl;

  state->m_StateType = DynamicsState<TLayoutConfig>::NONE;
  // write states
  for(unsigned int i=0 ; i< state->m_nSimBodies; i++){
    //std::cout << "q_"<<i<<": ";
    for(int k=0; k < NDOFqObj; k++){
      *this >> (double &)(state->m_SimBodyStates[i].m_q(k));
      //std::cout << "q"<<i <<state->m_SimBodyStates[i].m_q(k)  << std::endl;
      }
      //std::cout<< state->m_SimBodyStates[i].m_q.transpose()  << std::endl;
    if(m_bReadFullState){
        //std::cout << "u_"<<i<<": ";
       for(int k=0; k < NDOFuObj; k++){
         *this >> (double &)(state->m_SimBodyStates[i].m_u(k));
       }
       //std::cout<< state->m_SimBodyStates[i].m_u.transpose()  << std::endl;
    }else{
        //Dont read in velocities, its not needed!
        m_file_stream.seekg(m_nBytesPerUObj,std::ios_base::cur);
    }


  }
  return *this;
}

template<typename TLayoutConfig>
 MultiBodySimFile<TLayoutConfig> &  MultiBodySimFile<TLayoutConfig>::operator>>( DynamicsState<TLayoutConfig> & state )
{
  return this->operator>>(&state);
}

template<typename TLayoutConfig>
MultiBodySimFile<TLayoutConfig> &  MultiBodySimFile<TLayoutConfig>::operator>>( boost::shared_ptr<DynamicsState<TLayoutConfig> > & state )
{
  return this->operator>>(state.get());
}

template<typename TLayoutConfig>
bool  MultiBodySimFile<TLayoutConfig>::openSimFileRead(const boost::filesystem::path &file_path, const unsigned int nSimBodies, bool readFullState)
{
   m_errorString.str("");

   // Set if the read commands are reading the whole state! not only q! also u!
   m_bReadFullState = readFullState;

  closeSimFile();

  setByteLengths(nSimBodies);

  m_file_stream.open(file_path.string().c_str(), std::ios_base::binary | std::ios_base::in);
  m_file_stream.rdbuf()->pubsetbuf(m_Buffer, BUF_SIZE);
  m_file_stream.sync();
  if(m_file_stream.good()){

    // Read length
    if(readLength()){
      if(readHeader()){

        //Set the get pointer!
        m_file_stream.seekg(m_beginOfStates);
        m_filePath = file_path;
        return true;
      }
    }

  }

  m_errorString << "Could not open sim file: " << file_path.string()<<std::endl;

  closeSimFile();
  return false;
}


template<typename TLayoutConfig>
void MultiBodySimFile<TLayoutConfig>::closeSimFile()
{
   // Reset all values;
   m_nStates = 0;
   m_nBytes = 0;
   m_nBytesPerState = 0;
   m_nBytesPerU = 0;
   m_nBytesPerQ = 0;
   m_nSimBodies = 0;

   m_filePath = boost::filesystem::path();

  if(m_file_stream.is_open()){
    m_file_stream.close();
    m_file_stream.clear();
    // Flush the buffer!
  }

}

template<typename TLayoutConfig>
bool  MultiBodySimFile<TLayoutConfig>::readLength()
{
using namespace std;
  m_file_stream.seekg(0, ios::end);
  m_nBytes = (std::streamoff)m_file_stream.tellg();
  m_file_stream.seekg(0, ios::beg);

  //TODO
  std::cout << m_nBytes << "," << m_headerLength<<","<<m_nBytesPerState<<std::endl;
  if(m_nBytes > m_headerLength){
     long long int nStates = (m_nBytes - m_headerLength) / ( m_nBytesPerState );
     //cout << "States:" << (unsigned int) nStates << std::endl;
     if(nStates > 0){
       m_nStates = nStates;
       return true;
     }else{
        m_errorString << "Number of states: " << nStates<<" , binary file is corrupt!" <<std::endl;
     }
  }else{
      m_errorString << "Binary file contains no data, probably only header!" <<std::endl;
  }
  return false;
}
template<typename TLayoutConfig>
bool  MultiBodySimFile<TLayoutConfig>::readHeader()
{
  char signature[4];
  m_file_stream.read(signature,4);
  if(std::strncmp(signature,m_simHeader,SIM_FILE_SIGNATURE_LENGTH)==0){
    unsigned int nBodies, nDofqObj, nDofuObj;
    *this >> nBodies >> nDofqObj >> nDofuObj;

        bool abort;
        if(m_bReadFullState){
            abort = nBodies == m_nSimBodies && nDofuObj == NDOFuObj && nDofqObj == NDOFqObj;
        }else{
            abort = nBodies == m_nSimBodies && nDofqObj == NDOFqObj;
        }

        if(abort){
           m_beginOfStates = m_file_stream.tellg();
          return true;
        }else{
           m_errorString << "Binary file does not correspond to the number of bodies which should be simulated: "<< std::endl
              <<" Binary File describes: \tnSimBodies = "<<nBodies<< "\tnDofqObj = "<<nDofqObj<<"\tnDofuObj = " << nDofuObj << std::endl
              <<" Simulation requests: \t\tnSimBodies = "<<m_nSimBodies<< "\tnDofqObj = "<<NDOFqObj<<"\tnDofuObj = " << NDOFuObj<<std::endl;
        }

  }else{
      m_errorString << "Binary file contains a wrong header and is not equal to: '" << m_simHeader<<"'"<<std::endl;
  }

  return false;
}


//template<typename TLayoutConfig>
//bool MultiBodySimFile<TLayoutConfig>::getStateAt(DynamicsState<TLayoutConfig>& state, PREC t){
//
//}

template<typename TLayoutConfig>
void MultiBodySimFile<TLayoutConfig>::getEndState(DynamicsState<TLayoutConfig>& state){
      m_file_stream.seekg(m_beginOfStates);
      m_file_stream.seekg( (m_nStates-1)*m_nBytesPerState ,std::ios_base::cur);
      this->operator>>(&state);
      m_file_stream.seekg(m_beginOfStates);
}

template<typename TLayoutConfig>
unsigned int MultiBodySimFile<TLayoutConfig>::getNStates()
{
  return m_nStates;
}


#endif

