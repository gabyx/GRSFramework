#ifndef GRSF_Common_BinaryFile_hpp
#define GRSF_Common_BinaryFile_hpp

#include <boost/filesystem.hpp>


/**
* @ingroup Common
* @brief Binary file class to write a simple binary file!
*/
class BinaryFile{
public:

   /**
   * @brief Open a binary file.
   */
   void open(const boost::filesystem::path & file_path, std::ios::openmode Mode){
      if(m_file_stream.is_open()){
         m_file_stream.close();
      }
      m_file_stream.open(file_path.string().c_str(),Mode | std::ios::binary); // make it binary as a guaranty!!
      m_file_stream.clear();
      m_file_stream.rdbuf()->pubsetbuf(m_Buffer, BUF_SIZE);

   }
   /**
   * @brief Close the binary file.
   */
   void close(){
      m_file_stream.close();
   }

   /**
   * @brief Operator to write a generic value to the file as binary data.
   */
  template<typename T>
  BinaryFile & operator << (const T &value){
      m_file_stream.write(
       reinterpret_cast<char const *>(&value),
       sizeof(value)
       );
     return *this;
  }
   /**
   * @brief Operator to read a generic value from file as binary data.
   */
  template<typename T>
  BinaryFile & operator >> (T &value){
     m_file_stream.read(
       reinterpret_cast<char *>(&value),
       sizeof(value)
       );
     return *this;
  }

private:

  std::fstream m_file_stream;                      ///< The file stream which represents the binary data.
  enum BuffSize{ BUF_SIZE = 1<<14 };               ///< The internal buffer size.
  char          m_Buffer[BUF_SIZE];                ///< The buffer.
};


#endif
