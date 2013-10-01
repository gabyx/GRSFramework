#include "MultiBodySimFilePart.hpp"


const char MultiBodySimFilePart::m_simFileSignature[SIM_FILE_PART_SIGNATURE_LENGTH] = SIM_FILE_PART_SIGNATURE;

// Implementation

MultiBodySimFilePart::MultiBodySimFilePart(unsigned int nDOFqObj, unsigned int nDOFuObj, unsigned int bufferSize)
    : m_nBytesPerQObj(nDOFqObj*sizeof(double)), m_nBytesPerUObj(nDOFuObj*sizeof(double)) {
    m_nDOFuObj = nDOFuObj;
    m_nDOFqObj = nDOFqObj;

    m_nBytes = 0;
    m_nStates = 0;

    m_filePath = boost::filesystem::path();

    m_buf_size = bufferSize;
    m_Buffer = new char[m_buf_size];

    m_errorString.str("");
}

MultiBodySimFilePart::~MultiBodySimFilePart() {
    close();
    delete[] m_Buffer;
}



bool MultiBodySimFilePart::openWrite(const boost::filesystem::path &file_path, bool truncate) {
    m_errorString.str("");

    close();

    if(truncate) {

        m_file_stream.open(file_path.string().c_str(), std::ios_base::trunc | std::ios_base::binary | std::ios_base::out);
        m_file_stream.rdbuf()->pubsetbuf(m_Buffer, m_buf_size);
        if(m_file_stream.good()) {
            writeHeader();
            m_filePath = file_path;
            return true;
        }

        m_errorString << "Could not open and truncate sim file: " << file_path.string() <<std::endl;

    } else {
        //Here we need to check that, we set the put position exactly at the end.
        //We could make sure that the position is at the end of one state (unknown # bodies) by checking header for a location
        //But this requires to set a offset always in the header (not done so far)
        m_file_stream.open(file_path.string().c_str(), std::ios_base::app | std::ios_base::binary | std::ios_base::out);
        m_file_stream.rdbuf()->pubsetbuf(m_Buffer, m_buf_size);
        if(m_file_stream.good()) {
            // put location exactly at the end of file
            m_filePath = file_path;
            return true;
        }
        m_errorString << "Could not open sim file: " << file_path.string() <<" for appending data" <<std::endl;
    }

    close();

    return false;
}


void  MultiBodySimFilePart::writeHeader() {

    for(int i=0; i<SIM_FILE_PART_SIGNATURE_LENGTH; i++) {
        *this << m_simFileSignature[i];
    }

    m_beginHeader = m_file_stream.tellp();
    // The number of states will be written at the end when file is closed!
    *this << (unsigned int)m_nStates<< (unsigned int)m_nDOFqObj << (unsigned int)m_nDOFuObj; // Precision output is always double!

    m_beginOfStates = m_file_stream.tellp();
}


void MultiBodySimFilePart::close() {
    // Reset all values;
    m_nBytes = 0;
    m_nStates = 0;
    m_filePath = boost::filesystem::path();

    if(m_file_stream.is_open()) {
        m_file_stream.close();
        m_file_stream.clear();
    }

}

