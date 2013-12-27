#include "MultiBodySimFile.hpp"


const char MultiBodySimFile::m_simFileSignature[SIM_FILE_SIGNATURE_LENGTH] = SIM_FILE_SIGNATURE;


const std::streamoff MultiBodySimFile::m_nAdditionalBytesPerBody= setAdditionalBytes();
// Implementation

MultiBodySimFile::MultiBodySimFile(unsigned int nDOFqBody, unsigned int nDOFuBody, unsigned int bufferSize):
    m_nBytesPerQBody(nDOFqBody*sizeof(double)),
    m_nBytesPerUBody(nDOFuBody*sizeof(double)),
    m_nBytesPerBody(nDOFuBody*sizeof(double) + nDOFqBody*sizeof(double) + sizeof(RigidBodyIdType) + m_nAdditionalBytesPerBody),
    m_nDOFuBody(nDOFuBody),
    m_nDOFqBody(nDOFqBody),
    m_nStates(0),
    m_nBytesPerState(0),
    m_nBytes(0),
    m_nSimBodies(0)
{
    m_filePath = boost::filesystem::path();

    m_buf_size = bufferSize;
    m_Buffer = new char[m_buf_size];

    m_errorString.str("");
}

MultiBodySimFile::~MultiBodySimFile() {
    close();
    delete[] m_Buffer;
}


bool MultiBodySimFile::isGood() {
    if(m_file_stream.good()) {
        if( (m_nBytes - m_file_stream.tellg() ) >= ( m_nBytesPerState )  ) {
            return true;
        }
    }
    return false;
}


bool MultiBodySimFile::writeOutAllStateTimes() {
    using namespace std;
    m_errorString.str("");

    std::fstream file;
    file.close();
    boost::filesystem::path new_path = m_filePath.parent_path();
    new_path /= "SimFileInfo.txt";
    file.open(new_path.string().c_str(), std::ios::trunc | std::ios::out);
    if(file.good()) {
        file << "# Simfile Information for Path: " << m_filePath.string() <<endl;

        m_file_stream.seekg(m_beginOfStates);
        file << "# The following state times are in the sim file: " << m_filePath.string() <<endl;
        while(isGood()) {
            double t;
            *this >> (double &)t;
            file << "m_t: " << t <<endl;
            m_file_stream.seekg(m_nBytesPerBody * m_nSimBodies,ios_base::cur);
        }
        file.close();
        return true;
    }

    m_errorString << "Could not open text file: " << new_path.string()<< std::endl;

    return false;
}


void MultiBodySimFile::setByteLengths(const unsigned int nSimBodies) {
    m_nBytesPerState = nSimBodies * m_nBytesPerBody + 1*sizeof(double);
    m_nSimBodies = nSimBodies;
}


bool MultiBodySimFile::openWrite(const boost::filesystem::path &file_path, const unsigned int nSimBodies, bool truncate) {
    m_errorString.str("");

    close();

    setByteLengths(nSimBodies);

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
        //Here we need to check that, we set the put position exactly at the last state, we might have overhanging bits  because user has cancled and some binary stuff is hanging at the end!!!
        if(openRead(file_path,nSimBodies)) {
            // Reopen only in write mode!
            m_file_stream.close();
            m_file_stream.clear();
            //TODO Why here ::in mode?? Reason?
            m_file_stream.open(file_path.string().c_str() , (std::ios_base::binary | std::ios_base::out  | std::ios_base::in ) );
            m_file_stream.rdbuf()->pubsetbuf(m_Buffer, m_buf_size);
            if(m_file_stream.good()) {
                //Set the put pointer!
                m_file_stream.seekp(m_beginOfStates);
                m_file_stream.seekp( m_nStates*m_nBytesPerState ,std::ios_base::cur);
                m_filePath = file_path;

                return true;
            }
        }

        m_errorString << "Could not open sim file: " << file_path.string() <<" for appending data" <<std::endl;
    }

    close();

    return false;
}


void  MultiBodySimFile::writeHeader() {

    for(int i=0; i<SIM_FILE_SIGNATURE_LENGTH; i++) {
        *this << m_simFileSignature[i];
    }
    *this << (unsigned int)SIM_FILE_VERSION;
    *this << (unsigned int)m_nSimBodies << (unsigned int)m_nDOFqBody << (unsigned int)m_nDOFuBody; // Precision output is always double!
    *this << (unsigned int)m_additionalBytesType;

    m_beginOfStates = m_file_stream.tellp();
}


bool  MultiBodySimFile::openRead(const boost::filesystem::path &file_path, const unsigned int nSimBodies, bool readFullState) {
    m_errorString.str("");

    // Set if the read commands are reading the whole state! not only q! also u!
    m_bReadFullState = readFullState;

    close();

    setByteLengths(nSimBodies);

    m_file_stream.open(file_path.string().c_str(), std::ios_base::binary | std::ios_base::in);
    m_file_stream.rdbuf()->pubsetbuf(m_Buffer, m_buf_size);
    m_file_stream.sync();
    if(m_file_stream.good()) {

        // Read length
        if(readLength()) {
            if(readHeader()) {

                //Set the get pointer!
                m_file_stream.seekg(m_beginOfStates);
                m_filePath = file_path;
                return true;
            }
        }

    }

    m_errorString << "Could not open sim file: " << file_path.string() <<std::endl;

    close();
    return false;
}



void MultiBodySimFile::close() {
    // Reset all values;
    m_nStates = 0;
    m_nBytes = 0;
    m_nBytesPerState = 0;
    m_nSimBodies = 0;

    m_filePath = boost::filesystem::path();

    if(m_file_stream.is_open()) {
        m_file_stream.close();
        m_file_stream.clear();
        // Flush the buffer!
    }

}


bool  MultiBodySimFile::readLength() {
    using namespace std;
    m_file_stream.seekg(0, ios::end);
    m_nBytes = (std::streamoff)m_file_stream.tellg();
    m_file_stream.seekg(0, ios::beg);


    if(m_nBytes > m_headerLength) {
        long long int nStates = (m_nBytes - m_headerLength) / ( m_nBytesPerState );
        //cout << "States:" << (unsigned int) nStates << std::endl;
        if(nStates > 0) {
            m_nStates = nStates;
            return true;
        } else {
            m_errorString << "Number of states: " << nStates<<" , binary file is corrupt!" <<std::endl;
        }
    } else {
        m_errorString << "Binary file contains no data, probably only header!" <<std::endl;
    }
    return false;
}

bool  MultiBodySimFile::readHeader() {
    char signature[SIM_FILE_SIGNATURE_LENGTH];
    m_file_stream.read(signature,SIM_FILE_SIGNATURE_LENGTH);
    if(std::strncmp(signature,m_simFileSignature,SIM_FILE_SIGNATURE_LENGTH)==0) {

        unsigned int version;
        unsigned int  nBodies, nDofqBody, nDofuBody, addBytesType;
        *this >> version;

        if(version != SIM_FILE_VERSION){
            m_errorString << " Binary file has version: "<< version << " which does not fit the compiled version: " << SIM_FILE_VERSION << std::endl;
            return false;
        }

        *this >> nBodies >> nDofqBody >> nDofuBody >> addBytesType;

        if( addBytesType != m_additionalBytesType){
            m_errorString << " Binary file does not have additionalBytesType: " <<
            addBytesType << " equal to " <<m_additionalBytesType << std::endl;
            return false;
        }

        bool abort;
        if(m_bReadFullState) {
            abort = nBodies == m_nSimBodies && nDofuBody == m_nDOFuBody && nDofqBody == m_nDOFqBody;
        } else {
            abort = nBodies == m_nSimBodies && nDofqBody == m_nDOFqBody;
        }

        if(abort) {
            m_beginOfStates = m_file_stream.tellg();
            return true;
        } else {
            m_errorString <<" Binary file does not correspond to the number of bodies which should be simulated: "<< std::endl
                          <<" Binary File describes: \tnSimBodies = "<<nBodies<< "\tnDofqBody = "<<nDofqBody<<"\tnDofuBody = " << nDofuBody << std::endl
                          <<" Simulation requests: \t\tnSimBodies = "<<m_nSimBodies<< "\tnDofqBody = "<<m_nDOFqBody<<"\tnDofuBody = " << m_nDOFuBody<<std::endl;
        }

    } else {
        m_errorString << "Binary file contains a wrong header and is not equal to: '" << m_simFileSignature<<"'"<<std::endl;
    }

    return false;
}

unsigned int MultiBodySimFile::getNStates() {
    return m_nStates;
}


