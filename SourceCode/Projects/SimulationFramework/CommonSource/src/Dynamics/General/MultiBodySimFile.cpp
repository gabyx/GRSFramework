#include "MultiBodySimFile.hpp"


const char MultiBodySimFile::m_simFileSignature[SIM_FILE_SIGNATURE_LENGTH] = SIM_FILE_SIGNATURE;

MultiBodySimFile::MultiBodySimFile(unsigned int bufferSize):
    m_nDOFuBody(0),m_nDOFqBody(0),
    m_nBytes(0),
    m_nBytesPerState(0),
    m_nBytesPerQBody(0),
    m_nBytesPerUBody(0),
    m_additionalBytesPerBodyType(AdditionalBodyData::TypeEnum::NOTHING),
    m_nAdditionalBytesPerBody(0),
    m_nStates(0),
    m_nSimBodies(0),
    m_beginOfStates(0),
    m_readVelocities(true)
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
            m_file_stream.seekg(m_nBytesPerState - sizeof(double),ios_base::cur);
        }
        file.close();
        return true;
    }

    m_errorString << "Could not open text file: " << new_path.string()<< std::endl;

    return false;
}

/** given m_nDOFqBody, m_nDOFuBody, m_nSimBodies, m_nAdditionalBytesPerBody*/
void MultiBodySimFile::setByteLengths() {
    m_nBytesPerQBody = m_nDOFqBody*sizeof(double);
    m_nBytesPerUBody = m_nDOFuBody*sizeof(double);

    m_nBytesPerBody = m_nBytesPerUBody
                    + m_nBytesPerQBody
                    + std::streamsize(sizeof(RigidBodyIdType)) + m_nAdditionalBytesPerBody;

    m_nBytesPerState = m_nSimBodies * m_nBytesPerBody + std::streamsize(sizeof(double));
}

/** Only for writting*/
std::streamoff MultiBodySimFile::getAdditionalBytesPerBody()
{
        return AdditionalBodyData::getAdditionalBytesPerBody(m_additionalBytesPerBodyType);
}

bool MultiBodySimFile::openWrite(const boost::filesystem::path &file_path,
                                 unsigned int nDOFqBody,
                                 unsigned int nDOFuBody,
                                 const unsigned int nSimBodies,
                                 bool truncate)
{
    if(nDOFqBody == 0 || nDOFuBody == 0 || nSimBodies == 0){
        m_errorString << "Wrong openWrite parameters: nDOFqBody:" << nDOFqBody << " nDOFuBody: "<<nDOFuBody << " nSimBodies:" << nSimBodies<< std::endl;
        return false;
    }

    // Determine bytesType, and additional bytes, if necessary

    return openWrite_impl(file_path,
                          nDOFqBody,
                          nDOFuBody,
                          nSimBodies,
                          truncate
                          );
}


bool MultiBodySimFile::openWrite_impl(const boost::filesystem::path &file_path,
                                 unsigned int nDOFqBody,
                                 unsigned int nDOFuBody,
                                 const unsigned int nSimBodies,
                                 bool truncate,
                                 AdditionalBodyData::TypeEnum additionalBytesType,
                                 std::streamsize additionalBytesPerBody) {

    close();

    m_nDOFuBody = nDOFuBody;
    m_nDOFqBody = nDOFqBody;
    m_nAdditionalBytesPerBody = additionalBytesPerBody;
    m_additionalBytesPerBodyType = additionalBytesType;
    m_nSimBodies = nSimBodies;

    setByteLengths();
    m_errorString.str("");


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
        if(openRead(file_path,true,nSimBodies)) {
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

    m_file_stream.seekp(0);

    for(int i=0; i<SIM_FILE_SIGNATURE_LENGTH; i++) {
        *this << m_simFileSignature[i];
    }
    *this << (unsigned int)SIM_FILE_VERSION
    << (unsigned int)m_nSimBodies
    << (unsigned int)m_nDOFqBody
    << (unsigned int)m_nDOFuBody
    << (unsigned int)(EnumConversion::toIntegral(m_additionalBytesPerBodyType))
    << (unsigned int)m_nAdditionalBytesPerBody;

    m_beginOfStates = m_file_stream.tellp();
}


bool  MultiBodySimFile::openRead(const boost::filesystem::path &file_path,
                                 bool readVelocities,
                                 unsigned int nDOFqBody,
                                 unsigned int nDOFuBody,
                                 unsigned int nSimBodies) {

    close();


    m_nDOFuBody = nDOFuBody;
    m_nDOFqBody = nDOFqBody;
    // Set if the read commands are reading the whole state! not only q! also u!
    m_readVelocities = readVelocities;
    m_nSimBodies = nSimBodies;

    m_errorString.str("");

    m_file_stream.open(file_path.string().c_str(), std::ios_base::binary | std::ios_base::in);
    //m_file_stream.rdbuf()->pubsetbuf(0, 0);
    m_file_stream.rdbuf()->pubsetbuf(m_Buffer, m_buf_size);
    m_file_stream.sync();
    if(m_file_stream.good()) {

        // Read length
        if(readHeader()) {

            setByteLengths();

            if(readLength()) {
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
    m_nDOFuBody=0;
    m_nDOFqBody=0;
    m_nBytes=0;
    m_nBytesPerState=0;
    m_nBytesPerQBody=0;
    m_nBytesPerUBody=0;
    m_additionalBytesPerBodyType=AdditionalBodyData::TypeEnum::NOTHING;
    m_nAdditionalBytesPerBody=0;
    m_nStates=0;
    m_nSimBodies=0;
    m_beginOfStates=0;
    m_readVelocities =  true;

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
    m_file_stream.seekg(0);


    if(m_nBytes > m_headerLength) {
        std::streamsize nStates = (m_nBytes - m_headerLength) / ( m_nBytesPerState );
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

    // set to beginning of file:
    m_file_stream.seekg(0);

    char signature[SIM_FILE_SIGNATURE_LENGTH];
    m_file_stream.read(signature,SIM_FILE_SIGNATURE_LENGTH);
    if(std::strncmp(signature,m_simFileSignature,SIM_FILE_SIGNATURE_LENGTH)==0) {

        unsigned int version;
        unsigned int  nBodies, nDofqBody, nDofuBody, addBytesPerBody;
        *this >> version;

        if(version != SIM_FILE_VERSION){
            m_errorString << " Binary file has version: "<< version << " which does not fit the compiled version: " << SIM_FILE_VERSION << std::endl;
            return false;
        }

        unsigned int addType;

        *this >> nBodies >> nDofqBody >> nDofuBody >> addType >> addBytesPerBody;
        m_additionalBytesPerBodyType = static_cast<AdditionalBodyData::TypeEnum>(addType);
        m_nAdditionalBytesPerBody = addBytesPerBody;

        if( m_nAdditionalBytesPerBody < 0){
            m_errorString << "Binary File, m_nAdditionalBytesPerBody not valid : " << m_nAdditionalBytesPerBody << std::endl;
            return false;
        }

        // if the body count is zero we dont need to check on the values, so overwrite the values
        if(m_nSimBodies == 0){
            m_nSimBodies = nBodies;
        }
        if(m_nDOFqBody == 0){
            m_nDOFqBody = nDofqBody;
        }
        if(m_nDOFuBody == 0){
            m_nDOFuBody = nDofuBody;
        }

        bool ok;
        if(m_readVelocities) {
            ok = nBodies == m_nSimBodies && nDofuBody == m_nDOFuBody && nDofqBody == m_nDOFqBody;
        } else {
            ok = nBodies == m_nSimBodies && nDofqBody == m_nDOFqBody;
        }

        if(ok) {
            m_beginOfStates = m_file_stream.tellg();
            return true;
        } else {
            m_errorString <<" Binary file does not correspond to the number of bodies which should be simulated: "<< std::endl
                          <<" Binary File describes: \t\tnSimBodies = "<<nBodies<< "\tnDofqBody = "<<nDofqBody<<"\tnDofuBody = " << nDofuBody << std::endl
                          <<" Simulation requests: \t\tnSimBodies = "<<m_nSimBodies<< "\tnDofqBody = "<<m_nDOFqBody<<"\tnDofuBody = " << m_nDOFuBody<<std::endl;
        }

    } else {
        m_errorString << "Binary file contains a wrong header and is not equal to: '" << m_simFileSignature<<"'"<<std::endl;
    }

    return false;
}


MultiBodySimFile & MultiBodySimFile::operator << (MultiBodySimFile& file){



    const std::size_t blockSize = 1024 * 1024; // 1mb buffer size;
    std::vector<char> data(blockSize);

    //Calculate bytes to write to this file;
    file.m_file_stream.seekg(0, std::ios::cur);
    std::streampos currentLoc = file.m_file_stream.tellg();
    std::streamsize states_rest = (file.m_nBytes - currentLoc) / file.m_nBytesPerState;


    if( states_rest > 0){

        std::streamsize bytesToCopy = states_rest * file.m_nBytesPerState;

        //Write loop in blockSize
        for(std::size_t block = 0; block < bytesToCopy/blockSize; block++)
        {
            file.m_file_stream.read(&data[0], blockSize);
            m_file_stream.write(&data[0], blockSize);
        }

        std::streamsize restBytes = bytesToCopy%blockSize;
        // Write rest
        if(restBytes != 0)
        {
            file.m_file_stream.read(&data[0], restBytes);
            m_file_stream.write(&data[0], restBytes);
        }
    }

    return *this;
}


