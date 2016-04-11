// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/dynamics/general/MultiBodySimFilePart.hpp"


const char MultiBodySimFilePart::m_simFileSignature[SIM_FILE_PART_SIGNATURE_LENGTH] = SIM_FILE_PART_SIGNATURE;

// Implementation

MultiBodySimFilePart::MultiBodySimFilePart(unsigned int bufferSize)
{
    m_filePath = boost::filesystem::path();

    m_buf_size = bufferSize;
    m_Buffer = new char[m_buf_size];

    m_errorString.str("");
}

MultiBodySimFilePart::~MultiBodySimFilePart() {
    close();
    delete[] m_Buffer;
}

/** given m_nDOFqBody, m_nDOFuBody, m_nAdditionalBytesPerBody*/
void MultiBodySimFilePart::setByteLengths() {
    m_nBytesPerQBody = m_nDOFqBody*sizeof(double);
    m_nBytesPerUBody = m_nDOFuBody*sizeof(double);

    m_nBytesPerBody = m_nBytesPerUBody
                    + m_nBytesPerQBody
                    + std::streamsize(sizeof(RigidBodyIdType)) + m_nAdditionalBytesPerBody;
}


bool MultiBodySimFilePart::openWrite(const boost::filesystem::path &file_path,
                                 unsigned int nDOFqBody,
                                 unsigned int nDOFuBody,
                                 bool truncate)
{
    if(nDOFqBody == 0 || nDOFuBody == 0){
        m_errorString << "Wrong openWrite parameters: nDOFqBody:" << nDOFqBody << " nDOFuBody: "<<nDOFuBody <<  std::endl;
        return false;
    }

    m_nDOFuBody = nDOFuBody;
    m_nDOFqBody = nDOFqBody;
    m_nAdditionalBytesPerBody = 0;
    m_additionalBytesPerBodyType = 0;

    setByteLengths();

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

   *this << (unsigned int)SIM_FILE_PART_VERSION
    << (unsigned int)m_nStates // is written at the end!
    << (unsigned int)m_nDOFqBody
    << (unsigned int)m_nDOFuBody
    << (unsigned int)m_additionalBytesPerBodyType
    << (unsigned int)m_nAdditionalBytesPerBody;

    m_beginOfStates = m_file_stream.tellp();
}

/** Only for writting*/
std::streamoff MultiBodySimFilePart::getAdditionalBytesPerBody()
{
        switch(m_additionalBytesPerBodyType){
            case 0:
                return 0;
            default:
                GRSF_ERRORMSG("Additional Byte Type not implemented");
        }
        return 0;
}


void MultiBodySimFilePart::close() {

    //Write the m_nStates variable into the header
    m_file_stream.seekp(m_beginHeader);
    *this << (unsigned int)m_nStates;

    // Reset all values;
    m_nBytes = 0;
    m_nStates = 0;
    m_filePath = boost::filesystem::path();

    if(m_file_stream.is_open()) {
        m_file_stream.close();
        m_file_stream.clear();
    }

}

