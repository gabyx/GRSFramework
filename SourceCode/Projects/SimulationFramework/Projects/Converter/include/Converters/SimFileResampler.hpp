#ifndef SimFileResampler_hpp
#define SimFileResampler_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <boost/filesystem.hpp>
#include "ProgressBarCL.hpp"
#include "MultiBodySimFile.hpp"


#include "LogDefines.hpp"

class SimFileResampler{

public:
     void resample(const std::vector<boost::filesystem::path> & inputFiles,
              boost::filesystem::path outputFile,
              std::streamoff stepSize = 1,
              std::streamoff startStateIdx = 0,
              std::streamoff endStateIdx = std::numeric_limits<unsigned int>::max()) {

        m_iFiles = inputFiles;
        m_oFile  = outputFile;

        m_startStateIdx =startStateIdx;
        m_endStateIdx =endStateIdx;
        m_stepSize = stepSize;

        resample();

    }

private:

    std::vector<boost::filesystem::path>  m_iFiles;
    boost::filesystem::path m_oFile;

    std::streamoff m_startStateIdx,  m_endStateIdx, m_stepSize;

    void resample(){
          std::cerr << "---> Execute Resample" << std::endl;
        // First open all files and check consistency!
        std::cerr << "---> Check consistency of input files..." << std::endl;
        unsigned int bodies, dofq, dofu;

        MultiBodySimFile fromFile;

        if(m_iFiles.size() != 1) {
            THROWEXCEPTION("Only one input file is accepted for resample!")
        }
        if(m_iFiles[0] == m_oFile){
            THROWEXCEPTION("Input/Output Files are the same!")
        }

        // Check Simfile!

        if(!fromFile.openRead(m_iFiles[0])) {
            THROWEXCEPTION(fromFile.getErrorString());
        };

        std::cerr << fromFile.getDetails() << std::endl;

        // Resample


        std::cerr << "---> Open new output file at: "  <<  m_oFile << std::endl;
        MultiBodySimFile toFile;
        if(!toFile.openWrite_impl(m_oFile,
                                  fromFile.getNDOFq(),
                                  fromFile.getNDOFu(),
                                  0,
                                  true)
                                  ){
            THROWEXCEPTION(toFile.getErrorString());
        };

        // Make Buffer
        std::streamsize size = fromFile.m_nBytesPerState; // state size
        std::vector<char> byteBuffer(size); // buffer


        // Resample loop
        std::cerr << "---> Resample from: [" << m_startStateIdx << "," <<  m_endStateIdx << "] "  <<  m_oFile << " ";
        ProgressBarCL<std::streamoff> pBar(std::cerr,"",fromFile.m_nStates-1);
        pBar.start();
        m_endStateIdx = std::min(m_endStateIdx , fromFile.m_nStates - 1);
        for( std::streamoff stateIdx = m_startStateIdx; stateIdx < fromFile.m_nStates && stateIdx <= m_endStateIdx; stateIdx+=m_stepSize) {
             fromFile.m_file_stream.read(&byteBuffer[0],size);
             toFile.m_file_stream.write(&byteBuffer[0],size);
             // Jump over all remaining states
             fromFile.m_file_stream.seekg( size * (m_stepSize-1),std::ios_base::cur);
             pBar.update(stateIdx);
        }
        std::cerr << "---> Resample done! " << std::endl;

        // Write header
        toFile.m_additionalBytesPerBodyType = fromFile.m_additionalBytesPerBodyType;
        toFile.m_nAdditionalBytesPerBody = fromFile.m_nAdditionalBytesPerBody;
        toFile.m_nSimBodies = fromFile.m_nSimBodies;
        toFile.writeHeader();

        std::cerr << "---> Close Files!" << std::endl;
        fromFile.close();
        toFile.close();
    }


};

#endif // SimFileResampler_hpp
