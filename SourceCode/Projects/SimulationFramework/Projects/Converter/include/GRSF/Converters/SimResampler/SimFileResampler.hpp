#ifndef GRSF_Converters_SimFileResampler_hpp
#define GRSF_Converters_SimFileResampler_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <boost/filesystem.hpp>
#include "GRSF/Common/ProgressBarCL.hpp"
#include "GRSF/Dynamics/General/MultiBodySimFile.hpp"


#include "GRSF/Common/LogDefines.hpp"

class SimFileResampler{

public:
     void resample(const std::vector<boost::filesystem::path> & inputFiles,
              boost::filesystem::path outputFile,
              std::streamsize stepSize = 1,
              std::streamsize startStateIdx = 0,
              std::streamsize endStateIdx = std::numeric_limits<std::streamsize>::max(),
              bool splitStatesIntoFiles = false) {

        m_iFiles = inputFiles;
        m_oFile  = outputFile;

        m_startStateIdx =startStateIdx;
        m_endStateIdx =endStateIdx;
        m_stepSize = stepSize;
        m_splitStatesIntoFiles = splitStatesIntoFiles;

        resample();


    }

private:

    std::vector<boost::filesystem::path>  m_iFiles;
    boost::filesystem::path m_oFile;

    std::streamsize m_startStateIdx,  m_endStateIdx, m_stepSize;
    bool m_splitStatesIntoFiles;

    void resample(){
          std::cerr << "---> Execute Resample" << std::endl;
        // First open all files and check consistency!
        std::cerr << "---> Check consistency of input files..." << std::endl;

        MultiBodySimFile fromFile;

        if(m_iFiles.size() != 1) {
            ERRORMSG("Only one input file is accepted for resample!")
        }
        if(m_iFiles[0] == m_oFile){
            ERRORMSG("Input/Output Files are the same!")
        }

        // Check Simfile!

        if(!fromFile.openRead(m_iFiles[0])) {
            ERRORMSG(fromFile.getErrorString());
        };

        std::cerr << fromFile.getDetails().getString() << std::endl;

        // Resample
        std::unique_ptr<MultiBodySimFile> toFile;

        if(!m_splitStatesIntoFiles){
            std::cerr << "---> Open new output file at: "  <<  m_oFile << std::endl;
            toFile = openFile(fromFile,m_oFile); // unique ptr
        }else{
            std::cerr << "---> Splitting into files at: "  <<  m_oFile << std::endl;
        }

        // Make Buffer
        std::streamsize size = fromFile.m_nBytesPerState; // state size
        std::vector<char> byteBuffer(size); // buffer


        // Resample loop
        std::cerr << "---> Resample from: [" << m_startStateIdx << "," <<  m_endStateIdx << "] "  <<  m_oFile << " ";

        m_endStateIdx = std::min(m_endStateIdx , fromFile.m_nStates - 1);

        ProgressBarCL<std::streamoff> pBar(std::cerr,"", m_startStateIdx, m_endStateIdx);
        pBar.start();

        // Put read pointer to beginning of state
        fromFile.m_file_stream.seekg( m_startStateIdx * size,std::ios_base::cur);
        for( std::streamoff stateIdx = m_startStateIdx;
        stateIdx <= m_endStateIdx; stateIdx+=m_stepSize) {

             if(m_splitStatesIntoFiles){
                auto f = m_oFile.parent_path() / m_oFile.filename().replace_extension("");
                f += std::to_string(stateIdx);
                f.replace_extension(m_oFile.extension());
                toFile = openFile(fromFile,f); // unique ptr
             }

             fromFile.m_file_stream.read(&byteBuffer[0],size);
             //std::cout << "t: " <<  *reinterpret_cast<double*>(&byteBuffer[0]) << std::endl;
             toFile->m_file_stream.write(&byteBuffer[0],size);
             // Jump over all remaining states
             fromFile.m_file_stream.seekg( size * (m_stepSize-1),std::ios_base::cur);


             pBar.update(stateIdx);

             if(m_splitStatesIntoFiles){
                  toFile->close();
             }

        }
        pBar.update(m_endStateIdx);
        std::cerr << "---> Resample done! " << std::endl;
        std::cerr << "---> Close Files!" << std::endl;
        fromFile.close();
        if(!m_splitStatesIntoFiles){
            toFile->close();
        }
    }

    std::unique_ptr<MultiBodySimFile> openFile(MultiBodySimFile & fromFile, boost::filesystem::path f){
        std::unique_ptr<MultiBodySimFile> toFile = std::unique_ptr<MultiBodySimFile>(new MultiBodySimFile()) ;
        if(!toFile->openWrite_impl(   f,
                                  fromFile.getNDOFq(),
                                  fromFile.getNDOFu(),
                                  fromFile.m_nSimBodies,
                                  true,
                                  fromFile.m_additionalBytesPerBodyType,
                                  fromFile.m_nAdditionalBytesPerBody
                                  )){
            ERRORMSG(toFile->getErrorString());
        };
        return std::move(toFile);
    }


};

#endif // SimFileResampler_hpp
