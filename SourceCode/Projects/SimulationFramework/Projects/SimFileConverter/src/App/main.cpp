#include <iostream>
#include <string>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "Exception.hpp"

#include "AssertionDebug.hpp"

#include "ApplicationCLOptionsConverter.hpp"



#include <boost/filesystem.hpp>
#include "MultiBodySimFile.hpp"
class SimFileJoiner{
public:

    SimFileJoiner(const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path outputFile): m_iFiles(inputFiles), m_oFile(outputFile){}


    void join(){

        // First open all files and check consistency!
        std::cerr << "---> Check consistency of input files..." << std::endl;
        unsigned int bodies, dofq, dofu;

        MultiBodySimFile simFile;
        std::streamsize bState;
        unsigned int i = 0;

        if(m_iFiles.size() <=1){
            THROWEXCEPTION("To little input files specified!")
        }

        for(auto it=m_iFiles.begin(); it!=m_iFiles.end();it++){
            if(!simFile.openRead(*it)){
                THROWEXCEPTION(simFile.getErrorString());
            };
            if(i == 0){
                dofq = simFile.getNDOFq();
                dofu = simFile.getNDOFu();
                bodies = simFile.getNSimBodies();
                bState = simFile.getBytesPerState();
            }
            if(i != 0 && (simFile.getNSimBodies() != bodies || simFile.getBytesPerState() != bState )){
                THROWEXCEPTION("Number of bodies: " << simFile.getNSimBodies() << " , bytesPerState: "
                               << simFile.getBytesPerState() << " of file: "
                               << *it << " do not match bodies: " << bodies << " , bytesPerState: "
                               << bState <<" of first file!");
            }

            simFile.close();
            i++;
        }

        // Join all states together
        std::cerr << "---> Open new output file at: "  <<  m_oFile << " bodies: " << bodies<< std::endl;
        MultiBodySimFile output;
        output.openWrite(m_oFile,dofq,dofu,bodies);

        // Push all simfiles to output
        for(auto it=m_iFiles.begin(); it!=m_iFiles.end();it++){
            if(!simFile.openRead(*it)){
                THROWEXCEPTION(simFile.getErrorString());
            };
            output << simFile;
        }

        output.close();

    }

private:
    std::vector<boost::filesystem::path>  m_iFiles;
    boost::filesystem::path m_oFile;
};


int main(int argc, char **argv) {
    // Parsing Input Parameters===================================
    ApplicationCLOptions opts;
    ApplicationCLOptions::getSingletonPtr()->parseOptions(argc,argv);

    ApplicationCLOptions::getSingletonPtr()->checkArguments();
    ApplicationCLOptions::getSingletonPtr()->printArgs(std::cout);
    // End Parsing =================================


    if( ApplicationCLOptions::getSingletonPtr()->m_task == ApplicationCLOptions::Task::JOIN){

       try{
            SimFileJoiner joiner(opts.m_inputFiles,opts.m_outputFile);
            joiner.join();
       }catch(const Exception & e){
            std::cerr <<"Exception occured: " <<  e.what() << std::endl;
            exit(EXIT_FAILURE);
       }
    }else{
        std::cerr <<"Undefined task specified!";
        exit(EXIT_FAILURE);
    }


    return 0;
}
