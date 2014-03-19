#include <iostream>
#include <string>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "Exception.hpp"

#include "AssertionDebug.hpp"

#include "ApplicationCLOptionsConverter.hpp"

#include "SimFileJoiner.hpp"




int main(int argc, char **argv) {
    // Parsing Input Parameters===================================
    ApplicationCLOptions opts;
    ApplicationCLOptions::getSingletonPtr()->parseOptions(argc,argv);

    ApplicationCLOptions::getSingletonPtr()->checkArguments();
    ApplicationCLOptions::getSingletonPtr()->printArgs(std::cout);
    // End Parsing =================================


    if( ApplicationCLOptions::getSingletonPtr()->m_task == ApplicationCLOptions::Task::JOIN){

       try{
            SimFileJoiner joiner;
            joiner.join(opts.m_inputFiles,opts.m_outputFile, opts.m_timeRange, opts.m_bodyRange);
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
