// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <iostream>
#include <string>

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/common/Exception.hpp"

#include "GRSF/common/AssertionDebug.hpp"

#include "GRSF/common/ApplicationCLOptionsConverter.hpp"
#include "GRSF/common/ApplicationSignalHandler.hpp"

#include "GRSF/converters/simInfo/SimFileInfo.hpp"
#include "GRSF/converters/simJoiner/SimFileJoiner.hpp"
#include "GRSF/converters/simResampler/SimFileResampler.hpp"
#include "GRSF/converters/renderer/RenderConverter.hpp"
#include "GRSF/converters/analyzer/AnalyzerConverter.hpp"
#include "GRSF/converters/gridder/GridderConverter.hpp"

void printHelpAndExit(std::string o=""){
     std::cerr << "Wrong Options: '" << o <<"'"<< std::endl
            << " Help: \n"
            << "    converter sim      [-h|--help]        : convert '.sim' files \n"
            << "    converter siminfo  [-h|--help]        : info about '.sim' files \n"
            << "    converter renderer [-h|--help]        : produce render output of '.sim' files and scene XML by execution graph XML\n"
            << "    converter analyzer [-h|--help]        : analyze '.sim' files with execution graph XML\n"
            << "    converter gridder  [-h|--help]        : extract gridded data from '.sim' files" << std::endl;
            exit(EXIT_FAILURE);
}

void callBackSignalAndExit(int signum){
    std::cerr << "GRSFramework Converter: received signal: " << signum << " -> exit ..." << std::endl;
    // http://www.cons.org/cracauer/sigint.html
    // set sigint handler to nothing
    // and kill ourself
    signal(SIGINT, SIG_DFL);
    kill(getpid(),SIGINT);
}

void callBackSIGPIPE(int){
    std::cerr << "GRSFramework Converter: received SIGPIPE -> Pipe error: exit ..." << std::endl;
    signal(SIGINT, SIG_DFL);
    kill(getpid(),SIGINT);
}

int main(int argc, char **argv) {

    INSTANCIATE_UNIQUE_SINGELTON_CTOR(ApplicationSignalHandler,sigHandler, ( {SIGINT,SIGUSR2,SIGPIPE} ) )
    sigHandler->registerCallback({SIGINT,SIGUSR2},callBackSignalAndExit,"callBackSignalAndExit");
    sigHandler->registerCallback(SIGPIPE,callBackSIGPIPE,"callBackSIGPIPE");


    std::cout << "GRSFramework Converter: version: " GRSF_VERSION_STRING
    #ifndef NDEBUG
            " | config: " << "debug" << std::endl;
    #else
            " | config: " << "release" << std::endl;
    #endif

    if(argc > 1){
        if(std::string(argv[1]) == "sim"){
            // Parsing Input Parameters===================================
            ApplicationCLOptionsSimConverter opts;
            opts.parseOptions(argc-1,++argv);

            opts.checkArguments();
            opts.printArgs(std::cout);
            // End Parsing =================================


            if( opts.m_task == ApplicationCLOptionsSimConverter::Task::JOIN){

               try{
                    SimFileJoiner joiner;
                    joiner.join(opts.m_inputFiles,opts.m_outputFile, opts.m_timeRange, opts.m_bodyRange);
               }catch(const Exception & e){
                    std::cerr <<"Exception occured: " <<  e.what() << std::endl;
                    exit(EXIT_FAILURE);
               }
            }else if( opts.m_task == ApplicationCLOptionsSimConverter::Task::RESAMPLE){
               try{
                    SimFileResampler resampler;
                    resampler.resample(opts.m_inputFiles,opts.m_outputFile,
                                       opts.m_increment, opts.m_startStateIdx, opts.m_endStateIdx,
                                       opts.m_splitIntoFiles);
               }catch(const Exception & e){
                    std::cerr <<"Exception occured: " <<  e.what() << std::endl;
                    exit(EXIT_FAILURE);
               }

            }else{
                std::cerr <<"Undefined task specified!";
                exit(EXIT_FAILURE);
            }
        }
        else if(std::string(argv[1]) == "siminfo"){
            // Parsing Input Parameters===================================
            ApplicationCLOptionsSimInfo opts;
            opts.parseOptions(argc-1,++argv);

            opts.checkArguments();
            opts.printArgs(std::cerr);
            // End Parsing =================================

           try{
                SimFileInfo info;

                std::cout << info.getInfoString(opts.m_inputFiles,
                                                opts.m_increment,
                                                opts.m_startStateIdx,
                                                opts.m_endStateIdx,
                                                opts.m_skipFirstState,
                                                opts.m_prettyPrint,
                                                !opts.m_noTimeList );

           }catch(const Exception & e){
                std::cerr <<"Exception occured: " <<  e.what() << std::endl;
                exit(EXIT_FAILURE);
           }

        }
        else if(std::string(argv[1]) == "renderer"){

            Logging::LogManager logger; // singelton

            // Parsing Input Parameters===================================
            ApplicationCLOptionsRenderer opts;  // singelton
            opts.parseOptions(argc-1,++argv);
            opts.checkArguments();
            opts.printArgs(std::cerr);
            // End Parsing =================================

            try{
                RenderConverter renderConv(opts.getInputFiles(),
                                   opts.getSceneFile(),
                                   opts.getConverterLogicFile() ,
                                   opts.getRenderer());
                renderConv.convert();

            }catch(const Exception & e){
                    std::cerr <<"Exception occured: " <<  e.what() << std::endl;
                    exit(EXIT_FAILURE);
            }
        }else if(std::string(argv[1]) == "analyzer"){

            Logging::LogManager logger; // singelton

            // Parsing Input Parameters===================================
            ApplicationCLOptionsAnalyzer opts;  // singelton
            opts.parseOptions(argc-1,++argv);
            opts.checkArguments();
            opts.printArgs(std::cerr);
            // End Parsing =================================

            try{
                AnalyzerConverter analyzerConv(opts.getInputFiles(),
                                     opts.getSceneFile(),
                                     opts.getConverterLogicFile());
                analyzerConv.convert();

            }catch(const Exception & e){
                    std::cerr <<"Exception occured: " <<  e.what() << std::endl;
                    exit(EXIT_FAILURE);
            }
        }else if(std::string(argv[1]) == "gridder"){

            Logging::LogManager logger; // singelton

            // Parsing Input Parameters===================================
            ApplicationCLOptionsGridder opts;  // singelton
            opts.parseOptions(argc-1,++argv);
            opts.checkArguments();
            opts.printArgs(std::cerr);
            // End Parsing =================================

            try{
                GridderConverter gridderConv(opts.getInputFiles(),
                                     opts.getSceneFile(),
                                     opts.getConverterLogicFile());
                gridderConv.convert();

            }catch(const Exception & e){
                    std::cerr <<"Exception occured: " <<  e.what() << std::endl;
                    exit(EXIT_FAILURE);
            }
        }else{
           printHelpAndExit(std::string(argv[1]));
        }
    }else{
        printHelpAndExit();
    }
    return 0;
}
