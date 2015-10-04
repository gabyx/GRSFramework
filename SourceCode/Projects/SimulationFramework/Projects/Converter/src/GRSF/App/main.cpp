#include <iostream>
#include <string>

#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Common/Exception.hpp"

#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/Common/ApplicationCLOptionsConverter.hpp"
#include "GRSF/Common/ApplicationSignalHandler.hpp"

#include "GRSF/Converters/SimFileInfo.hpp"
#include "GRSF/Converters/SimFileJoiner.hpp"
#include "GRSF/Converters/SimFileResampler.hpp"
//#include "GRSF/Converters/RenderConverter.hpp"
#include "GRSF/Converters/AnalyzerConverter.hpp"
#include "GRSF/Converters/GridderConverter.hpp"

void printHelpAndExit(std::string o=""){
     std::cerr << "Wrong Options: '" << o <<"'"<< std::endl
            << " Help: \n"
            << "    converter sim      [-h|--help]        : Sim File Converter\n"
            << "    converter siminfo  [-h|--help]        : Sim File Info \n"
            << "    converter renderer [-h|--help]        : Sim File / Scene to Renderfile Converter" << std::endl;
            exit(EXIT_FAILURE);
}

void callBackSIGINT(int){
    std::cerr << "Converter:: exit ..." << std::endl;
    exit(EXIT_FAILURE);
}

void callBackSIGPIPE(int){
    std::cerr << "Converter:: Pipe error: exit ..." << std::endl;
    exit(EXIT_FAILURE);
}

int main(int argc, char **argv) {

    ApplicationSignalHandler sigHandler( {SIGINT,SIGTERM,SIGUSR1,SIGUSR2,SIGPIPE} );
    sigHandler.registerCallback(SIGINT,callBackSIGINT,"callBackSIGINT");
    sigHandler.registerCallback(SIGPIPE,callBackSIGPIPE,"callBackSIGPIPE");


    #ifndef NDEBUG
            std::cerr << "GRSFramework Converter: build: ?, config: " << "debug" << std::endl;
    #else
            std::cerr << "GRSFramework Converter: build: ?, config: " << "release" << std::endl;
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

//            Logging::LogManager logger; // singelton
//
//            // Parsing Input Parameters===================================
//            ApplicationCLOptionsRenderer opts;  // singelton
//            opts.parseOptions(argc-1,++argv);
//            opts.checkArguments();
//            opts.printArgs(std::cerr);
//            // End Parsing =================================
//
//            try{
//                RenderConverter renderConv(opts.getInputFiles(),
//                                   opts.getOutputFile(),
//                                   opts.getOutputDir(),
//                                   opts.getSceneFile(),
//                                   opts.getConverterLogicFile() ,
//                                   opts.getRenderer());
//                renderConv.convert();
//
//            }catch(const Exception & e){
//                    std::cerr <<"Exception occured: " <<  e.what() << std::endl;
//                    exit(EXIT_FAILURE);
//            }
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
                                     opts.getOutputFile(),
                                     opts.getOutputDir(),
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
                                     opts.getOutputFile(),
                                     opts.getOutputDir(),
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
