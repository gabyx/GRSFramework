#include <iostream>
#include <string>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "Exception.hpp"

#include "AssertionDebug.hpp"

#include "ApplicationCLOptionsConverter.hpp"

#include "SimFileJoiner.hpp"
#include "SimFileResampler.hpp"
#include "RenderScriptConverter.hpp"


int main(int argc, char **argv) {


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
                    resampler.resample(opts.m_inputFiles,opts.m_outputFile, opts.m_stepSize);
               }catch(const Exception & e){
                    std::cerr <<"Exception occured: " <<  e.what() << std::endl;
                    exit(EXIT_FAILURE);
               }

            }else{
                std::cerr <<"Undefined task specified!";
                exit(EXIT_FAILURE);
            }
        }
        else if(std::string(argv[1]) == "renderer"){

            Logging::LogManager logger; // singelton

            // Parsing Input Parameters===================================
            ApplicationCLOptionsRenderer opts;  // singelton
            opts.parseOptions(argc-1,++argv);
            opts.checkArguments();
            opts.printArgs(std::cout);
            // End Parsing =================================

            RenderScriptConverter renderConv;
            renderConv.convert(opts.m_inputFiles,opts.m_outputFile,opts.m_sceneFile, opts.m_materialFile ,opts.m_renderer);

        }else{
            std::cerr << "Wrong Options: " << argv[1] << std::endl
            << " Help: \n"
            << "    converter sim      [-h|--help]        : Sim File Converter\n"
            << "    converter renderer [-h|--help]        : Sim File / Scene to Renderfile Converter" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    return 0;
}
