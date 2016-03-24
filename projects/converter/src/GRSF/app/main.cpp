// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

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
