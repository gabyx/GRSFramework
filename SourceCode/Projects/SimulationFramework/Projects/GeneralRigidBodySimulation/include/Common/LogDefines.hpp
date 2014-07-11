﻿/*
 *  LogDefines.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef LogDefines_hpp
#define LogDefines_hpp

/**
* @ingroup Common
* @defgroup LogDefines  Log Defines for the Framework
* @brief These defines specify the over all loggers of the framework. The can be enabled and disabled.
*/
/* @{ */


/** @name Global Log Macros
* @brief
*/
/* @{ */
#define LOG( logptr , message )  ( * (logptr) ) << message ;  ///< Macro to easily write into a SimpleLogger::Log.
#define LOGLEVEL( level , setlevel , logptr , message) if( level <= setlevel ){  LOG(logptr,message); }


#ifndef NDEBUG
    // DEBUG!
    /// SceneParser
    #define SCENEPARSER_LOGLEVEL 3  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
    #define LOGSC( log , message ) LOG(log,message);
    #define LOGSCLEVEL( level, logptr , message ) LOGLEVEL( level, SCENEPARSER_LOGLEVEL , logptr , message);
    #define LOGSCLEVEL1( logptr , message) LOGSCLEVEL( 1 , logptr , message) ;
    #define LOGSCLEVEL2( logptr , message) LOGSCLEVEL( 2 , logptr , message) ;
    #define LOGSCLEVEL3( logptr , message) LOGSCLEVEL( 3 , logptr , message) ;
    #define SKIPLOGSC( logptr , message )  LOGSCLEVEL( 1 , logptr , message) ;

#else
    /// SceneParser
    #define SCENEPARSER_LOGLEVEL 1  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
    #define LOGSC( log , message ) LOG(log,message);
    #define LOGSCLEVEL( level, logptr , message ) LOGLEVEL( level, SCENEPARSER_LOGLEVEL , logptr , message);
    #define LOGSCLEVEL1( logptr , message) LOGSCLEVEL( 1 , logptr , message) ;
    #define LOGSCLEVEL2( logptr , message) LOGSCLEVEL( 2 , logptr , message) ;
    #define LOGSCLEVEL3( logptr , message) LOGSCLEVEL( 3 , logptr , message) ;
    #define SKIPLOGSC( logptr , message )  LOGSCLEVEL( 1 , logptr , message) ;

#endif


/* @} */


/** @name File and Folders
* @brief
*/
/* @{ */

#define PROCESS_FOLDER_PREFIX "ProcessGUI_"

#define GLOBAL_LOG_FOLDER_DIRECTORY "./Logs"

#define VIDEO_FOLDER_PATH "./Videos"                       ///< Directory where new simulation folders are place during record.
#define VIDEO_FOLDER_PREFIX "Video_"                       ///< Directory where the video frames are place during record.
#define SIM_VIDEO_PREFIX "VideoFrame_"                     ///< File prefix for the video frames;

#define SIMULATION_FOLDER_PATH "./SimFiles"                      ///< Directory where new simulation folders are place during record.
#define SIM_FOLDER_PREFIX_RESAMPLE "SimDataRESAMPLEGUI_"            ///< Directory where the video frames are place during record.
#define SIM_SCENE_FILE_NAME "SceneFile"                          ///< File name for the Scene xml file which describes the scene
#define SIM_FOLDER_PREFIX_REALTIME "SimDataREALTIMEGUI_"            ///< Prefix for the simulation folder during realtime simulation.
#define SIM_FOLDER_PREFIX_RECORD "SimDataRECORDGUI_"                ///< Prefix for the simulation folder during record.
#define SIM_FOLDER_PREFIX_INIT "SimDataInitialState_"             ///< Prefix for the folder where the inital states are written, press Key I!
#define SIM_INIT_FILE_PREFIX "InitialState"                      ///< The name for the Initial state file!
#define SIM_FILE_PREFIX "SimState"                              ///< Prefix for the .sim file.
#define SOLVER_LOG_FILE_PREFIX "SolverLog"                       ///< Prefix for the solver log file.
#define COLLISION_DATA_FILE_PREFIX "CollisionData"               ///< Prefix for the collision data file.
#define SYSTEM_DATA_FILE_PREFIX "SimData"                     ///< Prefix for the system data file.
/* @} */


/** @name SimulationLog
* @brief All these defines are used in the solver thread. The output goes into the solver log with filename #SOLVER_LOG_FILE_PREFIX
*/

#define SIMULATION_LOG_TO_CONSOLE true



/** @name Solver Threads
* @brief All these defines are used in the solver thread. The output goes into the solver log with filename #SOLVER_LOG_FILE_PREFIX
*/
/* @{ */
#ifndef NDEBUG
  // DEBUG!
  #define CoutLevelSolver 3            ///<   0 for Off,  1 for Basics, 2 for Advanced, 3 for Full Output
  #define CoutLevelSolverWhenContact 3 ///<   0 for Off,  1 for Basics, 2 for Advanced, 3 for Full Output
  #define LogToFileSolver 1            ///< {0,1} Determines if logstream is saved into a file.
  #define LogToConsoleSolver 0         ///< {0,1} Determines if logstream is outputted into console.
#else
  #define CoutLevelSolver 2
  #define CoutLevelSolverWhenContact 1
  #define LogToFileSolver 1
  #define LogToConsoleSolver 0
#endif
/* @} */

/** @name System Data file for Record Mode only.
* @brief
*/
/* @{ */
#define OUTPUT_SIMDATA_FILE 1  ///< {0,1} Sets if the System Data file is outputted
#define CALCULATE_COND_OF_G 0     ///< {0,1} Set if the condition of the G matrix is calculated and outputted. Takes alot of time!
#define CALCULATE_DIAGDOM_OF_G 1  ///< {0,1} Set if the diagonal dominant criteria is calculated, the number shows how many rows are not diagonal dominant!
#define MEASURE_TIME_PROX 1
/* @} */


/** @name System Data file for Record Mode only.
* @brief
*/
/* @{ */
#define OUTPUT_COLLISIONDATA_FILE 0 ///< {0,1} Sets if the Collision Data file is outputted
/* @} */


/** @name Playback Manager
* @brief  Log file of the Playback Manager.
*/
/* @{ */
#define LogToFilePlayback 1      ///< {0,1} Set if log is outputted to file or not.
#define LogToConsolePlayback 1   ///< {0,1} Set if log is outputted to console or not.
/* @} */

/** @name Loader Thread
* @brief Log file of the Loader Thread which is started during playback.
*/
/* @{ */
#define LogToFileLoader 1        ///< {0,1} Set if log is outputted to file or not.
#define LogToConsoleLoader 1     ///< {0,1} Set if log is outputted to console or not.
/* @} */


/** @name App Log File
* @brief Log File for the Application.
*/
/* @{ */
#define LogToFileApp 1     ///< {0,1} Set if log is outputted to file or not.
#define LogToConsoleApp 1  ///< {0,1} Set if log is outputted to console or not.
/* @} */


/** @name Ogre Log File
* @brief Ogre File for the Application.
*/
/* @{ */
#define LogToFileOgre 1     ///< {0,1} Set if log is outputted to file or not.
#define LogToConsoleOgre 1  ///< {0,1} Set if log is outputted to console or not.
/* @} */



/** @name  Deconstructor and Constructor Macros
* @brief Deconstructor and Constructor Macros to Debug correct dealloction of objects.
*/
/* @{ */
#ifndef NDEBUG
#define DECONSTRUCTOR_MESSAGE \
 std::cout << "Destructor: "<< typeid(*this).name()  <<" , @ : "<< this << std::endl;
#define CONSTRUCTOR_MESSAGE \
  std::cout << "Constructor: "<< typeid(*this).name()  <<" , @ : "<< this << std::endl;;
#else
  #define DECONSTRUCTOR_MESSAGE
  #define CONSTRUCTOR_MESSAGE
#endif
/* @} */



/** @name State Ring Pool */
/* @{ */
#define LogToFileStateRingPool 0 ///< {0,1} Set if log is outputted to file or not.
/* @} */

/** @name State Pool */
/* @{ */
#define LogToFileStatePool 0     ///< {0,1} Set if log is outputted to file or not.
/* @} */


/* @} */
#endif
