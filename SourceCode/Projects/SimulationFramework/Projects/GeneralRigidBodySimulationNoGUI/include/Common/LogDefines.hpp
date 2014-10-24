/*
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
#define LOGLEVEL(level,setlevel,logptr,message) if( level <= setlevel ){  LOG(logptr,message); }

#ifndef NDEBUG
    // DEBUG!
    /// SceneParser
    #define SCENEPARSER_LOGLEVEL 2  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
#else
    /// SceneParser
    #define SCENEPARSER_LOGLEVEL 2  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
#endif

/** SceneParser Log Macros*/
#define LOGSC( log , message ) LOG(log,message);
#define LOGSCLEVEL( level, logptr , message ) LOGLEVEL( level, SCENEPARSER_LOGLEVEL , logptr , message);
#define LOGSCLEVEL1( logptr , message) LOGSCLEVEL( 1 , logptr , message) ;
#define LOGSCLEVEL2( logptr , message) LOGSCLEVEL( 2 , logptr , message) ;
#define LOGSCLEVEL3( logptr , message) LOGSCLEVEL( 3 , logptr , message) ;
#define SKIPLOGSC( logptr , message )  LOGSCLEVEL( 1 , logptr , message) ;

/* @} */


/** @name File and Folders
* @brief
*/
/* @{ */

#define PROCESS_FOLDER_PREFIX "ProcessNoGUI"

#define GLOBAL_LOG_FOLDER_DIRECTORY "./Logs"

#define SIMULATION_FOLDER_PATH "./SimFiles"                       ///< Directory where new simulation folders are place during record.
#define SIM_FILE_EXTENSION ".sim"                                       ///< File extension for .sim file.
#define SIM_SCENE_FILE_NAME "SceneFile"                                 ///< File name for the Scene xml file which describes the scene
#define SIM_FOLDER_PREFIX_RECORD "SimDataRECORD_"                ///< Prefix for the simulation folder during record.
#define SIM_FOLDER_PREFIX_INIT "SimDataInitialState_"             ///< Prefix for the folder where the inital states are written, press Key I!
#define SIM_INIT_FILE_EXTENSION ".sim"                              ///< File extension for .siminit file.
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
    #define SOLVERLOG_LOGLEVEL 1  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
    #define SOLVERLOG_LOGLEVEL_CONTACT 0  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
    #define SOLVERLOG_TOFILE 1            ///< {0,1} Determines if logstream is saved into a file.
    #define SOLVERLOG_TOCONSOLE 1         ///< {0,1} Determines if logstream is outputted into console.
#else
    #define SOLVERLOG_LOGLEVEL 1  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
    #define SOLVERLOG_LOGLEVEL_CONTACT 0  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
    #define SOLVERLOG_TOFILE 1
    #define SOLVERLOG_TOCONSOLE 0
#endif

/** Inclusion Solver Log Macros */
#define LOGSL( log , message ) LOG(log,message);
#define LOGSLLEVEL( level, logptr , message ) LOGLEVEL( level, SOLVERLOG_LOGLEVEL , logptr , message);
#define LOGSLLEVEL1( logptr , message) LOGSLLEVEL( 1 , logptr , message) ;
#define LOGSLLEVEL2( logptr , message) LOGSLLEVEL( 2 , logptr , message) ;
#define LOGSLLEVEL3( logptr , message) LOGSLLEVEL( 3 , logptr , message) ;

#define LOGSLLEVEL_CONTACT( level, logptr , message ) LOGLEVEL( level, SOLVERLOG_LOGLEVEL_CONTACT , logptr , message);
#define LOGSLLEVEL1_CONTACT( logptr , message) LOGSLLEVEL( 1 , logptr , message) ;
#define LOGSLLEVEL2_CONTACT( logptr , message) LOGSLLEVEL( 2 , logptr , message) ;
#define LOGSLLEVEL3_CONTACT( logptr , message) LOGSLLEVEL( 3 , logptr , message) ;

/* @} */

/** @name System Data file for Record Mode only.
* @brief
*/
/* @{ */
#define OUTPUT_SIMDATA_FILE 1  ///< {0,1} Sets if the System Data file is outputted
#define OUTPUT_SIMDATAITERATION_FILE 1 ///< {0,1} Sets if the SimDataIteration file is outputted
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



/** @name  Deconstructor and Constructor Macros
* @brief Deconstructor and Constructor Macros to Debug correct dealloction of objects.
*/
/* @{ */
#ifdef _DEBUG
#define DESTRUCTOR_MESSAGE \
 LOG(m_pSimulationLog, "Destructor: "<< typeid(*this).name()  <<" , @ : "<< this;);
#define CONSTRUCTOR_MESSAGE \
  LOG(m_pSimulationLog, "Constructor: "<< typeid(*this).name()  <<" , @ : "<< this;);
#else
  #define DECONSTRUCTOR_MESSAGE
  #define CONSTRUCTOR_MESSAGE
#endif
/* @} */

/* @} */
#endif
