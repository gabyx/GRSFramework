// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_LogDefines_hpp
#define GRSF_common_LogDefines_hpp

/**
* @ingroup Common
* @defgroup LogDefines  Log Defines for the Framework
* @brief Defines for logging and output functionality.
*/
/* @{ */

/** @name Global Log Macros
* @brief
*/
/* @{ */
#define LOG(logptr, message) (*(logptr)) << message;  ///< Macro to easily write into a SimpleLogger::Log.
#define LOGLEVEL(level, setlevel, logptr, message) \
    if (level <= setlevel)                         \
    {                                              \
        LOG(logptr, message);                      \
    }

/// Simulation Log
#define SIMULATION_LOG_TO_CONSOLE false

#ifndef NDEBUG
// DEBUG!

/// Body Communicator
#define LOGBC(logptr, message) \
    LOG(logptr,                \
        message);  ///< Macro to easily write into a SimpleLogger::Log (only for the neighbour communicator part).
/// Serializer
#define LOGBC_SZ(logptr, message) \
    LOG(logptr, message);  ///< Macro to easily write into a SimpleLogger::Log (only for the serialization part).

/// Inclusion Communicator
#define LOGIC(logptr, message) \
    LOG(logptr,                \
        message);  ///< Macro to easily write into a SimpleLogger::Log (only for the inclusion communicator part).
                   /// Seriliazer
#define LOGIC_SZ(logptr, message) \
    LOG(logptr, message);  ///< Macro to easily write into a SimpleLogger::Log (only for the serialization part).

/// Process Communicator
#define LOGPC(logptr, message)  // LOG(logptr,message) ; ///< Macro to easily write into a SimpleLogger::Log (only for
                                // the process communicator part).

/// Topology Builder
#define TOPOBUILDER_LOGLEVEL 3  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
#define TOPOLOGY_BUILDER_WRITE_TOPO
#define TOPOLOGY_BUILDER_WRITE_PREDICTED_POINTS
#define TOPOLOGY_BUILDER_WRITE_NEAREST_DISTANCES

/// SceneParser
#define SCENEPARSER_LOGLEVEL 2  /// 0 - No output, 1 basic output, 2 medium output, 3 full output

/// Solver
#define SOLVERLOG_LOGLEVEL 3          /// 0 - No output, 1 basic output, 2 medium output, 3 full output
#define SOLVERLOG_LOGLEVEL_CONTACT 1  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
#define SOLVERLOG_TOFILE 1            ///< {0,1} Determines if logstream is saved into a file.
#define SOLVERLOG_TOCONSOLE 0         ///< {0,1} Determines if logstream is outputted into console.

#else
#define LOGBC(logptr, message)  // LOG(logptr,message) ;
#define LOGBC_SZ(logptr, message)  // LOG(logptr,message) ; ///< Macro to easily write into a SimpleLogger::Log (only
// for the serialization part).

#define LOGIC(logptr, message)     // LOG(logptr,message) ;
#define LOGIC_SZ(logptr, message)  // LOG(logptr,message) ;

#define LOGPC(logptr, message)  // LOG(logptr,message)

/// SceneParser
#define SCENEPARSER_LOGLEVEL 2  /// 0 - No output, 1 basic output, 2 medium output, 3 full output

/// Topobuilder
#define TOPOBUILDER_LOGLEVEL 2  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
#define TOPOLOGY_BUILDER_WRITE_TOPO
#define TOPOLOGY_BUILDER_WRITE_PREDICTED_POINTS
#define TOPOLOGY_BUILDER_WRITE_NEAREST_DISTANCES

/// Solver
#define SOLVERLOG_LOGLEVEL 1          /// 0 - No output, 1 basic output, 2 medium output, 3 full output
#define SOLVERLOG_LOGLEVEL_CONTACT 0  /// 0 - No output, 1 basic output, 2 medium output, 3 full output
#define SOLVERLOG_TOFILE 1
#define SOLVERLOG_TOCONSOLE 0

#endif

/** TopoBuilder Log Macros */
#define LOGTB(logptr, message) \
    LOG(logptr, message);  ///< Macro to easily write into a SimpleLogger::Log (only for the topology builder part).
#define LOGTBLEVEL(level, logptr, message) LOGLEVEL(level, TOPOBUILDER_LOGLEVEL, logptr, message);
#define LOGTBLEVEL1(logptr, message) LOGTBLEVEL(1, logptr, message);
#define LOGTBLEVEL2(logptr, message) LOGTBLEVEL(2, logptr, message);
#define LOGTBLEVEL3(logptr, message) LOGTBLEVEL(3, logptr, message);

/** SceneParser Log Macros */
#define LOGSC(log, message) LOG(log, message);
#define LOGSCLEVEL(level, logptr, message) LOGLEVEL(level, SCENEPARSER_LOGLEVEL, logptr, message);
#define LOGSCLEVEL1(logptr, message) LOGSCLEVEL(1, logptr, message);
#define LOGSCLEVEL2(logptr, message) LOGSCLEVEL(2, logptr, message);
#define LOGSCLEVEL3(logptr, message) LOGSCLEVEL(3, logptr, message);
#define SKIPLOGSC(logptr, message) LOGSCLEVEL(1, logptr, message);

/** Inclusion Solver Log Macros */
#define LOGSL(log, message) LOG(log, message);
#define LOGSLLEVEL(level, logptr, message) LOGLEVEL(level, SOLVERLOG_LOGLEVEL, logptr, message);
#define LOGSLLEVEL1(logptr, message) LOGSLLEVEL(1, logptr, message);
#define LOGSLLEVEL2(logptr, message) LOGSLLEVEL(2, logptr, message);
#define LOGSLLEVEL3(logptr, message) LOGSLLEVEL(3, logptr, message);

#define LOGSLLEVEL_CONTACT(level, logptr, message) LOGLEVEL(level, SOLVERLOG_LOGLEVEL_CONTACT, logptr, message);
#define LOGSLLEVEL1_CONTACT(logptr, message) LOGSLLEVEL(1, logptr, message);
#define LOGSLLEVEL2_CONTACT(logptr, message) LOGSLLEVEL(2, logptr, message);
#define LOGSLLEVEL3_CONTACT(logptr, message) LOGSLLEVEL(3, logptr, message);

/* @} */

/** @name File and Folders
* @brief
*/
/* @{ */
#define PROCESS_FOLDER_PREFIX "ProcessMPI_"
#define GLOBAL_LOG_FOLDER_DIRECTORY "./Logs"

#define SIMULATION_FOLDER_PATH "./SimFiles"  ///< Directory where new simulation folders are place during record.
#define SIM_SCENE_FILE_NAME "SceneFile"      ///< File name for the Scene xml file which describes the scene
#define SIM_FOLDER_PREFIX_RECORD "SimDataRECORDMPI_"  ///< Prefix for the simulation folder during record.
#define SIM_FOLDER_PREFIX_INIT \
    "SimDataInitialState_"  ///< Prefix for the folder where the inital states are written, press Key I!
#define SIM_INIT_FILE_PREFIX "InitialState"         ///< The name for the Initial state file!
#define SIM_FILE_PREFIX "SimState"                  ///< Prefix for the .sim file.
#define SOLVER_LOG_FILE_PREFIX "SolverLog"          ///< Prefix for the solver log file.
#define COLLISION_DATA_FILE_PREFIX "CollisionData"  ///< Prefix for the collision data file.
#define SYSTEM_DATA_FILE_PREFIX "SimData"           ///< Prefix for the system data file.
/* @} */

/** @name State data files.
* @brief
*/
/* @{ */
#define OUTPUT_SIMDATA_FILE  ///< Sets if the System Data file is outputted
//#define OUTPUT_SIMDATAITERATION_FILE  ///< Sets if the SimDataIteration file is outputted
#define CALCULATE_COND_OF_G \
    0  ///< {0,1} Set if the condition of the G matrix is calculated and outputted. Takes alot of time!
#define CALCULATE_DIAGDOM_OF_G \
    1  ///< {0,1} Set if the diagonal dominant criteria is calculated, the number shows how many rows are not diagonal
       /// dominant!
#define MEASURE_TIME_PROX
//#define MEASURE_TIME_PROX_DETAIL

//#define OUTPUT_COLLISIONDATA_FILE       ///< Sets if the Collision Data file is outputted
/* @} */

/** @name MPI Saftey Checks */
/* @{ */
#ifndef NDEBUG
// DEBUG!
#define SAFETY_CHECK_SPLITBODYUPDATE
#else

#endif
/* @} */

/** @name  Destructor and Constructor Macros */
/* @{ */
#ifndef NDEBUG
#define DESTRUCTOR_MESSAGE std::cerr << "Destructor: " << typeid(*this).name() << " , @ : " << this << std::endl;
#define CONSTRUCTOR_MESSAGE std::cerr << "Constructor: " << typeid(*this).name() << " , @ : " << this << std::endl;
#else
#define DESTRUCTOR_MESSAGE
#define CONSTRUCTOR_MESSAGE
#endif
/* @} */

/* @} */
#endif
