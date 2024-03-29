// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

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
#define CLEARLOG logstream.str("");                                  ///< Macro to easily write into a Ogre::Log.
#define LOG(_logptr_, _message_) (*(_logptr_)) _message_;            ///< Macro to easily write into a Ogre::Log.
#define OGRE_LOG(_logptr_) (_logptr_)->logMessage(logstream.str());  ///< Macro to easily write into a Ogre::Log.
/* @} */

/** @name File and Folders
* @brief
*/
/* @{ */
#define GLOBAL_LOG_FOLDER_DIRECTORY "./Logs"

#define VIDEO_FOLDER_PATH "./Videos"    ///< Directory where new simulation folders are place during record.
#define VIDEO_FOLDER_PREFIX "Video_"    ///< Directory where the video frames are place during record.
#define SIM_VIDEO_PREFIX "VideoFrame_"  ///< File prefix for the video frames;

#define SIMULATION_FOLDER_PATH "./SimFiles"  ///< Directory where new simulation folders are place during record.
#define SIM_FOLDER_PREFIX_RESAMPLE "SimDataRESAMPLE_"  ///< Directory where the video frames are place during record.
#define SIM_FILE_EXTENSION ".sim"                      ///< File extension for .sim file.
#define SIM_SCENE_FILE_NAME "SceneFile"                ///< File name for the Scene xml file which describes the scene
#define SIM_FOLDER_PREFIX_REALTIME "SimDataREALTIME_"  ///< Prefix for the simulation folder during realtime simulation.
#define SIM_FOLDER_PREFIX_RECORD "SimDataRECORD_"      ///< Prefix for the simulation folder during record.
#define SIM_FOLDER_PREFIX_INIT \
    "SimDataInitialState_"              ///< Prefix for the folder where the inital states are written, press Key I!
#define SIM_INIT_FILE_EXTENSION ".sim"  ///< File extension for .siminit file.
#define SIM_INIT_FILE_PREFIX "InitialState"         ///< The name for the Initial state file!
#define SIM_FILE_PREFIX "SimulationState"           ///< Prefix for the .sim file.
#define SOLVER_LOG_FILE_PREFIX "SolverLog"          ///< Prefix for the solver log file.
#define COLLISION_DATA_FILE_PREFIX "CollisionData"  ///< Prefix for the collision data file.
#define SYSTEM_DATA_FILE_PREFIX "SimulationData"    ///< Prefix for the system data file.
/* @} */

/** @name Solver Threads
* @brief All these defines are used in the solver thread. The output goes into the solver log with filename
* #SOLVER_LOG_FILE_PREFIX
*/
/* @{ */
#ifndef NDEBUG
// DEBUG!
#define CoutLevelSolver 1             ///<   0 for Off,  1 for Basics, 2 for Advanced, 3 for Full Output
#define CoutLevelSolverWhenContact 1  ///<   0 for Off,  1 for Basics, 2 for Advanced, 3 for Full Output
#define LogToFileSolver 1             ///< {0,1} Determines if logstream is saved into a file.
#define LogToConsoleSolver 1          ///< {0,1} Determines if logstream is outputted into console.
#define AllCoutToLogSolver 0  ///< {0,1} Determines if all couts are routed into the logstream (not working yet).
#else
#define CoutLevelSolver 1
#define CoutLevelSolverWhenContact 1
#define LogToFileSolver 1
#define LogToConsoleSolver 1
#define AllCoutToLogSolver 0
#endif
/* @} */

/** @name System Data file for Record Mode only.
* @brief
*/
/* @{ */
#define OUTPUT_SYSTEMDATA_FILE 1  ///< {0,1} Sets if the System Data file is outputted
#define CALCULATE_COND_OF_G \
    0  ///< {0,1} Set if the condition of the G matrix is calculated and outputted. Takes alot of time!
#define CALCULATE_DIAGDOM_OF_G \
    1  ///< {0,1} Set if the diagonal dominant criteria is calculated, the number shows how many rows are not diagonal
       /// dominant!
#define MEASURE_TIME_PROX 1
/* @} */

/** @name System Data file for Record Mode only.
* @brief
*/
/* @{ */
#define OUTPUT_COLLISIONDATA_FILE 0  ///< {0,1} Sets if the Collision Data file is outputted
/* @} */

/** @name Playback Manager
* @brief  Log file of the Playback Manager.
*/
/* @{ */
#define LogToFilePlayback 1     ///< {0,1} Set if log is outputted to file or not.
#define LogToConsolePlayback 1  ///< {0,1} Set if log is outputted to console or not.
/* @} */

/** @name Loader Thread
* @brief Log file of the Loader Thread which is started during playback.
*/
/* @{ */
#define LogToFileLoader 1     ///< {0,1} Set if log is outputted to file or not.
#define LogToConsoleLoader 1  ///< {0,1} Set if log is outputted to console or not.
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

#include <sstream>
#include "RenderContext.hpp"

/** @name  Deconstructor and Constructor Macros
* @brief Deconstructor and Constructor Macros to Debug correct dealloction of objects.
*/
/* @{ */
#ifdef _DEBUG
#define DECONSTRUCTOR_MESSAGE                                                     \
    std::stringstream logstream;                                                  \
    logstream << "Deconstructor: " << typeid(*this).name() << " , at : " << this; \
    RenderContext::getSingletonPtr()->m_pAppLog->logMessage(logstream.str().c_str());

#define CONSTRUCTOR_MESSAGE                                                     \
    std::stringstream logstream;                                                \
    logstream << "Constructor: " << typeid(*this).name() << " , at : " << this; \
    RenderContext::getSingletonPtr()->m_pAppLog->logMessage(logstream.str().c_str());
#else
#define DECONSTRUCTOR_MESSAGE
#define CONSTRUCTOR_MESSAGE
#endif
/* @} */

/** @name State Ring Pool */
/* @{ */
#define LogToFileStateRingPool 0  ///< {0,1} Set if log is outputted to file or not.
/* @} */

/** @name State Pool */
/* @{ */
#define LogToFileStatePool 0  ///< {0,1} Set if log is outputted to file or not.
/* @} */

/* @} */
#endif
