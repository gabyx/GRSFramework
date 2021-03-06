// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_AssertionDebug_hpp
#define GRSF_common_AssertionDebug_hpp

// Add an Assertion Debuggin!

//#define NDEBUG
#include <assert.h>
#include <iostream>
#include <stdlib.h>
#include <typeinfo>

#include <mpi.h>

#include "GRSF/common/Exception.hpp"
#include "GRSF/common/LogDefines.hpp"

// This is the MPI version for our Debugging shit =)

#ifndef NDEBUG
/**
    * @brief An Assert Macro to use within C++ MPI code.
    * @param condition The condition which needs to be truem otherwise an assertion is thrown!
    * @param message The message in form of cout out expression like: "Variable" << i<< "has failed"
    */

#define LOGASSERTMSG(condition, log, message) \
    {                                         \
        if (!(condition))                     \
        {                                     \
            LOG(log, message);                \
            GRSF_ERRORMSG(message);           \
        }                                     \
    }

#define GRSF_ASSERTMSG(condition, message) \
    {                                      \
        if (!(condition))                  \
        {                                  \
            GRSF_ERRORMSG(message);        \
        }                                  \
    }

#define WARNINGMSG(condition, message)                                              \
    {                                                                               \
        if (!(condition))                                                           \
        {                                                                           \
            std::cerr << "WARNING: " << #condition << " : " << std::endl            \
                      << message << std::endl                                       \
                      << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl; \
        }                                                                           \
    }

#else
//#define GRSF_ASSERTMSG(condition,message);
//#define WARNINGMSG(condition,message);
//#define LOGASSERTMSG( _log_ , _assert_ , _statement_ );

#define LOGASSERTMSG(condition, log, message) \
    {                                         \
        if (!(condition))                     \
        {                                     \
            LOG(log, message);                \
            GRSF_ERRORMSG(message);           \
        }                                     \
    }

#define GRSF_ASSERTMSG(condition, message) \
    {                                      \
        if (!(condition))                  \
        {                                  \
            GRSF_ERRORMSG(message);        \
        }                                  \
    }

#define WARNINGMSG(condition, message)                                              \
    {                                                                               \
        if (!(condition))                                                           \
        {                                                                           \
            std::cerr << "WARNING: " << #condition << " : " << std::endl            \
                      << message << std::endl                                       \
                      << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl; \
        }                                                                           \
    }

#endif

/**
    * @brief An Error Macro to use within C++ for MPI code.
    * Writes in a global file!
    */
#define GRSF_ERRORMSG(_message_) GRSF_THROWEXCEPTION(_message_);

#define GRSF_ERRORMSG2(_message1_, _message2_) GRSF_ERRORMSG(_message1_ << _message2_)

#ifndef NDEBUG
/**
    * @brief An Assert Macro for MPI routines to use within C++ MPI code.
     * Writes in a global file!
    */
#define ASSERTMPIERROR(error_code, message)                \
    {                                                      \
        if (error_code != MPI_SUCCESS)                     \
        {                                                  \
            char* string = nullptr;                        \
            int length;                                    \
            MPI_Error_string(error_code, string, &length); \
            GRSF_ERRORMSG2(string, message);               \
        }                                                  \
    }

#else
//#define ASSERTMPIERROR( error_code , message );

#define ASSERTMPIERROR(error_code, message)                \
    {                                                      \
        if (error_code != MPI_SUCCESS)                     \
        {                                                  \
            char* string = nullptr;                        \
            int length;                                    \
            MPI_Error_string(error_code, string, &length); \
            GRSF_ERRORMSG2(string, message);               \
        }                                                  \
    }

#endif

#endif
