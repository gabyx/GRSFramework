/*
 *  LogDefines.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef LogDefines_hpp
#define LogDefines_hpp


/** @name Global Log Macros
* @brief for SimFileConverter
*/
/* @{ */
#ifndef NDEBUG
  // DEBUG!
  #define LOGSJ(A){ A } // Output SimfileJoiner
#else
  #define LOGSJ(A)
#endif
/* @} */

#endif
