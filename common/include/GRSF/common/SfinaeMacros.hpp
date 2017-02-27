// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_SfinaeMacros_hpp
#define GRSF_common_SfinaeMacros_hpp

#include <type_traits>

/** use this macro in sfinae selection in templated functions
 *
 *   template<typename TopoType,
 *            typename std::enable_if<MPILayer::isGridTopoBuilder<TopoType>::value>::type * = nullptr
 *            typename std::enable_if<MPILayer::isPolymorphic<TopoType>::value>::type * = nullptr
 *   >
 *   void foo(){}
 *
 *   becomes =>
 *
 *   template<typename TopoType,
 *           SFINAE_ENABLE_IF( MPILayer::isGridTopoBuilder<TopoType>::value ),
 *           SFINAE_ENABLE_IF( MPILayer::isPolymorphic<TopoType>::value ),
 *   >
 *   void foo(){}
 */
#define SFINAE_ENABLE_IF(__meta__) typename std::enable_if<(__meta__)>::type* = nullptr

#endif
