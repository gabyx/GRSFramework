// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MyContainerTypeDefs_hpp
#define GRSF_dynamics_general_MyContainerTypeDefs_hpp

//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/StdVector>
#include <map>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

/** @brief
*	These are some container definitions
*/

namespace MyContainers
{
// Sepcial STL map where the type is 16byte aligned
template <typename Key, typename Type, typename Comp = std::less<Key>>
using StdMapAligned = std::map<Key, Type, Comp, Eigen::aligned_allocator<std::pair<const Key, Type>>>;

// Sepcial STL map where the type is 16byte aligned
template <typename Key, typename Type, typename Hash = std::hash<Key>, typename Pred = std::equal_to<Key>>
using StdUMapAligned = std::unordered_map<Key, Type, Hash, Pred, Eigen::aligned_allocator<std::pair<const Key, Type>>>;

// Special STL vectors where the type is 16byte aligned
template <typename Type>
using StdVecAligned = std::vector<Type, Eigen::aligned_allocator<Type>>;
}

/**
* @brief This macro is used to typedef all custom container types.
*/
#define DEFINE_CONTAINER_TYPES                                                                                 \
                                                                                                               \
    template <typename Key, typename Type, typename Comp = std::less<Key>>                                     \
    using StdMapAligned = MyContainers::StdMapAligned<Key, Type, Comp>;                                        \
                                                                                                               \
    template <typename Key, typename Type, typename Hash = std::hash<Key>, typename Pred = std::equal_to<Key>> \
    using StdUMapAligned = MyContainers::StdUMapAligned<Key, Type, Hash, Pred>;                                \
                                                                                                               \
    template <typename Type>                                                                                   \
    using StdVecAligned = MyContainers::StdVecAligned<Type>;

#endif
