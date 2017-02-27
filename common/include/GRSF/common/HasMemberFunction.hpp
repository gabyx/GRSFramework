// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_HasMemberFunction_hpp
#define GRSF_common_HasMemberFunction_hpp

/** Check if a template type has member function defined by the use of SFINAE
 *
 * DEFINE_HAS_MEMBER_FUNCTION( stream )
 * struct X{ void stream(int){} };
 * struct Y{};
 * static_assert(hasMemberFunction_stream<X, int>() == true, "fail X");
 * static_assert(hasMemberFunction_stream<Y, int>() == false, "fail Y");
 *
 *
 */

#define DEFINE_HAS_MEMBER_FUNCTION_INTERNAL_STRUCTNAME(NAME) hasMemberFunctionDetails_##NAME

#define DEFINE_HAS_MEMBER_FUNCTION(NAME)                                                                   \
                                                                                                           \
    struct DEFINE_HAS_MEMBER_FUNCTION_INTERNAL_STRUCTNAME(NAME)                                            \
    {                                                                                                      \
        template <class>                                                                                   \
        struct sfinae_true : std::true_type                                                                \
        {                                                                                                  \
        };                                                                                                 \
                                                                                                           \
        template <typename TTT, typename... Args>                                                          \
        static auto test(int) -> sfinae_true<decltype(std::declval<TTT>().NAME(std::declval<Args>()...))>; \
                                                                                                           \
        template <typename, typename... Args>                                                              \
        static auto test(long) -> std::false_type;                                                         \
    };                                                                                                     \
                                                                                                           \
    template <typename TTT, typename... Args>                                                              \
    struct hasMemberFunction_##NAME                                                                        \
        : decltype(DEFINE_HAS_MEMBER_FUNCTION_INTERNAL_STRUCTNAME(NAME)::template test<TTT, Args...>(0))   \
    {                                                                                                      \
    };

#endif
