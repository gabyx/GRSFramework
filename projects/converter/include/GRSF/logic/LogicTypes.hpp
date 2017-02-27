// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_logic_LogicTypes_hpp
#define GRSF_logic_LogicTypes_hpp

#include <memory>

#include <boost/mpl/at.hpp>
#include <boost/mpl/find.hpp>
#include <boost/mpl/joint_view.hpp>
#include <boost/mpl/vector.hpp>

#include <boost/filesystem.hpp>

#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/converters/renderer/RenderMaterial.hpp"

namespace LogicTypes
{
DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

// All main types =========================================================================
using TypeSeq = boost::mpl::vector<double,
                                   float,
                                   bool,
                                   char,
                                   short,
                                   int,
                                   long int,
                                   long long int,
                                   unsigned char,
                                   unsigned short,
                                   unsigned int,
                                   unsigned long int,
                                   unsigned long long int,
                                   std::string,
                                   boost::filesystem::path,
                                   Vector3,
                                   Quaternion,
                                   VectorQBody,
                                   VectorUBody,
                                   RenderMaterial*>;

// =========================================================================================

// The basic types (for SFINAE)
using TypeSeqBasic = boost::mpl::vector<double,
                                        float,
                                        bool,
                                        char,
                                        short,
                                        int,
                                        long int,
                                        long long int,
                                        unsigned char,
                                        unsigned short,
                                        unsigned int,
                                        unsigned long int,
                                        unsigned long long int,
                                        std::string,
                                        boost::filesystem::path>;

// All arithmetic types of TypeSeq (for SFINAE)
using TypeSeqArithmetic = boost::mpl::vector<double,
                                             float,
                                             bool,
                                             char,
                                             short,
                                             int,
                                             long int,
                                             long long int,
                                             unsigned char,
                                             unsigned short,
                                             unsigned int,
                                             unsigned long int,
                                             unsigned long long int>;

// All string assignable types, without conversion! (for SFINAE)
using TypeSeqStringAssignable = boost::mpl::vector<std::string, boost::filesystem::path>;

// TODO Make this meta:: dependent! and more nice!

#define LOGICSOCKET_CASE_SWITCH(N)                                \
    case N:                                                       \
    {                                                             \
        typedef typename boost::mpl::at_c<TypeSeq, N>::type Type; \
        visitor(castToType<Type>());                              \
    }                                                             \
    break;

#define LOGICSOCKET_APPLY_VISITOR_SWITCH                                                       \
                                                                                               \
    switch (this->m_type)                                                                      \
    {                                                                                          \
        LOGICSOCKET_CASE_SWITCH(0)                                                             \
        LOGICSOCKET_CASE_SWITCH(1)                                                             \
        LOGICSOCKET_CASE_SWITCH(2)                                                             \
        LOGICSOCKET_CASE_SWITCH(3)                                                             \
        LOGICSOCKET_CASE_SWITCH(4)                                                             \
        LOGICSOCKET_CASE_SWITCH(5)                                                             \
        LOGICSOCKET_CASE_SWITCH(6)                                                             \
        LOGICSOCKET_CASE_SWITCH(7)                                                             \
        LOGICSOCKET_CASE_SWITCH(8)                                                             \
        LOGICSOCKET_CASE_SWITCH(9)                                                             \
        LOGICSOCKET_CASE_SWITCH(10)                                                            \
        LOGICSOCKET_CASE_SWITCH(11)                                                            \
        LOGICSOCKET_CASE_SWITCH(12)                                                            \
        LOGICSOCKET_CASE_SWITCH(13)                                                            \
        LOGICSOCKET_CASE_SWITCH(14)                                                            \
        LOGICSOCKET_CASE_SWITCH(15)                                                            \
        LOGICSOCKET_CASE_SWITCH(16)                                                            \
        LOGICSOCKET_CASE_SWITCH(17)                                                            \
        LOGICSOCKET_CASE_SWITCH(18)                                                            \
        LOGICSOCKET_CASE_SWITCH(19)                                                            \
        default:                                                                               \
            GRSF_ERRORMSG("TYPE: " << this->m_type << " not implemented in switch statement"); \
    };

// TypesErrorString
static const char* getTypeName(unsigned int i)
{
    static const char* types[] = {"double",
                                  "float",
                                  "bool",
                                  "char",
                                  "short",
                                  "int",
                                  "long int",
                                  "long long int",
                                  "unsigned char",
                                  "unsigned short",
                                  "unsigned int",
                                  "unsigned long int",
                                  "unsigned long long int",
                                  "string",
                                  "path",
                                  "Vector3",
                                  "Quaternion",
                                  "VectorQBody",
                                  "VectorUBody",
                                  "RenderMaterial *"};
    return types[i];
}
template <typename T>
static const char* getTypeName()
{
    typedef typename boost::mpl::find<TypeSeq, T>::type iter;
    return getTypeName(iter::pos::value);
}
};

#endif  // LogicTypes_hpp
