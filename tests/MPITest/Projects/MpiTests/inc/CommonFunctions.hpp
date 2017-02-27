// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef CommonFunctions_h
#define CommonFunctions_h

// Includes =================================
#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "TypeDefs.hpp"
// ==========================================

// using namespace std;
// using namespace Eigen;
// using namespace Ogre;

namespace Utilities
{
/**
* @ingroup Common
* @defgroup CommonFunctions Common Functions
* @brief
*/
/* @{ */

/**
* @brief This functions casts a string into another template specifix type.
* @param t The ouput where the casted string is put into.
* @param s The string to cast.
* @param f The format (e.g std::dec).
* @return true if conversion worked, false if not.
*/
template <class T>
bool stringToType(T& t, const std::string& s, std::ios_base& (*f)(std::ios_base&))
{
    std::istringstream iss(s);
    return !(iss >> f >> t).fail();
}

inline bool operator==(const std::string& a, const std::string& b)
{
    return (a.size() == b.size())   // optimization on some platforms
           && (a.compare(b) == 0);  // actual compare
}

inline bool operator==(const std::string& a, const char* b)
{
    return strcmp(a.c_str(), b) == 0;
}
inline bool operator==(const char* a, const std::string& b)
{
    return b == a;
}

template <typename T>
bool stringToType(T& t, const std::string& s)
{
    return stringToType(t, s, std::dec);
}

template <>
bool stringToType<bool>(bool& t, const std::string& s);

/**
* @brief This functions returns a uniformally distributed random number:
* @param low Lower bound of random number.
* @param high Upper bound of random number.
* @return Random number in range [low,high]
*/
double randd(double low, double high);

/**
* @brief Converts a std::vector with column vectors from Eigen into a Eigen Matrix.
*/
template <class PREC, std::size_t M, std::size_t N>
void vec2Mat(const std::vector<Eigen::Matrix<PREC, M, 1>>& vec, Eigen::Matrix<PREC, M, N>& A)
{
    for (int i = 0; i < vec.size(); i++)
    {
        A.col(i) = vec[i];
    }
};

/**
* @brief Converts a std::vector with scalar values in it into a Eigen Vector.
*/
template <class PREC, std::size_t M>
void vec2Vec(const std::vector<PREC>& vec, Eigen::Matrix<PREC, M, 1>& V)
{
    for (int i = 0; i < vec.size(); i++)
    {
        V[i] = vec[i];
    }
};

/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a Vector3.
*/
template <typename PREC>
bool stringToVector3(typename MyMatrix<PREC>::Vector3& vector3, std::string s)
{
    unsigned int i = 0, j;
    PREC         number;

    if (s.empty())
    {
        return false;
    }

    while (s[i] == ' ')
    {
        i++;
        if (i > s.length() - 1)
        {
            break;
        }
    }
    if (i == s.length())
    {
        return false;
    }
    else
    {
        j = i;
        while (s[j] != ' ')
        {
            j++;
            if (j > s.length() - 1)
            {
                break;
            }
        }
        if (!stringToType<PREC>(number, s.substr(i, j - i)))
        {
            return false;
        }
        vector3(0) = number;  ////////////////////////x
        i          = j;

        while (s[i] == ' ')
        {
            i++;
            if (i > s.length() - 1)
            {
                break;
            }
        }
        if (i == s.length())
        {
            return false;
        }
        else
        {
            j = i;
            while (s[j] != ' ')
            {
                j++;
                if (j > s.length() - 1)
                {
                    break;
                }
            }
            if (!stringToType<PREC>(number, s.substr(i, j - i)))
            {
                return false;
            }
            vector3(1) = number;  ////////////////////////y
            i          = j;

            while (s[i] == ' ')
            {
                i++;
                if (i > s.length() - 1)
                {
                    break;
                }
            }
            if (i == s.length())
            {
                return false;
            }
            else
            {
                j = i;
                while (s[j] != ' ')
                {
                    j++;
                    if (j > s.length() - 1)
                    {
                        break;
                    }
                }
                if (!stringToType<PREC>(number, s.substr(i, j - i)))
                {
                    return false;
                }
                vector3(2) = number;  ////////////////////////z
            }
        }
    }
    return true;
}

/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a Vector4.
*/
template <class PREC>
bool stringToVector4(typename MyMatrix<PREC>::Vector4& vector4, std::string s)
{
    unsigned int i = 0, j;
    PREC         number;
    if (s.empty())
    {
        return false;
    }
    while (s[i] == ' ')
    {
        i++;
        if (i > s.length() - 1)
        {
            break;
        }
    }
    if (i == s.length())
    {
        return false;
    }
    else
    {
        j = i;
        while (s[j] != ' ')
        {
            j++;
            if (j > s.length() - 1)
            {
                break;
            }
        }
        if (!stringToType<PREC>(number, s.substr(i, j - i)))
        {
            return false;
        }
        vector4(0) = number;  ////////////////////////x
        i          = j;

        while (s[i] == ' ')
        {
            i++;
            if (i > s.length() - 1)
            {
                break;
            }
        }
        if (i == s.length())
        {
            return false;
        }
        else
        {
            j = i;
            while (s[j] != ' ')
            {
                j++;
                if (j > s.length() - 1)
                {
                    break;
                }
            }
            if (!stringToType<PREC>(number, s.substr(i, j - i)))
            {
                return false;
            }
            vector4(1) = number;  ////////////////////////y
            i          = j;

            while (s[i] == ' ')
            {
                i++;
                if (i > s.length() - 1)
                {
                    break;
                }
            }
            if (i == s.length())
            {
                return false;
            }
            else
            {
                j = i;
                while (s[j] != ' ')
                {
                    j++;
                    if (j > s.length() - 1)
                    {
                        break;
                    }
                }
                if (!stringToType<PREC>(number, s.substr(i, j - i)))
                {
                    return false;
                }
                vector4(2) = number;  ////////////////////////z
                i          = j;

                while (s[i] == ' ')
                {
                    i++;
                    if (i > s.length() - 1)
                    {
                        break;
                    }
                }
                if (i == s.length())
                {
                    return false;
                }
                else
                {
                    j = i;

                    while (s[j] != ' ')
                    {
                        j++;
                        if (j > s.length() - 1)
                        {
                            break;
                        }
                    }
                    if (!stringToType<PREC>(number, s.substr(i, j - i)))
                    {
                        return false;
                    }
                    vector4(3) = number;  ////////////////////////w
                }
            }
        }
    }

    return true;
}
};

/** @} */

#endif
