// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "GRSF/common/CPUTimer.hpp"
#include "GRSF/common/CommonFunctions.hpp"

void stringTest()
{
    const std::string s = "1 2 3 4 5 6.234 7 8.2 9 10.123 11";
    {
        typename MyMatrix<double>::Vector3 v;
        Utilities::stringToVector3(v, s);
        std::cout << "v: " << v.transpose() << std::endl;
    }

    std::cout << std::endl << "set:" << std::endl;
    {
        std::set<int> ss;
        Utilities::stringToType(ss, s);
        for (auto& f : ss)
        {
            std::cout << "," << f;
        }
    }

    std::cout << std::endl << "Vector3 bool:" << std::endl;
    {
        const std::string s = "true false true";
        typename MyMatrix<bool>::Vector3 v;
        Utilities::stringToVector3(v, s);
        std::cout << "v: " << v << std::endl;
    }
    std::cout << std::endl << "vector:" << std::endl;
    {
        const std::string s = "1 2 3 4 5 6.234 7 8.2 9 10.123 11";
        std::vector<double> v;
        if (!Utilities::stringToType(v, s))
        {
            std::cout << "ERROR ";
        };
        for (auto& f : v)
        {
            std::cout << "," << f;
        }
    }

    std::cout << std::endl << "vector:" << std::endl;
    {
        const std::string s = "1 2 3 4 5 6.234 7 8.2 9.3 10.123 11";
        std::vector<double> v;
        if (!Utilities::stringToType(v, s))
        {
            std::cout << "ERROR ";
        };
        for (auto& f : v)
        {
            std::cout << "," << f;
        }
    }

    std::cout << std::endl << "vector:" << std::endl;
    {
        const std::string s = "0 1 false true ";
        std::vector<bool> v;
        if (!Utilities::stringToType(v, s))
        {
            std::cout << "ERROR ";
        };
        for (auto f : v)
        {
            std::cout << "," << f;
        }
    }

    {
        std::cout << std::endl << "Special Binary Pairs Test" << std::endl;
        //        const std::string s = "1,3 0,4 0,7 1,2";
        const std::string s = "1,3";
        std::vector<uint64_t> v;
        if (!Utilities::stringToType<std::vector<uint64_t>, Utilities::CommaSeperatedPairBinShift<uint64_t, uint32_t>>(
                v, s))
        {
            std::cout << "ERROR ";
        };
        for (auto& f : v)
        {
            std::cout << "," << f;
        }
    }

    {
        std::cout << std::endl << "Special Binary Pair Test" << std::endl;
        const std::string s = "1,3 0,4 0,7 1,2";
        std::pair<uint64_t, uint64_t> v;
        if (!Utilities::stringToType<std::pair<uint64_t, uint64_t>,
                                     Utilities::CommaSeperatedPairBinShift<uint64_t, uint32_t>>(v, s))
        {
            std::cout << "ERROR ";
        };
        std::cout << v.first << "," << v.second;
    }

    //    auto func = [&](int a){std::cout << a << std::endl;};
    //    foo(func);
}

void doBenchmark()
{
    const int loops = 1000000;
    unsigned int i  = 0;

    START_TIMER(start)
    const std::string s = "1 2 3 4 5 6.234 7 8.2 9.3 10.123 11";
    std::vector<std::string> strings(loops, s);

    for (auto& ss : strings)
    {
        std::vector<double> v;
        Utilities::stringToType(v, ss);
    }
    STOP_TIMER_MILLI(count, start)
    std::cout << "Time: " << count << std::endl;
}
