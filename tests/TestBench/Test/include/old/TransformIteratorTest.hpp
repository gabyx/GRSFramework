// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <boost/iterator/transform_iterator.hpp>

#define INIT_TIMER auto start = std::chrono::high_resolution_clock::now();
#define START_TIMER start     = std::chrono::high_resolution_clock::now();
#define STOP_TIMER(name)                                                                                      \
    double count =                                                                                            \
        std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count(); \
    std::cout << "RUNTIME of " << name << ": " << count << " ms " << std::endl;

template <typename Iterator>
int runLoop(Iterator s, Iterator e)
{
    int a;
    for (auto it = s; it != e; it++)
    {
        a += *it;
    }
    return a;
}

template <typename Iterator>
int runLoop2(Iterator s, Iterator e)
{
    int a;
    for (auto it = s; it != e; it++)
    {
        a += it->m_a;
    }
    return a;
}

struct Data
{
    ;
    int m_a    = 1;
    int m_b    = 2;
    double m_c = 3;
};

int getA(Data& d)
{
    return d.m_a;
}

int runTest()
{
    INIT_TIMER

    typedef std::vector<Data> Cont;
    Cont v(10000);
    typedef std::function<int(Data&)> Function;
    typedef boost::transform_iterator<Function, typename Cont::iterator> TransformItType;

    Function getter = &getA;
    int a;
    {
        START_TIMER
        a = runLoop(boost::make_transform_iterator(v.begin(), getter), boost::make_transform_iterator(v.end(), getter));
        STOP_TIMER("runLoop: [transform_iterator]")
    }
    {
        START_TIMER
        a = runLoop2(v.begin(), v.end());
        STOP_TIMER("runLoop: [std::vector]")
        std::cout << a;
    }
}
