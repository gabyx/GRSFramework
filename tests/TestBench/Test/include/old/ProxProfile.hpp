// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef ProxProfile_hpp
#define ProxProfile_hpp

#include <chrono>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <type_traits>

#include <cmath>
#include <limits>

#include <chrono>

// THIS ONE SHOULD BE BETTER!
template <typename PREC, typename Derived>
static void inline doProxSingle1(const PREC& radius,
                                 const Eigen::MatrixBase<Derived>& y,
                                 Eigen::MatrixBase<Derived>& out)
{
    PREC absvalue = y.squaredNorm();
    if (absvalue > radius * radius)
    {
        out = y / (sqrt(absvalue) * radius);
        // out *= sqrt(absvalue);
    }
}

template <typename PREC, typename Derived>
static void inline doProxSingle2(const PREC& radius,
                                 const Eigen::MatrixBase<Derived>& y,
                                 Eigen::MatrixBase<Derived>& out)
{
    PREC absvalue = y.norm();
    if (absvalue > radius)
    {
        out = y / (absvalue * radius);
        // out *= absvalue;
    }
}

// THIS ONE SHOULD BE EVEN BETTER (only one division)!
template <typename PREC, typename Derived>
static void inline doProxSingle3(const PREC& radius,
                                 const Eigen::MatrixBase<Derived>& y,
                                 Eigen::MatrixBase<Derived>& out)
{
    PREC absvalue = y.squaredNorm();
    if (absvalue > radius * radius)
    {
        out *= (radius / sqrt(absvalue));
        // out *= sqrt(absvalue);
    }
}

template <typename PREC, typename Derived>
static void inline doProxSingle4(const PREC& radius,
                                 const Eigen::MatrixBase<Derived>& y,
                                 Eigen::MatrixBase<Derived>& out)
{
    PREC absvalue = y.norm();
    if (absvalue > radius)
    {
        out *= (radius / absvalue);
        // out *= absvalue;
    }
}

void proxProfile()
{
    std::cout << " Start profiling prox: " << std::endl;
    typedef std::chrono::high_resolution_clock Clock;
    using unit_t = std::chrono::duration<double, std::milli>;
    using std::chrono::duration_cast;
    Clock clock;

    Eigen::Matrix<double, 2, 1> v1(2, 2);
    Eigen::Matrix<double, 2, 1> r(2, 2);

    // ====================================================
    double radius = 1;
    v1.setConstant(2);
    for (int i = 0; i < 10; i++)
    {
        doProxSingle1(radius, v1, r);
    }
    auto t0 = clock.now();
    for (int i = 0; i < 1e9; i++)
    {
        doProxSingle1(radius, v1, r);
    }
    auto t1 = clock.now();
    r *= 1;
    unit_t ms = duration_cast<unit_t>(t1 - t0);
    std::cout << " doSingleProx1 (sqNorm) Disk proj: " << ms.count() << " ms " << std::endl;
    std::cout << " Res: " << r.transpose() << std::endl;

    radius = 5;
    v1.setConstant(2);
    r.setConstant(2);
    std::cout << " v1: " << v1.transpose() << std::endl;
    for (int i = 0; i < 10; i++)
    {
        doProxSingle1(radius, v1, r);
    }
    t0 = clock.now();
    for (int i = 0; i < 1e9; i++)
    {
        doProxSingle1(radius, v1, r);
    }
    t1 = clock.now();
    r *= 1;
    ms = duration_cast<unit_t>(t1 - t0);
    std::cout << " doSingleProx1 (sqNorm) Disk non-proj: " << ms.count() << " ms " << std::endl;
    std::cout << " Res: " << r.transpose() << std::endl;
    // ====================================================

    // ====================================================
    radius = 1;
    v1.setConstant(2);
    r.setConstant(2);

    for (int i = 0; i < 10; i++)
    {
        doProxSingle2(radius, v1, r);
    }
    t0 = clock.now();
    for (int i = 0; i < 1e9; i++)
    {
        doProxSingle2(radius, v1, r);
    }
    t1 = clock.now();
    r *= 1;
    ms = duration_cast<unit_t>(t1 - t0);
    std::cout << " doSingleProx2 (norm) Disk proj: " << ms.count() << " ms " << std::endl;
    std::cout << " Res: " << r.transpose() << std::endl;
    radius = 5;
    v1.setConstant(2);
    r.setConstant(2);
    for (int i = 0; i < 10; i++)
    {
        doProxSingle2(radius, v1, r);
    }
    t0 = clock.now();
    for (int i = 0; i < 1e9; i++)
    {
        doProxSingle2(radius, v1, r);
    }
    t1 = clock.now();
    r *= 1;
    ms = duration_cast<unit_t>(t1 - t0);
    std::cout << " doSingleProx2 (norm) Disk non-proj: " << ms.count() << " ms " << std::endl;
    std::cout << " Res: " << r.transpose() << std::endl;
    // ====================================================

    // ====================================================
    radius = 1;
    v1.setConstant(2);
    for (int i = 0; i < 10; i++)
    {
        doProxSingle3(radius, v1, r);
    }
    t0 = clock.now();
    for (int i = 0; i < 1e9; i++)
    {
        doProxSingle3(radius, v1, r);
    }
    t1 = clock.now();
    r *= 1;
    ms = duration_cast<unit_t>(t1 - t0);
    std::cout << " doSingleProx3 (sqNorm) Disk proj: " << ms.count() << " ms " << std::endl;
    std::cout << " Res: " << r.transpose() << std::endl;

    radius = 5;
    v1.setConstant(2);
    r.setConstant(2);
    std::cout << " v1: " << v1.transpose() << std::endl;
    for (int i = 0; i < 10; i++)
    {
        doProxSingle3(radius, v1, r);
    }
    t0 = clock.now();
    for (int i = 0; i < 1e9; i++)
    {
        doProxSingle3(radius, v1, r);
    }
    t1 = clock.now();
    r *= 1;
    ms = duration_cast<unit_t>(t1 - t0);
    std::cout << " doSingleProx3 (sqNorm) Disk non-proj: " << ms.count() << " ms " << std::endl;
    std::cout << " Res: " << r.transpose() << std::endl;
    // ====================================================

    // ====================================================
    radius = 1;
    v1.setConstant(2);
    r.setConstant(2);

    for (int i = 0; i < 10; i++)
    {
        doProxSingle4(radius, v1, r);
    }
    t0 = clock.now();
    for (int i = 0; i < 1e9; i++)
    {
        doProxSingle4(radius, v1, r);
    }
    t1 = clock.now();
    r *= 1;
    ms = duration_cast<unit_t>(t1 - t0);
    std::cout << " doSingleProx4 (norm) Disk proj: " << ms.count() << " ms " << std::endl;
    std::cout << " Res: " << r.transpose() << std::endl;
    radius = 5;
    v1.setConstant(2);
    r.setConstant(2);
    for (int i = 0; i < 10; i++)
    {
        doProxSingle4(radius, v1, r);
    }
    t0 = clock.now();
    for (int i = 0; i < 1e9; i++)
    {
        doProxSingle4(radius, v1, r);
    }
    t1 = clock.now();
    r *= 1;
    ms = duration_cast<unit_t>(t1 - t0);
    std::cout << " doSingleProx4 (norm) Disk non-proj: " << ms.count() << " ms " << std::endl;
    std::cout << " Res: " << r.transpose() << std::endl;
    // ====================================================
}

#endif
