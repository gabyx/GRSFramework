// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/common/CPUTimer.hpp"

void eigenSpeedTest()
{
    using Idx = Eigen::Array<int, 3, 1>;

    Idx max;
    max.setRandom();
    Idx idx;
    idx.setRandom();
    Idx t;
    t.setZero();

    auto loops = 5000000;

    Idx zero;
    zero.setZero();
    srand(5);
    START_TIMER(start)
    EIGEN_ASM_COMMENT("BEGIN1");
    for (auto k = 0; k < loops; ++k)
    {
        idx.setRandom();
        t += zero.max(max.min(idx));
    }
    EIGEN_ASM_COMMENT("END1");

    STOP_TIMER_MILLI(count, start)
    std::cout << "Eigen3: time: " << count << " ms " << t << std::endl;

    //    t.setZero();
    //    srand(5);
    //    START_TIMER(start2)
    //    EIGEN_ASM_COMMENT("BEGIN2");
    //    for(auto k=0; k<loops;++k){
    //        idx.setRandom();
    //        t(0) += std::max(   std::min( max(0), idx(0)),  0   );
    //        t(1) += std::max(   std::min( max(1), idx(1)),  0   );
    //        t(2) += std::max(   std::min( max(2), idx(2)),  0  );
    //    }
    //    EIGEN_ASM_COMMENT("END2");
    //    STOP_TIMER_MILLI(count2,start2)
    //
    //    std::cout << "Seperate: time: " << count2 << " ms " << t << std::endl;
}
