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

#include "TestFunctions.hpp"

#include "GRSF/common/CPUTimer.hpp"
#include "GRSF/common/CommonFunctions.hpp"

GRSF_TEST(StringParseTest, Test1)
{
    const std::string         s             = "1 2 3 4 5 6 7 8.5 9.2 10";
    const std::vector<int>    s_checkInt    = {1, 2, 3, 4, 5, 6, 7, 8.5, 9.2, 10};
    const std::vector<double> s_checkDouble = {1, 2, 3, 4, 5, 6, 7, 8.5, 9.2, 10};

    {
        typename MyMatrix::Vector3<double> v;
        auto                               succ = Utilities::stringToType(v, s);
        EXPECT_FALSE(succ);
        EXPECT_DOUBLE_EQ(v[0], s_checkDouble[0]);
        EXPECT_DOUBLE_EQ(v[1], s_checkDouble[1]);
        EXPECT_DOUBLE_EQ(v[2], s_checkDouble[2]);
    }

    {
        std::set<int> ss;
        auto          succ = Utilities::stringToType(ss, s);
        EXPECT_FALSE(succ);
        EXPECT_TRUE(ss.size() == 7);

        for (auto& f : ss)
        {
            auto it = std::find(s_checkInt.begin(), s_checkInt.end(), f);
            if (it == s_checkInt.end())
            {
                EXPECT_TRUE(false) << "not found!";
            }
        }
    }

    {
        const std::string                s = "true false true";
        typename MyMatrix::Vector3<bool> v;
        auto                             succ = Utilities::stringToType(v, s);
        EXPECT_TRUE(succ);
        EXPECT_TRUE(v.size() == 3);
        EXPECT_TRUE(v[0] == true);
        EXPECT_TRUE(v[1] == false);
        EXPECT_TRUE(v[2] == true);
    }

    {
        std::vector<double> v;
        auto                succ = Utilities::stringToType(v, s);
        EXPECT_TRUE(succ);
        EXPECT_TRUE(v.size() == s_checkDouble.size());
        for (int i = 0; i < v.size(); ++i)
        {
            EXPECT_DOUBLE_EQ(v[i], s_checkDouble[i]);
        }
    }

    {
        const std::string s = "0 1 false true ";
        std::vector<bool> v;
        auto              succ = Utilities::stringToType(v, s);
        EXPECT_TRUE(succ);
        EXPECT_TRUE(v.size() == 4);
        EXPECT_TRUE(v[0] == false);
        EXPECT_TRUE(v[1] == true);
        EXPECT_TRUE(v[2] == false);
        EXPECT_TRUE(v[3] == true);
    }

    {
        const std::string     s = "1,3 0,4 0,7 2,3123";
        std::vector<uint64_t> v;
        auto                  succ =
            Utilities::stringToType<std::vector<uint64_t>, Utilities::CommaSeperatedPairBinShift<uint64_t, uint32_t>>(
                v, s);
        EXPECT_TRUE(succ);
        EXPECT_TRUE(v.size() == 4);
        EXPECT_TRUE(v[0] == (1UL << 32 | 3));
        EXPECT_TRUE(v[1] == (0UL << 32 | 4));
        EXPECT_TRUE(v[2] == (0UL << 32 | 7));
        EXPECT_TRUE(v[3] == (2UL << 32 | 3123));
    }

    {
        const std::string     s = "1,3 0,4 0,7 1,2,4";
        std::vector<uint64_t> v;
        auto                  succ =
            Utilities::stringToType<std::vector<uint64_t>, Utilities::CommaSeperatedPairBinShift<uint64_t, uint32_t>>(
                v, s);
        EXPECT_FALSE(succ);
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
