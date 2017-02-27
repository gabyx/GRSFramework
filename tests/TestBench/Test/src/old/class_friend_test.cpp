// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <Eigen/Dense>
#include <iostream>
#include <utility>
#include <vector>

class SomeClass
{
    protected:
    void* m_pValue;

    public:
    SomeClass()
    {
    }

    template <typename T>
    SomeClass(int a){};

    template <typename T>
    operator T(void)
    {
        return (T)m_pValue;
    }
};

int main()
{
    SomeClass scInt(10);
    std::cout << (int)scInt << std::endl;
    SomeClass scChar('a');
    std::cout << (char)scChar << std::endl;
}