// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <iostream>

using namespace std;

class A
{
    public:
    virtual void foo()
    {
        cout << "A" << endl;
    };
};

template <typename T>
class B : public A
{
    public:
    void foo()
    {
        cout << "B" << endl;
    };
};

int main()
{
    A* a = new B<int>();
    a->foo();
    system("pause");
}