// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef TemplateTemplateParameters_hpp
#define TemplateTemplateParameters_hpp

#include <iostream>
#include <vector>

template <typename T>
class A
{
public:
    template <template <typename, typename> class U, typename Allocator>
    void foo(U<T, Allocator>& list)
    {
        typename U<T, Allocator>::iterator it;
        for (it = list.begin(); it != list.end(); it++)
        {
            std::cout << *it << std::endl;
        }
    };
};

void templateTemplateParamsTest()
{
    A<int> a;

    std::vector<int> b;

    a.foo<std::vector>(b);
};

#endif
