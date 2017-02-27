// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_RedirectOutput_hpp
#define GRSF_common_RedirectOutput_hpp

#include <iostream>

class RedirectOutputs
{
    std::ostream&         myStream;
    std::streambuf* const myBuffer;

public:
    RedirectOutputs(std::ostream& lhs, std::ostream& rhs = std::cout) : myStream(rhs), myBuffer(myStream.rdbuf())
    {
        myStream.rdbuf(lhs.rdbuf());
    }

    ~RedirectOutputs()
    {
        myStream.rdbuf(myBuffer);
    }
};

#endif
