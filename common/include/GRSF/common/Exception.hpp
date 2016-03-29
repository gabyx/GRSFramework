// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_Exception_hpp
#define GRSF_common_Exception_hpp

#include <stdexcept>
#include <exception>
#include <string>
#include <sstream>


class Exception : public std::runtime_error {
public:
    Exception(const std::stringstream & ss): std::runtime_error(ss.str()){};
private:

};

#define THROWEXCEPTION( message ) {std::stringstream ___s___ ; ___s___ << message << std::endl << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl; throw Exception(___s___);}



#endif // Exception_hpp
