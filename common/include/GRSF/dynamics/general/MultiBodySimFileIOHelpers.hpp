// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MultiBodySimFileIOHelpers_hpp
#define GRSF_dynamics_general_MultiBodySimFileIOHelpers_hpp

#include <Eigen/Dense>

namespace IOHelpers{

//    template<typename Derived>
//    std::ostream & operator<<( std::ostream &os, const Eigen::EigenBase<Derived>  & m){
//        os.write(reinterpret_cast<char const *>(m.derived().data()), m.size()*sizeof(typename Derived::Scalar));
//        return os;
//    }
//
//    template<typename Derived>
//    std::istream &operator>>( std::istream &os, Eigen::EigenBase<Derived>  & m){
//        os.read(reinterpret_cast<char *>(m.derived().data()), m.size()*sizeof(typename Derived::Scalar));
//        return os;
//    }

    template<typename Derived>
    void writeBinary(std::ostream &os, const Eigen::EigenBase<Derived>  & m)
    {
        os.write(reinterpret_cast<char const *>(m.derived().data()), m.size()*sizeof(typename Derived::Scalar));
    }

    template<typename Derived>
    void readBinary(std::istream &os, const Eigen::EigenBase<Derived>  & m)
    {
        os.read(reinterpret_cast<char *>(
                                         const_cast<typename Derived::Scalar*>(m.derived().data())),
                                         m.size()*sizeof(typename Derived::Scalar));
    }
};

#endif
