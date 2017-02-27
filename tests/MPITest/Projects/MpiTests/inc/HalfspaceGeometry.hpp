// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef HalfspaceGeometry_hpp
#define HalfspaceGeometry_hpp

#include <TypeDefs.hpp>
#include <boost/serialization/access.hpp>

#include "PlaneGeometry.hpp"

template <class PREC>
class HalfspaceGeometry : public PlaneGeometry<PREC>
{
    public:
    DEFINE_MATRIX_TYPES
    HalfspaceGeometry() : PlaneGeometry<PREC>(){};
    HalfspaceGeometry(const Vector3& n, const Vector3& p) : PlaneGeometry<PREC>(n, p){};

    private:
    friend class boost::serialization::access;
};

#endif
