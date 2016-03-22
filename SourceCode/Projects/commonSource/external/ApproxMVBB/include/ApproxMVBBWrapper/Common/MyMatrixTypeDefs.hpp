// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef ApproxMVBB_Common_MyMatrixDefs_hpp
#define ApproxMVBB_Common_MyMatrixDefs_hpp

#include "GRSF/Dynamics/General/MyMatrixTypeDefs.hpp"

namespace ApproxMVBB{

    template<typename PREC>
    using MyMatrix = MyMatrix<PREC>;


    using MyMatrixIOFormat = MyMatrixIOFormat;

};


#define ApproxMVBB_DEFINE_MATRIX_TYPES_OF( _PREC_ ) DEFINE_MATRIX_TYPES_OF( _PREC_ ) 

#endif

