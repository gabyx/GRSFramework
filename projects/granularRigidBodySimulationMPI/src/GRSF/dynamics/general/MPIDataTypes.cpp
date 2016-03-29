// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <mpi.h>
#include "GRSF/dynamics/general/MPIDataTypes.hpp"


namespace MPILayer{

    MPI_Datatype DataTypes::MPIVector3 = MPI_DATATYPE_NULL;


    MPI_Op ReduceFunctions::MinVector3 = MPI_OP_NULL;
    MPI_Op ReduceFunctions::MaxVector3 = MPI_OP_NULL;

};
