#include <mpi.h>
#include "GRSF/dynamics/general/MPIDataTypes.hpp"


namespace MPILayer{

    MPI_Datatype DataTypes::MPIVector3 = MPI_DATATYPE_NULL;


    MPI_Op ReduceFunctions::MinVector3 = MPI_OP_NULL;
    MPI_Op ReduceFunctions::MaxVector3 = MPI_OP_NULL;

};
