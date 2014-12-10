#include <mpi.h>
#include "GMSF/Dynamics/General/MPIDataTypes.hpp"


namespace MPILayer{

    MPI_Datatype DataTypes::MPIVector3 = MPI_DATATYPE_NULL;


    MPI_Op ReduceFunctions::MinVector3 = MPI_OP_NULL;
    MPI_Op ReduceFunctions::MaxVector3 = MPI_OP_NULL;

};
