#include "ContactGraphNodeDataMPI.hpp"

#include <boost/preprocessor/iteration/local.hpp>
#include <boost/preprocessor/cat.hpp>

#include <vector>

// Dump Matrices
#include <mpi.h>
#include <ostream>
#include <sstream>


#include RigidBody_INCLUDE_FILE


namespace LMatrixGenerator{

    DEFINE_LAYOUT_CONFIG_TYPES

    #include "LMatrixGeneration/generate_LInvMatrix_Multiplicity.hpp"

};

// needed whitespace here----*
#define BOOST_PP_LOCAL_LIMITS (2, 8)

#define FUNC_PREFIX \
    LMatrixGenerator::generate_LInvMatrix_Multiplicity_
#define APPLY(n,v) n(v)
#define GENERATE_FUNCTION_NAME(n) \
    APPLY( BOOST_PP_CAT(FUNC_PREFIX , n ) , v );


#define BOOST_PP_LOCAL_MACRO(n) \
    v.clear(); \
    GENERATE_FUNCTION_NAME(n) \
    LInv.push_back( MatrixSparse( (n-1)*NDOFuBody , (n-1)*NDOFuBody  ) ); \
    LInv.back().setFromTriplets(v.begin(),v.end()); \

//Constructor for the LMatrices
ContactGraphNodeDataSplitBody::LInvMatrices::LInvMatrices(){

    //Generate Matrix
    std::vector<MatrixSparseTriplet> v;

    #include BOOST_PP_LOCAL_ITERATE()


//    // open file and dump matrices :-)
//    MPI_Init(0,0);
//    int rank;
//    MPI_Comm_rank(MPI_COMM_WORLD,&rank);
//    std::stringstream s;
//    s << "LMatrixDump" << rank << ".txt";
//    std::ofstream f(s.str(),std::ios::trunc);
//
//    for(int i=0; i< LInv.size(); i++){
//        f <<"LInv"<<i+2<< ":"<< std::endl << LInv[i] << std::endl;
//    }
//
//    MPI_Finalize();
//    ERRORMSG("Dumped LMatrices!")

}



// Static initializer for the LMatrices
ContactGraphNodeDataSplitBody::LInvMatrices ContactGraphNodeDataSplitBody::m_LInvMatrices;



#undef BOOST_PP_LOCAL_LIMITS
#undef GENERATE_FUNCTION_NAME
#undef BOOST_PP_LOCAL_MACRO
