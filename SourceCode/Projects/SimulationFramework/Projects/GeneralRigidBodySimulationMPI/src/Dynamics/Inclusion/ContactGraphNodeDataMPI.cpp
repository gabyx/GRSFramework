#include "ContactGraphNodeDataMPI.hpp"

#include <boost/preprocessor/iteration/local.hpp>
#include <boost/preprocessor/cat.hpp>

#include <vector>

namespace LMatrixGenerator{

    DEFINE_LAYOUT_CONFIG_TYPES

    #include "LMatrixGeneration/generate_LMatrix_Multiplicity.hpp"

};

// needed whitespace here----*
#define BOOST_PP_LOCAL_LIMITS (2, 8)

#define FUNC_PREFIX \
    LMatrixGenerator::generate_LMatrix_Multiplicity_
#define APPLY(n,v) n(v)
#define GENERATE_FUNCTION_NAME(n) \
    APPLY( BOOST_PP_CAT(FUNC_PREFIX , n ) , v );


#define BOOST_PP_LOCAL_MACRO(n) \
    v.clear(); \
    GENERATE_FUNCTION_NAME(n) \
    BOOST_PP_CAT(L,n).setFromTriplets(v.begin(),v.end()); \

//Constructor for the LMatrices
ContactGraphNodeDataSplitBody::LMatrices::LMatrices(){

    //Generate Matrix
    std::vector<MatrixSparseTriplet> v;
    #include BOOST_PP_LOCAL_ITERATE()
}



// Static initializer for the LMatrices
ContactGraphNodeDataSplitBody::LMatrices ContactGraphNodeDataSplitBody::m_LMatrices;



#undef BOOST_PP_LOCAL_LIMITS
#undef GENERATE_FUNCTION_NAME
#undef BOOST_PP_LOCAL_MACRO
