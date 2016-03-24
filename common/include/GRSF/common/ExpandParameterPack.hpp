
#ifndef GRSF_common_ExpandParameterPack_hpp
#define GRSF_common_ExpandParameterPack_hpp

#include <type_traits>

namespace ExpandParameterPack {
    /** Empty function */
    struct expandType {
        template <typename... T>
        expandType(T&&...) {}
    };
};

/** Use this macro to call a function on parameter pack
*   For example:
*     template<typename...T>
*     void work(T&&... t){
*        // call print on all t's
*        EXPAND_PARAMETERPACK(print(t))
*     }
*
*   The macro creates an empty type expandType and initializing it with a a list of zeros.
*   Initialization guarantees evaluation order!! (if you would call an empty function instead, this does not guarantee evaluation order)
*   Potential side effect: Compiler does not optimize out the empty non used expandType, but that will definitely not happen!

*   Source: http://stackoverflow.com/questions/17339789/how-to-call-a-function-on-all-variadic-template-args
*/

#define EXPAND_PARAMETERPACK(PATTERN) \
        ExpandParameterPack::expandType{ 0, ((PATTERN), void(), 0)... } ;


#endif

