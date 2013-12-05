#ifndef ContactFeasibilityTable_hpp
#define ContactFeasibilityTable_hpp


#include "CompileTimeArray.hpp"

/**
* @ingroup Inclusion
* @brief These namespaces define compile-time tables to determine the feasibility of certain contact configurations
* which can be checked by the boolean value int the array.
*/

// A table for the Single Implementation has not been made so far, it is not needed


namespace ContactFeasibilityTableMPI{

/*                |------> i2
                       Locals  ||  Remotes
                   ==========================
                    sim|sta|ani||sim|sta|ani|
 _                 ==========================
 |    L   simulated| 1 | 1 | x || 1 | 0 | x |
 |    o      static| 1 | 0 | x || 1 | 0 | x |
 |    c.   animated| x | x | x || x | x | x |
 |        ---------------------------------------
 |    R   simulated| 1 | 1 | x || 0 | 0 | x |
 i1   e      static| 0 | 0 | x || 0 | 0 | x |
      m.   animated| x | x | x || x | x | x |

      1= feasible or allowed
      x= not implemented

*/

    DEFINE_RIGIDBODY_CONFIG_TYPES
    typedef RigidBodyType::BodyState BS;

    // 2D to 1D index with length L (store symetric matrix as 1D array)
    template<char L, char i1, char i2>
    constexpr char make1DIndexSym(){
        return i1*L +i2 - i1*(i1+1)/2;
    }

    //Generator
    struct Generator{
        static constexpr bool generate(size_t index){

            return
                (//Sim Local - Sim Local
                 index==make1DIndexSym<(char)BS::NSTATES, (char)BS::SIMULATED, (char)BS::SIMULATED>() ? true :
                    (
                    // Sim Local - Static Local
                    index==make1DIndexSym<(char)BS::NSTATES,(char)BS::SIMULATED, (char)BS::STATIC>() ? true :
                        (
                        // Sim Local - Sim Remote ( + BS::NSTATES)
                        index==make1DIndexSym<(char)BS::NSTATES,(char)BS::SIMULATED, (char)BS::SIMULATED + (char)BS::NSTATES>() ? true :
                            (
                                 // Static Local - Sim Remote ( + BS::NSTATES)
                                 index == make1DIndexSym<(char)BS::NSTATES,(char)BS::STATIC, (char)BS::SIMULATED + (char)BS::NSTATES>() ? true : false
                            )
                        )
                    )
                );
        }
    };
    static const size_t size = (2*(size_t)BS::NSTATES* (2*(size_t)BS::NSTATES + 1) / 2);
    typedef CompileTimeArray::Array<Generator, size > Array;

    template<typename Stream>
    void printArray(Stream & os){
        os << "ContactFeasibilityTableMPI: [";
        for(int i=0; i<size-1;i++){
            os << Array::values[i];
            os << ",";
        }
        os <<Array::values[size-1] << " ]" << std::endl;
    };
};


#endif // ContactFeasibilityTable_hpp
