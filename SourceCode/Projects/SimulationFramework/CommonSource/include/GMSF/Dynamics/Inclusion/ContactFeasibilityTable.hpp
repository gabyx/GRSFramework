#ifndef GMSF_Dynamics_Inclusion_ContactFeasibilityTable_hpp
#define GMSF_Dynamics_Inclusion_ContactFeasibilityTable_hpp

#include "AssertionDebug.hpp"
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
 |    o      static| - | 0 | x || 0 | 0 | x |
 |    c.   animated| - | - | x || x | x | x |
 |        ---------------------------------------
 |    R   simulated| - | - | - || 1 | 0 | x |
 i1   e      static| - | - | - || - | 0 | x |
      m.   animated| - | - | - || - | - | x |

      1= feasible or allowed
      0= infeasible
      x= not implemented
      -= symmetric value

*/

    DEFINE_RIGIDBODY_CONFIG_TYPES
    using BS = RigidBodyType::BodyMode;

    // 2D to 1D index with length L (store symetric matrix as 1D array)
    template<char L, char i1, char i2>
    constexpr char make1DIndexSym(){
        return i1*L +i2 - i1*(i1+1)/2;
    }

    //Generator
    struct ContactFeasibilityTableMPI_Generator{
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
                                 // Sim Remote ( + BS::NSTATES) - Sim Remote ( + BS::NSTATES)
                                 index == make1DIndexSym<(char)BS::NSTATES,(char)BS::SIMULATED + (char)BS::NSTATES,
                                                                           (char)BS::SIMULATED + (char)BS::NSTATES>() ? true : false
                            )
                        )
                    )
                );
        }
    };
    static const size_t size = (2*(size_t)BS::NSTATES* (2*(size_t)BS::NSTATES + 1) / 2);
    using Array = CompileTimeArray::Array<ContactFeasibilityTableMPI_Generator, size >;

    template<typename Stream>
    void printArray(Stream & os){
        os << "ContactFeasibilityTableMPI: [";
        for(int i=0; i<size-1;i++){
            os << Array::values[i];
            os << ",";
        }
        os <<Array::values[size-1] << " ]" << std::endl;
    };

    template<typename RigidBodyType>
    bool checkFeasibilityOfContact(const RigidBodyType * p1, const RigidBodyType * p2, std::pair<bool,bool> & isRemote ){
        //Define the feasibility table
        using BS = typename RigidBodyType::BodyMode;
        //printArray(std::cout);

        // calculate table index
        char i1 = (char)p1->m_eMode;
        char i2 = (char)p1->m_eMode;

        // add offset if remote
        isRemote.first = false;
        isRemote.second = false;
        if(p1->m_pBodyInfo){
            if(p1->m_pBodyInfo->m_isRemote){
                i1 += (char)BS::NSTATES;
                isRemote.first = true;
            }
        }
        if(p2->m_pBodyInfo){
            if(p2->m_pBodyInfo->m_isRemote){
                i2 += (char)BS::NSTATES;
                isRemote.second = true;
            }
        }

        if(i1>i2){
            std::swap(i1,i2);
        }
        // Index into symetric array data of bools
        ASSERTMSG(i1* (char)BS::NSTATES +i2  - i1*(i1+1)/2 < size, "Index wrong: id1: "<< p1->m_id <<" i1: " << i1 << "id2: " << p2->m_id<< " i2: " << i2 )
        if(Array::values[i1* (char)BS::NSTATES +i2  - i1*(i1+1)/2 ] == true){

            return true;
        }
        return false;
    }


};


#endif // ContactFeasibilityTable_hpp
