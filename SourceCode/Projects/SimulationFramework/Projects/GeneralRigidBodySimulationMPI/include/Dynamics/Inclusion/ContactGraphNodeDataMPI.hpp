#ifndef ContactGraphNodeDataMPI_hpp
#define ContactGraphNodeDataMPI_hpp


#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"

#include "ContactGraphNodeData.hpp"

class ContactGraphNodeDataSplitBody {
public:

    DEFINE_MPI_INFORMATION_CONFIG_TYPES
    DEFINE_RIGIDBODY_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphNodeDataSplitBody(RigidBodyType * body): m_pBody(body) {
        m_u_G.setZero();
        m_LambdaBack.setZero();
        m_LambdaFront.setZero();

    };

    ~ContactGraphNodeDataSplitBody() {};

    bool addRank(const RankIdType & rank) {

        auto pairRes = m_partRanks.insert( std::pair<RankIdType,unsigned int>(rank,m_partRanks.size()) );

        return pairRes.second;
    }

    /** plus one because this rank counts also as on split body to the overall body*/
    unsigned int getMultiplicity() {
        return m_partRanks.size() + 1;
    }


    std::set< std::pair<RankIdType,unsigned int> > m_partRanks; ///< Participating ranks to insertion number, defines the multiplicity

    RigidBodyType * m_pBody;

    /** These values get set from all remotes*/
    VectorDyn m_u_G; ///< all velocities of all split bodies m_u_G = [u1,u2,u3,u4,u5...], correspond to rank in vector

    VectorDyn m_LambdaBack;  ///< Bilateral Lambda (Lambda_M_tilde = M⁻¹*Lambda_M
    VectorDyn m_LambdaFront; ///< Bilateral Lambda


    VectorDyn m_gamma; ///< Gamma = [u1-u2, u2-u3, u3-u4,..., un-1- un], u1 is always the velocity of the owner!

    MatrixSparse & getLMatrix(){
        switch(getMultiplicity()){
            case 2:
                return m_LMatrices.L2;
                break;
            case 3:
                return m_LMatrices.L3;
                break;
            case 4:
                return m_LMatrices.L4;
                break;
            case 5:
                return m_LMatrices.L5;
                break;
            case 6:
                return m_LMatrices.L6;
                break;
            case 7:
                return m_LMatrices.L7;
                break;
            case 8:
                return m_LMatrices.L8;
                break;
            default:
                ERRORMSG("Requested LMatrix for multiplicity: " << getMultiplicity() << " which is not implemented!")
        }
    }

private:

    static struct LMatrices{
        LMatrices();
        MatrixSparse L2;
        MatrixSparse L3;
        MatrixSparse L4;
        MatrixSparse L5;
        MatrixSparse L6;
        MatrixSparse L7;
        MatrixSparse L8;

    } m_LMatrices;

};


#endif
