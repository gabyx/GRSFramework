#ifndef GRSF_Dynamics_Inclusion_ContactGraphNodeData_hpp
#define GRSF_Dynamics_Inclusion_ContactGraphNodeData_hpp


#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/AssertionDebug.hpp"

#include RigidBodySolverData_INCLUDE_FILE

#include "GRSF/Dynamics/Inclusion/ContactParameter.hpp"
#include "GRSF/Dynamics/Inclusion/ContactPercussion.hpp"

#include "GRSF/Dynamics/Buffers/FrontBackBuffer.hpp"
#include "GRSF/Dynamics/Collision/CollisionData.hpp"

// This two data classes are used for  m_eContactModel = ContactModels::N_ContactModel,
//                                      m_eContactModel = ContactModels::UCF,
//                                      m_eContactModel = ContactModels::UCFC_ContactModel,
class ContactGraphNodeDataUCFBase {
public:

    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // not required

    MatrixUBody3 m_W_body[2];
    Vector3 m_chi;

    Vector3  m_eps;

    unsigned int m_nodeColor;
    const CollisionData * m_pCollData = nullptr;
    ContactParameter m_contactParameter;

    ContactPercussion * m_cache = nullptr;
};


class ContactGraphNodeDataUCF : public ContactGraphNodeDataUCFBase {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphNodeDataUCF()/*:
        m_LambdaBack(m_LambdaBack_internal.data()),
        m_LambdaFront(m_LambdaFront_internal.data())*/
    {
        m_uBufferPtr[0] = nullptr; ///< Points to the velocity buffer only if the body is simulated
        m_uBufferPtr[1] = nullptr; ///< Points to the velocity buffer only if the body is simulated
        m_bConverged = false; ///< Flag if convergence criteria is fulfilled, either InVelocityLocal, InLambda, InEnergyMix (with Lambda, and G_ii)

    }


    typename BodySolverDataType::VelocityBufferType * m_uBufferPtr[2]; ///< Pointers into the right Front BackBuffer for bodies 1 and 2

    Vector3 m_LambdaBack;
    Vector3 m_LambdaFront;
    //    MatrixMap<Vector3> m_LambdaBack; // Uncomment for faster swap with MatrixMap
    //    MatrixMap<Vector3> m_LambdaFront;

    Vector3  m_R_i_inv_diag; // Build over G_ii
    Matrix33 m_G_ii;        // just for R_ii, and maybee later for better solvers!

    Vector3 m_b;

    bool m_bConverged; ///< Converged either in LambdaLocal (lambdaNew to lambdaOld), or

   void swapVelocities() {
        if(m_uBufferPtr[0]){ m_uBufferPtr[0]->m_back.swap(m_uBufferPtr[0]->m_front); }
        if(m_uBufferPtr[1]){ m_uBufferPtr[1]->m_back.swap(m_uBufferPtr[1]->m_front); }
    };

    void swapLambdas() {
        m_LambdaBack.swap(m_LambdaFront);
    };

private:

//    Vector3 m_LambdaBack_internal;
//    Vector3 m_LambdaFront_internal;

};

/*
* The EdgeData class for the Contact Graph, nothing is deleted in this class, this is plain old data!
*/
class ContactGraphEdgeData {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    ContactGraphEdgeData(): m_pBody(nullptr) {};

    RigidBodyType * m_pBody; // Tells to which body this edges belongs!

};


class ContactGraphNodeDataDriftCorrector{
public:

    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // not required

    VectorUBody  m_W_body[2];
    PREC m_chi;

    const CollisionData * m_pCollData = nullptr;
};


#endif
