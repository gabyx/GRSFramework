#ifndef ContactGraphNodeData_hpp
#define ContactGraphNodeData_hpp


#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"


#include "ContactParameter.hpp"

#include "FrontBackBuffer.hpp"
#include "CollisionData.hpp"

// This two data classes are used for  m_eContactModel = ContactModels::N_ContactModel,
//                                      m_eContactModel = ContactModels::UCF_ContactModel,
//                                      m_eContactModel = ContactModels::UCFC_ContactModel,
class ContactGraphNodeData {
public:

    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphNodeData(): m_pCollData(NULL) {
        m_W_body1.setZero();
        m_W_body2.setZero();
        m_chi.setZero();
        m_eps.setZero();
    }

    ContactGraphNodeData(CollisionData * collDataPtr): m_pCollData(collDataPtr) {}

    MatrixUBodyDyn m_W_body1;
    MatrixUBodyDyn m_W_body2;
    VectorDyn m_chi;

    VectorDyn  m_eps;


    unsigned int m_nodeColor;

    const CollisionData * m_pCollData;

    ContactParameter m_contactParameter;
};


class ContactGraphNodeDataIteration : public ContactGraphNodeData {
public:

    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphNodeDataIteration()
    {
        m_LambdaBack.setZero();
        m_LambdaFront.setZero();

        m_b.setZero();

        m_u1BufferPtr = NULL; ///< Points to the velocity buffer only if the body is simulated
        m_u2BufferPtr = NULL; ///< Points to the velocity buffer only if the body is simulated

        m_bConverged = false; ///< Flag if convergence criteria is fulfilled, either InVelocityLocal, InLambda, InEnergyMix (with Lambda, and G_ii)
    }


    ~ContactGraphNodeDataIteration(){

    }

    FrontBackBuffer<VectorUBody,FrontBackBufferPtrType::NoPtr, FrontBackBufferMode::NoConst> * m_u1BufferPtr; ///< Pointers into the right Front BackBuffer for bodies 1 and 2
    FrontBackBuffer<VectorUBody,FrontBackBufferPtrType::NoPtr, FrontBackBufferMode::NoConst> * m_u2BufferPtr; ///< Only valid for Simulated Objects


    VectorDyn m_LambdaBack;
    VectorDyn m_LambdaFront;

    VectorDyn m_R_i_inv_diag; // Build over G_ii
    MatrixDyn m_G_ii; // just for R_ii, and maybee later for better solvers!

    VectorDyn m_b;

    bool m_bConverged; ///< Converged either in LambdaLocal (lambdaNew to lambdaOld), or

   void swapVelocities() {

        if(m_u1BufferPtr){ m_u1BufferPtr->m_back.swap(m_u1BufferPtr->m_front); }

        if(m_u2BufferPtr){m_u2BufferPtr->m_back.swap(m_u2BufferPtr->m_front); }
    };

    void swapLambdas() {
        m_LambdaBack.swap(m_LambdaFront);
    };

};

/*
* The EdgeData class for the Contact Graph, nothing is deleted in this class, this is plain old data!
*/
class ContactGraphEdgeData {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphEdgeData(): m_pBody(NULL) {};

    RigidBodyType * m_pBody; // Tells to which body this edges belongs!


};

#endif