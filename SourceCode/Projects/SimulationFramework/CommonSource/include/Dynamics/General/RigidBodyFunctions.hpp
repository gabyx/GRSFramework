#ifndef RigidBodyFunctions_hpp
#define RigidBodyFunctions_hpp



namespace RigidBodyFunctions {

    template<typename TRigidBody>
    void initMassMatrixInv(TRigidBody * body)
    {
        // Massmatrix Inverse
        body->m_MassMatrixInv_diag = body->m_MassMatrix_diag.array().inverse().matrix();
    }

    template<typename TRigidBody>
    void initMassMatrixAndHTerm(TRigidBody * body,
                                const typename MyMatrix<typename TRigidBody::PREC >::Vector3 & gravitiy)
    {
        //Mass Matrix
        body->m_MassMatrix_diag.template head<3>().setConstant(body->m_mass);
        body->m_MassMatrix_diag.template tail<3>() = body->m_K_Theta_S;

        // Massmatrix Inverse
        initMassMatrixInv<TRigidBody>(body);
//        // H_const term
//        body->m_h_term_const.template head<3>() =  (body)->m_mass * gravitiy;
    }


};


#endif // RigidBodyFunctions_hpp
