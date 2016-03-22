#include <iostream>

#include "Eigen/Dense"
#include "Eigen/StdVector"

#include "TypeDefs.hpp"
#include "RigidBody.hpp"


#include <boost/shared_ptr.hpp>

using namespace std;

class A{
public:
    A(){
        cout << this<<"::CTOR"<<endl;
    }
    explicit A(int a){
        cout << this<<"::CTOR(int)"<<endl;
    }
    A(const A & a){
        cout << this<<"::COPYConstructor"<<endl;
    }
    A & operator=(const A & a){
        cout << this<<"::Assign"<<endl;
    }

};


int testRigidBody(){
//
    RigidBodyBase<MyRigidBodyConfig> b;

    b.m_State.get_r_S() = MyMatrix<double>::Vector3::Ones()*2;
    b.m_State.setQuaternion(MyMatrix<double>::Quaternion::Ones());
    cout << b.m_State.getQuaternion() << std::endl;
    cout << b.m_State.get_omega_IK() << std::endl;
    cout << b.m_State.get_v_S() << std::endl;
    cout << "q: "<<b.m_State.m_q << std::endl;

    RigidBodyStateQuaternion<MyRigidBodyConfig::LayoutConfigType> a;
    a.getQuaternion();

    createNewSolverDataType(&b);
    b.m_pSolverData->m_uBuffer.getBack().setOnes();
    cout << b.m_pSolverData->m_uBuffer.getBack() << std::endl;
    cout << b.m_State.m_u << std::endl;

    //RigidBodyStateQuaternion<LayoutConfig< double, DynamicLayout<6,6> >> aa;
//     aa.getQuaternion(); // Does not and should not work!

}
