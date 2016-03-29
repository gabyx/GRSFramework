// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <iostream>

#include "Eigen/Dense"
#include "Eigen/StdVector"

using namespace Eigen;
using namespace std;


// template von LayoutConfig
class RigidBodyStateBase{
	public:
	Eigen::Matrix<double,6,1> u;
	Eigen::Matrix<double,7,1> q;

	const Eigen::Matrix<double,6,1> & getU(){
		return u;
	};
	const Eigen::Matrix<double,7,1> & getQ(){
		return q;
	};
};


//Prototype
//template< LayoutConfig> class RigidBodyState ;


struct MyMatrix {
	typedef Eigen::Matrix<double,7,1> VectorQObj;
	typedef Eigen::Matrix<double,6,1> VectorUObj;
	typedef Eigen::Matrix<double,4,1> Quaternion;
};


//Spezialisatzion für Quaternion, Pos, Vel und Ang Velo
//template<PREC>
class RigidBodyState/*<DynamicLayoutConfig<PREC,7,6> >*/ : public RigidBodyStateBase/*<DynamicLayoutConfig<PREC,7,6> > */{
public:
	Matrix<double,3,3> A_IK;


	MatrixBlock<MyMatrix::VectorQObj,4,1>::const_type  getQuaternion() const {
		return MatrixBlock<MyMatrix::VectorQObj,4,1>::const_type(q,0,0);
	}


	void setQuaternion(const MyMatrix::Quaternion & quat){
		getQuaternionRef() = quat;
		// Calculate A_IK from Quaternion;
	}
	void setQuaternionToUnit(){
		getQuaternionRef() = MyMatrix::Quaternion::Identity();
		A_IK.setIdentity();
	}
	void normalizeQuaternion(){
		getQuaternionRef().normalize();
	}

	MatrixBlock<MyMatrix::VectorQObj,3,1>::type  get_r_S(){
		return q.head<3>();
	}

	MatrixBlock<MyMatrix::VectorUObj,3,1>::type  get_v_S(){
		return u.head<3>();
	}

	MatrixBlock<MyMatrix::VectorUObj,3,1>::type   get_omega_IK(){
		return u.tail<3>();
	}



private:

    //Make also const as it should not be changed from outside!
    MatrixBlock<MyMatrix::VectorQObj,4,1>::type  getQuaternionRef() {
		return MatrixBlock<MyMatrix::VectorQObj,4,1>::type(q,3,0);
	}


};

class SolverData{
    public:

	//MAtric zeugs, Masse, h term usw...

	SolverData(RigidBodyState &state): uBack(state.u){
	}

	void swap(){
		(uBack).swap(uFront); //Internal pointer swap!

	}

	//Actual Buffer
	Eigen::Matrix<double,6,1> &uBack;
	Eigen::Matrix<double,6,1> uFront;

private:

};

void frissQuaternion(const Matrix<double,4,1> & quat){
	return;
};

class RigidBody {
	public:
	RigidBody(): solvData(state){
	};
	~RigidBody(){
	};

	// Alle anderen Sachen!

	//Geometry
	//Material

	RigidBodyState state;
	SolverData solvData;
};

//class AA{
//public:
//    AA(const int & a):b(a){};
//    int & b;
//};


template<typename T1, typename T2=T1>
	void foo(T1 t, T2 a){
}

#include "FrontBackBuffer.hpp"

int testRigidBody() {
    RigidBody r;
	r.state.q.setRandom();
	r.state.u.setRandom();

	cout << "q: "<< r.state.q.transpose() << endl;
	cout << "Pointer:" << &r.state.q(0) << endl;
	cout << "AFTER:::" << endl;
	Matrix<double,4,1> quat = r.state.getQuaternion()+r.state.getQuaternion();
	//r.state.getQuaternion().setZero();
	const RigidBodyState sta;
	sta.getQuaternion();

    //VectorBlock< Matrix<double,7,1> ,4>  block =	r.state.getQuaternion();
//	VectorBlock< Matrix<double,7,1> ,4>  block1 = block;
//	block1.setOnes();
	cout << "q: "<< r.state.q.transpose()  << endl;
//	cout << "quat:" << quat << endl;
//	cout << "Pointer:" << &quat(0) << endl;
//
//	cout << "q: " << r.state.q.transpose()  << endl;
	//cout << 	Matrix<double,4,1>::RowSize <<endl;

	cout << "u: "<< r.state.u.transpose()  << endl;
	r.solvData.uBack.setOnes().array() *=1;
	cout << "u: "<< r.state.u.transpose()  << endl;

	// Test swap
	r.solvData.uFront.setOnes().array() *= 2;
	cout << "u: "<< r.state.u.transpose()  << endl;
	r.solvData.swap();
	cout << "After swap u: "<< r.state.u.transpose()  << endl;
	cout << "u: "<< r.solvData.uBack.transpose()  << endl;
	cout << "u: "<< r.solvData.uFront.transpose()  << endl;

    r.state.get_r_S() = Vector3d::Ones()*3;
    cout << "q.r: "<< r.state.get_r_S() << endl<<endl;

    int b;
    int a;
    FrontBackBuffer<int&> buffer(a,b);
    buffer.m_Back = 33;
    buffer.m_Front = 55;
    cout << a <<","<<b << endl;

    int c;
    const int & d = c;
    //int & f= d;
    //AA aaa(a)
}
