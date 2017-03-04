// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <cmath>
#include <iostream>
#include <string>

#define EIGEN_DONT_VECTORIZE
#include <Eigen/Dense>

using namespace std;

/*
 *  TypeDefs.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
*/

#include <Eigen/Dense>
// CONFIG Template parameter which is used all over the
// framework!=========================================================
using namespace Eigen;

template <typename Derived, typename DerivedOther>
void foo(const MatrixBase<Derived>& a, const MatrixBase<DerivedOther>& b)
{
    Eigen::MatrixBase<Derived>& a_ref      = const_cast<Eigen::MatrixBase<Derived>&>(a);
    Eigen::MatrixBase<DerivedOther>& b_ref = const_cast<Eigen::MatrixBase<DerivedOther>&>(b);

    a_ref(0, 0) = 5;

    cout << "foo:: a" << a << endl;
    cout << "foo:: b" << b << endl;  // b is evaluated first here! which gives wrong result because
}

template <typename PREC, typename Derived>
void doProxSingle(const PREC& radius, const Eigen::MatrixBase<Derived>& y)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    Eigen::MatrixBase<Derived>& y_ref = const_cast<Eigen::MatrixBase<Derived>&>(y);

    //// Solve the set (disc with radius mu_P_N), one r is used for the prox!
    PREC absvalue;
    // Prox normal
    using std::max;
    y_ref(0) = max(y_ref(0), 0.0);
    // Prox tangential
    absvalue = y_ref.segment<2>(1).norm();
    if (absvalue > radius * y_ref(0))
    {
        y_ref.segment<2>(1) = y_ref.segment<2>(1) / absvalue * radius * y_ref(0);
    }
}

template <typename T>
class A
{
};

template <int N>
struct rec
{
    typedef A<typename rec<N - 1>::type> type;
};

template <>
struct rec<0>
{
    typedef A<int> type;
};

enum
{
    DD,
    DH
};

template <int K>
void gagafunction()
{
    if (K == DD)
    {
        cout << " DD" << endl;
    }

    if (K == DH)
    {
        cout << " DH" << endl;
    }
}

int main()
{
    /*gagafunction<DH>();


   VectorXd a(6);
   a.setConstant(4);
   VectorXd b(6);
   b.setConstant(2);
*/
    //// ALIAS
    // foo(a.head<4>(), b.head<4>() + a.head<4>());
    // cout << " a"<< a <<endl;
    // cout << " b"<< b <<endl;

    // NO ALIAS
    /*foo(a.head<4>(), (b.head<4>() + a.head<4>()).eval());
   cout << " a"<< a <<endl;
   cout << " b"<< b <<endl;*/

    // Eigen::VectorBlock<Eigen::Matrix<double,-1,1>,3> &reff = a.segment<3>(0);
    ////reff += 4;
    // reff = Vector3d(0,1,3);
    // cout << reff<<endl;
    // cout << a<<endl;

    // Matrix<double, 12,12> G;
    // G.setConstant(1);
    // Matrix<double,12,1> P;
    // P.setConstant(3);

    // P.segment<10>(0).noalias() = (P.head<6>(0) + P.segment<6>(2)); //+ G.block(0,0,10,12)*P);
    // cout <<"With Aliasing:" << P <<endl;

    // P.setConstant(3);
    // P.segment<10>(0).noalias() = (P.segment<10>(0) + G.block(0,0,10,12)*P);
    // cout <<"With out Aliasing:" << P <<endl;

    //

    /*Matrix<double, 8,8> K1;
   Matrix<double, 8,8> K2;

   K1.setRandom();
   K2.setRandom();

   Matrix<double, 8,8> K3;
   Matrix<double, 8,8> K4;
   K3 = K1;
   K4 = K2;
   cout <<"K3: "<<endl<< K3<<endl;
   cout <<"K4: "<<endl<< K4<<endl;
   Matrix<double, 8,8> result = K3*K4;
   cout <<"Correct result Mult: "<<endl<< result<<endl;

   K3 = K3*K4;
   cout <<"Eigen, evaluates into temporary still correct: "<<endl<< K3 <<endl;

   K3 = K1;
   K4 = K2;
   cout <<"K3: "<<endl<< K3<<endl;
   cout <<"K4: "<<endl<< K4<<endl;
   K4.noalias() = K3*K4;
   cout <<"Eigen, evaluates not into temporary, should be aliased (very bad): "<<endl<< K4 <<endl;*/

    /*Matrix<double,4,1> P;
   P.setConstant(2);

   P.segment<3>(0).noalias() = P.segment<3>(0) / P(0);
   cout << P <<endl;

*/

    Matrix<double, 10, 10> c;
    c.setRandom();
    c = c.triangularView<Eigen::Upper>();
    cout << c << endl;

    Vector3d a, b;
    a.setRandom();
    b.setRandom();

    cout << a << endl;
    cout << b << endl;
    a.swap(b);

    cout << a << endl;
    cout << b << endl;

    system("pause");
};