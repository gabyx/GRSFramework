// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <Eigen/Dense>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

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
using namespace boost::math;

template <typename T>
void func()
{
#define AA (T::A)
    cout << AA << endl;
}

#define TABLESIZE 1024
#define BUFSIZE TABLESIZE
#define AFTERX(x) 4 + x
#define XAFTERX(x) 4 + x

template <int Id>
struct A
{
    static void foo_s()
    {
        foo<Id>();
    }

    template <int Id>
    static void   foo();

    template <>
    static void foo<1>()
    {
        cout << "wuaaaa 1" << endl;
    }

    template <>
    static void foo<2>()
    {
        cout << "wuaaaa 2" << endl;
    }
};

template <typename _PREC, int _VariantId, int _alignMatrix, bool _bAbortIfConverged, bool _nCheckConvergedFlag>
struct JorProxGPUVariantSettingsWrapper
{
    typedef typename _PREC PREC;
    static const int       VariantId           = _VariantId;
    static const int       alignMatrix         = _alignMatrix;
    static const bool      bAbortIfConverged   = _bAbortIfConverged;
    static const bool      nCheckConvergedFlag = _nCheckConvergedFlag;
};

template <typename T>
struct JorProxGPUVariant
{
};

template <int _VariantId, bool _alignMatrix>
struct JorProxGPUVariantSettings
{
    template <typename _PREC, bool _bAbortIfConverged, bool _nCheckConvergedFlag>  // Settings from above
    struct JorProxGPUVariantType
    {
        typedef typename JorProxGPUVariant<
            JorProxGPUVariantSettingsWrapper<_PREC, _VariantId, _alignMatrix, _bAbortIfConverged, _nCheckConvergedFlag>>
            TGPUVariant;
    };
};

template <typename Ref, int N1>
struct RunImpl;

template <typename Ref>
struct RunImpl<Ref, 2>
{
    RunImpl(Ref* ptr)
    {
        m_pRef = ptr;
    }

    template <typename T>
    void run(int& a, int& b, T& c)
    {
        a = b + c + m_pRef->m;
    }

    Ref* m_pRef;
};

template <typename Ref>
struct RunImpl<Ref, 1>
{
    RunImpl(Ref* ptr)
    {
        m_pRef = ptr;
    }

    template <typename T>
    void run(int& a, T& b)
    {
        a = a + b + m;
    }

    Ref* m_pRef;
};

template <int _nArgs>
class Variant
{
public:
    static const int nArgs = _nArgs;

    Variant() : m_RunImpl(this)
    {
        m = 3;
    };

    typename RunImpl<Variant<nArgs>, nArgs> m_RunImpl;

    int m;
};

template <typename Ref, typename Ref2>
class RunDispatchBase
{
public:
    RunDispatchBase(Ref* ptr, Ref2* ptr2)
    {
        m_pRef  = ptr;
        m_pRef2 = ptr2;
    }

    Ref*  m_pRef;
    Ref2* m_pRef2;
};

template <typename Ref, typename Ref2, int N2>
struct RunDispatch;
template <typename Ref, typename Ref2>
class RunDispatch<Ref, Ref2, 2> : public RunDispatchBase<Ref, Ref2>
{
public:
    RunDispatch(Ref* ptr, Ref2* ptr2) : RunDispatchBase(ptr, ptr2)
    {
    }

    void runDispatch()
    {
        m_pRef2->m_RunImpl.run(m_pRef->m_a, m_pRef->m_b, m_pRef->m_c);
    }
};

template <typename TVariant>
struct TestVariant
{
    static const int nArgs = TVariant::nArgs;  // say this parameter comes from outside
    // say we want to call a run function depending on this argument, if for example nArgs = 3, (ConvexSet =
    // RplusAndDisk) then we give some different arguments
    // then if nArgs = 4 (ConvexSet = RplusAndContensouEllipsoid). The Test Class provides all parameters, which are set
    // up and initialized, the underlying Variant has only the correct RunImpl which needs
    // some number of args...

    TestVariant() : m_runDispatch(this, &m_variant)
    {
    }

    void run()
    {
        m_runDispatch.runDispatch();
    }

    RunDispatch<TestVariant<TVariant>, TVariant, nArgs> m_runDispatch;

    TVariant m_variant;

    int m_a, m_b, m_c;
};

template <int _A>
struct G
{
    const static int A = _A;
};

template <typename Derived>
void func2(const MatrixBase<Derived>& a)
{
    cout << typeid(Derived).name() << endl;
    cout << typeid(Derived::StorageKind).name() << endl;
    int b = (Derived::Flags & RowMajorBit);
    cout << a.outerStride() << endl;
    Derived::Scalar* c = const_cast<Derived::Scalar*>(&a.operator()(0, 0));
    cout << "Storage:" << c[2] << endl;

    // const Derived::Scalar* d = a.data();
}

int main()
{
    // typedef JorProxGPUVariantSettings<3,true>::JorProxGPUVariantType<double,true,true>::TGPUVariant TGPUVariant;

    // A<2>::foo_s();

    TestVariant<Variant<2>> TV;
    TV.run();

    // cout <<"float representation of number in Binary form"<<endl;

    // int num = 0;
    // float fnum;
    // float a= 1;
    // float b = 0;
    // fnum = a/b; //std::numeric_limits<float>::infinity();
    // if(!isnormal(fnum)){
    // cout << "WUAAA"<<endl;
    //}

    // printf("INFINITIY: %i    , %x  \n ", *(int*)&fnum, *(int*)&fnum);

    //// Print some values
    // for(int i=-50;i<=50;i++){
    // printf("float: %0.13f    \t int:  %i  \n ", *(float*)&i, i);

    //}

    // func<G<3> >();

    // typedef double PREC;
    // Eigen::Matrix<PREC,Eigen::Dynamic, Eigen::Dynamic,Eigen::ColMajor> T_internal(3+2,3); // We save row-major which
    // is T' in col-major interpretation which makes cuBlas faster!
    // T_internal.setRandom();
    // //Eigen::Matrix<PREC,Eigen::Dynamic, Eigen::Dynamic>  & T =  T_internal.block(0,0,3,3); // Copied! Stupidloo
    // Eigen::Block< Eigen::Matrix<PREC,Eigen::Dynamic, Eigen::Dynamic,Eigen::ColMajor> ,Eigen::Dynamic, Eigen::Dynamic>
    // T = T_internal.block(0,0,3,3);
    // T(0,0)=1000;
    // func2(T);
    // func2(T_internal);
    // func2(Eigen::Vector2d());
    // cout << T_internal <<endl<<endl;
    // cout << T.outerStride() <<endl;
    // cout << T.rows() <<endl;

    // cout << XAFTERX(TABLESIZE) <<endl;
    // //cout << int(Eigen::Matrix<PREC,Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>::Flags &
    // Eigen::RowMajorBit)<<endl;

    system("pause");
};
