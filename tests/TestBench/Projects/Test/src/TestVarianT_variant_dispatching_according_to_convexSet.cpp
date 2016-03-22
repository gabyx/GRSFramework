#include <iostream>
#include <string>
#include <limits>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <cmath>
#include <Eigen/Dense>

#include <boost/any.hpp>

using namespace std;

/*
 *  TypeDefs.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
*/

#include <Eigen/Dense>
// CONFIG Template parameter which is used all over the framework!=========================================================

using namespace Eigen;
using namespace boost::math;


template<typename Ref, int N1> struct RunImpl;
   // Those are the different imlpementation for  ConvexSet 2   
template<typename Ref> struct RunImpl<Ref,2>{

   RunImpl(Ref * ptr){
      m_pRef = ptr;
   }

   template<typename T>
   void run(int &a, int &b , T &c){
      a = b + c + m_pRef->m;
   }

   Ref * m_pRef;

};

// Those are the different imlpementation for  ConvexSet 1
template<typename Ref> struct RunImpl<Ref,1>{

   RunImpl(Ref * ptr){
      m_pRef = ptr;
   }

   template<typename T>
   void run(int &a,  T &b){
      a= a + b+ m;
   }

   Ref * m_pRef;

};

/// BETTER SPECIALIZE THE WHOLE CLASS ON THE CONVEX TYPE!
template<int _nConvexSet>
class Variant{
public:
      static const int nConvexSet = _nConvexSet;


      Variant(): m_RunImpl(this){
         m = 3;
      };

      typename RunImpl<Variant<nConvexSet>,nConvexSet> m_RunImpl;

      int m;

};


template<typename Ref, typename Ref2> class RunDispatchBase{
public:
    RunDispatchBase(Ref * ptr, Ref2 *ptr2){
      m_pRef = ptr;
      m_pRef2 = ptr2;
   }

    
   Ref * m_pRef;
   Ref2 * m_pRef2;

};

template<typename Ref,typename Ref2, int N2> struct RunDispatch;
template<typename Ref,typename Ref2> class RunDispatch<Ref, Ref2, 2> : public RunDispatchBase<Ref,Ref2>{

public:
   RunDispatch(Ref* ptr, Ref2 *ptr2): RunDispatchBase(ptr,ptr2){}

   void runDispatch(){
      m_pRef2->m_RunImpl.run(m_pRef->m_a,m_pRef->m_b,m_pRef->m_c);
   }

};


template< typename TVariant>
struct ProxTestVariant{

   static const int nConvexSet = TVariant::nConvexSet; // say this parameter comes from outside
   // say we want to call a run function depending on this argument, if for example nArgs = 3, (ConvexSet = RplusAndDisk) then we give some different arguments
   //then if nArgs = 4 (ConvexSet = RplusAndContensouEllipsoid). The Test Class provides all parameters, which are set up and initialized, the underlying Variant has only the correct RunImpl which needs
   // some number of args...

  ProxTestVariant():  m_runDispatch(this,&m_variant){}

   void run(){
     m_runDispatch.runDispatch();
   }
   
   RunDispatch<ProxTestVariant<TVariant>,TVariant,nConvexSet> m_runDispatch;

   TVariant m_variant;
   
   int m_a, m_b, m_c;

};


struct FalseType { static const bool  value = false ; };
struct TrueType {  static const bool  value = true ; };


template <typename T1, typename T2>
struct IsSame
{
  typedef ::FalseType Result;
  static const bool result = false;
};


template <typename T>
struct IsSame<T,T>
{
TrueType Result;
static const bool result = true;
};

namespace OtherType{
   struct Type1{};
}

template< typename _T> // Settings from below
struct Settings{
   typedef _T myT;
   //typedef char static_assert_failed[ ((IsSame< myT,OtherType::Type1>::Result::value)) ? 1 : -1 ];
};

struct FOFO{
   static const int a = 3;
};
int main(){
  
  Eigen::Matrix<double,6,6> a;
  a.setRandom();
  a.diagonal().setConstant(2);
  cout << a <<endl;
  cout << (a.array().abs().matrix().rowwise().sum() - a.diagonal().array().abs().matrix()) <<endl;
  cout << (a.diagonal().array().abs() < (a.array().abs().matrix().rowwise().sum() - a.diagonal().array().abs().matrix()).array() ) <<endl;

  cout << "SUM: " << (a.diagonal().array().abs() < (a.array().abs().matrix().rowwise().sum() - a.diagonal().array().abs().matrix()).array()).count() <<endl;
   //Settings<OtherType::Type1> a;

   //cout << (IsSame<OtherType::Type1,OtherType::Type1>::Result::value)<< endl;

   //Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> * pG =  new Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>(3,4);
   //(*pG).setRandom(); delete pG;
   //cout << (*pG).block(0,0,2,2) <<endl;
   
   //ProxTestVariant<Variant<2>> TV;
   //TV.run();

  system("pause");
};
