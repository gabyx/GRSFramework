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

using namespace std;

#include "StaticAssert.hpp"

//class A{
//
//    public:
//    template<>
//    A& operator << (const int & v){
//        cout << v<< endl;
//        return *this;
//    };
//
//private:
//
//    template<typename T>
//    A& operator << (const T & v){
//        cout << v<< endl;
//        return *this;
//    };
//
//};

template<int N>
struct AA{
   static const int Z = N;
   STATIC_ASSERT(Z % 2 == 0);
};

template<int N>
struct BB{
   static const int Z = N;
   STATIC_ASSERT(Z % 3 == 0);
};
typedef AA<2> GG;
typedef BB<3> TT;


struct Default{};

template<bool B, typename _M, typename _D>
struct SettingsManualDefault;


template<typename _M, typename _D>
struct SettingsManualDefault<true,_M,_D>{
   typedef typename _M TValue; 
};

template<typename _D>
struct SettingsManualDefault<true,Default,_D>{
    typedef typename _D TValue; 
};

template<typename _M, typename _D>
struct SettingsManualDefault<false,_M,_D>{
    typedef typename _D TValue; 
};
template<typename _D>
struct SettingsManualDefault<false,Default,_D>{
    typedef typename _D TValue; 
};

template<int N,int M>
struct IsEqual{
   static const bool result=false;
};

template<int M>
struct IsEqual<M,M>{
   static const bool result=true;
};


class Test2{
protected:

protected:
  int a;
  void foo();

};

class Test3: public Test2{
  void foo(){
  };

  void start(){
    a;
    &Test2::foo;
  }

};

int main()
{



   /*A a;
   a << 4;*/
   TT z;
   GG u;

   SettingsManualDefault<IsEqual<1,1>::result,Default,int>::TValue a=3.4;
   cout <<typeid(a).name() <<endl;
   SettingsManualDefault<IsEqual<1,2>::result,char,double>::TValue b=34;
   cout <<typeid(b).name() <<endl;
   system("pause");

}
