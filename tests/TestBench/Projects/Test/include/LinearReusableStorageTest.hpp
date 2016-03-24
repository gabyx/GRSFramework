// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef LinearReusableStorageTest_hpp
#define LinearReusableStorageTest_hpp

#include <typeinfo>
#include <vector>
#include <limits>
#include <iterator>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/DemangleTypes.hpp"
#include "GRSF/common/CPUTimer.hpp"
#include "GRSF/common/LinearReusableStorage.hpp"


DEFINE_LAYOUT_CONFIG_TYPES

struct A{
    A(){};
    A(float a){v.setConstant(a);};
    void foo(){};
    Eigen::Vector4f v;
    ~A(){
        //std::cout << "Destruct A" << std::endl;
    }
};



void linearReusableStorageTest() {
//    #ifdef NDEBUG
//        const int size = 1000000;
//    #else
//        const int size = 1000;
//    #endif
//    int loops = 500;
//    std::cout << " make vector " << std::endl;
//    {
//
//
//        std::vector<A*> vec;
//
//        START_TIMER(start)
//        A *arr = new A[size];
//        vec.reserve(size);
//
//        for(int i = 0; i<size;++i){
//            vec.push_back(arr+i);
//        }
//        STOP_TIMER_SEC(count,start);
//
//        std::cout << typeid(vec).name() <<" alloc time: "<< count << " sec. " <<  std::endl;
//        START_TIMER(start2)
//        for(int i=0;i<loops;i++){
//            for(auto * v: vec){
//                v->v += v->v;
//                v->v -= v->v;
//            }
//        }
//        STOP_TIMER_SEC(count2,start2);
//        std::cout << typeid(vec).name() <<" iter time: "<< count2 << " sec. " <<  std::endl;
//        delete[] arr;
//    }
//    {
//
//
//        StdVecAligned<A> vec;
//
//        START_TIMER(start)
//        vec.reserve(size);
//        for(int i=0;i<size;i++){ vec.emplace_back(i);}
//        STOP_TIMER_SEC(count,start);
//        std::cout << typeid(vec).name() <<" alloc time: "<< count << " sec. " <<  std::endl;
//
//        START_TIMER(start2)
//        for(int i=0;i<loops;i++){
//            for(auto & v:vec){
//                v.v += v.v;
//                v.v -= v.v;
//            }
//        }
//        STOP_TIMER_SEC(count2,start2);
//        std::cout << typeid(vec).name() <<" iter time: "<< count2 << " sec. " <<  std::endl;
//
//    }
//
//
//    {
//        LinearReusableStorage<A> vec;
//
//        START_TIMER(start)
//        vec.reserve(size);
//        for(int i=0;i<size;i++){ vec.emplace_back(i);}
//        STOP_TIMER_SEC(count,start);
//        std::cout << typeid(vec).name() <<" alloc time: "<< count << " sec. " <<  std::endl;
//
//        START_TIMER(start2)
//        for(int i=0;i<loops;i++){
//            for(A * v : vec){
//                v->v += v->v;
//                v->v -= v->v;
//            }
//        }
//        STOP_TIMER_SEC(count2,start2);
//        std::cout << typeid(vec).name() <<" iter time: "<< count2 << " sec. " <<  std::endl;
//        std::cout << vec.back()->v << std::endl;
//    }


    {


        LinearReusableStorage<double> vec;
        vec.clear();
        for(int i = 0;i<20;++i){
            vec.push_back(i);
        }


        for(auto it = vec.begin(); it != vec.end(); ++it){
            std::cout << *(*it) << ",";
        }

        vec.shuffleUniformly();
        for(auto v: vec)
            std::cout << *v << ", ";
        std::cout << std::endl << "Sorted: " ;
        std::sort(vec.begin(),vec.end(),[](double * a, double * b){return *a<*b;});
        for(auto v: vec)
            std::cout << *v << ", ";
        std::cout << std::endl;

        vec.resize(33);
        for(auto v: vec)
            std::cout << *v << ", ";
        std::cout << std::endl;

        vec.shuffleUniformly();
        for(auto v: vec)
            std::cout << *v << ", ";
        std::cout << std::endl;

        vec.resize(100);

        vec.shuffleUniformly();
        std::sort(vec.begin(),vec.end(),[](double * a, double * b){return *a<*b;});
        for(auto v: vec)
            std::cout << *v << ", ";
        std::cout << std::endl;
    }

    LinearReusableStorage<A> vec;
    vec.clear();
    for(int i = 0;i<20;++i){
        vec.push_back(i);
    }


    const auto & vecC = vec;
    auto it = vecC.begin();
    std::cout <<it->v(0)<< demangle::type(it) << std::endl;
    auto it2 = vec.cbegin();
    std::cout <<it2->v(0)<< demangle::type(it2) << std::endl;


}


#endif


