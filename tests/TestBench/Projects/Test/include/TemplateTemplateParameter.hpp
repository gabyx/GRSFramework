#ifndef TemplateTemplateParameters_hpp
#define TemplateTemplateParameters_hpp

#include <vector>
#include <iostream>

template<typename T>
class A{
public:

    template< template<typename,typename> class U, typename  Allocator>
    void foo( U<T,Allocator> & list){
        typename U<T,Allocator>::iterator it;
        for(it = list.begin();it != list.end(); it++){
            std::cout << *it <<std::endl;
        }
    };


};

void templateTemplateParamsTest(){

    A<int> a;

    std::vector<int> b;

    a.foo< std::vector >(b);

};


#endif
