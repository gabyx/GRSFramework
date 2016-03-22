#ifndef TemplateDeduction_hpp
#define TemplateDeduction_hpp

template<typename A, typename B, typename C>
void foo( B & b, C & c){

    std::cout <<  b + c;
}


void test(){
    int a;
    double b;
    foo<float,int,double>(a,b);
}

#endif


