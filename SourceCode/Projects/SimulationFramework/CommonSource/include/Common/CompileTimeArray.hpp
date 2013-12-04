#ifndef CompileTimeArray_hpp
#define CompileTimeArray_hpp


// http://stackoverflow.com/questions/2978259/programmatically-create-static-arrays-at-compile-time-in-c
// Usage as: (MetaFunc is a class which defines a MetaFunc<index>::value for each index
//    template<unsigned int index> struct MetaFunc {
//         enum { value = index };
//    };
//    const unsigned int count = 10;
//    typedef CompileTimeArray::generate_array<count, MetaFunc>::result A;
//
//    for (size_t i=0; i<count; ++i)
//        std::cout << A::data[i] << "\n";

namespace CompileTimeArray{
    namespace internal{

        template<unsigned int... values>
        struct ArrayHolder{
            static unsigned int data[sizeof...(values)];
        };
        // static initializer  with variadic template arguments
        template<unsigned int... values>
        unsigned int ArrayHolder<values... >::data[] = { values... };

        template<unsigned int N, template<unsigned int> class Func, unsigned int... values>
        struct generateArray_impl{
            typedef typename generateArray_impl<
                                        N-1,
                                        Func,
                                        Func<N-1>::value, values...
                                    >::result result;
        };

        template<template<unsigned int> class Func, unsigned int... values>
        struct generateArray_impl<0,Func, values...>{
            typedef ArrayHolder<values...> result;
        };
    };


    //Main function
    template<unsigned int N, template<unsigned int> class Func>
    struct generateArray{
        typedef typename internal::generateArray_impl<N,Func>::result result;
    };
};

#endif // CompileTimeArray_hpp
