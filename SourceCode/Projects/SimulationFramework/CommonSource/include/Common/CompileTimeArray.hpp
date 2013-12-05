#ifndef CompileTimeArray_hpp
#define CompileTimeArray_hpp


// http://stackoverflow.com/questions/2978259/programmatically-create-static-arrays-at-compile-time-in-c
// Usage as: (Generator is a struct which defines a generator(size_t i) function for each index i
// It needs to be a constexpr! -> Only one line code is accepted!
// struct special
//{
//    static constexpr double generator(size_t index) {
//        return (index==1)? 3 :
//            ( (index==3)? 100 :
//                ((index==0)? 40 : 0)
//            );
//    }
//};
//    const unsigned int count = 10;
//    typedef CompileTimeArray::Array<count, special> A;
//
//    for (size_t i=0; i<count; ++i)
//        std::cout << A::values[i] << "\n";

namespace CompileTimeArray{
    namespace internal{

        // generate the intgeral tempalte sequence ================
        template <size_t...> struct indices {};

        template <size_t N, typename T> struct MakeIndices;

        template <size_t... Indices>
        struct MakeIndices<0, indices<Indices...> > {
            typedef indices<0, Indices...> type;
        };

        template <size_t N, size_t... Indices>
        struct MakeIndices<N, indices<Indices...> > {
            typedef typename MakeIndices<N-1, indices<N, Indices...> >::type type;
        };
        // =========================================================

        // Define template for ArrayHolder
        template <typename Gen, size_t N, typename T> struct ArrayHolder;

        //Sepcialization
        template <typename Gen, size_t N, size_t... Indices>
        struct ArrayHolder<Gen, N, indices<Indices...> >
        {
            static decltype(Gen::generate(size_t())) values[N];
        };

        //Static Initialization
        template <typename Gen, size_t N, size_t... Indices>
        decltype(Gen::generate(size_t()))  ArrayHolder<Gen, N, indices<Indices...> >::values[N] =  { Gen::generate(Indices)...};

    };

    template <typename Gen, size_t N>
    struct Array : internal::ArrayHolder<Gen, N, typename internal::MakeIndices<N-1,internal::indices<> >::type  >{};

};


#endif // CompileTimeArray_hpp
