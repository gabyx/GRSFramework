#ifndef GRSF_common_CompileTimeArray_hpp
#define GRSF_common_CompileTimeArray_hpp

#include <type_traits>
/**
* @brief http://stackoverflow.com/questions/2978259/programmatically-create-static-arrays-at-compile-time-in-c
* Usage as: (Generator is a struct which defines a  " generator(size_t i) " function for each index i
* It needs to be a constexpr! -> Only one line code is accepted!
* Take care: using A = CompileTimeArray::Array<count, GenA>;  and  using B = CompileTimeArray::Array<count, GenA>; somewhere else
* uses the same underlying static array!  Distinguish the arrays by using two different generator structs!
*
*    struct specialArray
*    {
*       static constexpr double generate(size_t index) {
*           return (index==1)? 3 :
*               ( (index==3)? 100 :
*                   ((index==0)? 40 : 0)
*               );
*       }
*    };
* 
*  struct linearRange : CompileTimeArray::RangeGenerator<0>{};
* 
*    const unsigned int count = 10;
*    using A = CompileTimeArray::Array<count, specialArray>;
*	 using B = CompileTimeArray::Array<count, linearRange>;
* 
*    for (size_t i=0; i<count; ++i)
*        std::cout << A::values[i] << "\n";
* 
*    for (size_t i=0; i<count; ++i)
*        std::cout << B::values[i] << "\n";
* 
*/
namespace CompileTimeArray{
    namespace internal{

        // generate the intgeral template sequence ================
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

        //Specialization
        template <typename Gen, size_t N, size_t... Indices>
        struct ArrayHolder<Gen, N, indices<Indices...> >
        {
            static decltype(Gen::generate(size_t())) values[N];
        };

        //Static Initialization
        template <typename Gen, size_t N, size_t... Indices>
        decltype(Gen::generate(size_t()))  ArrayHolder<Gen, N, indices<Indices...> >::values[N] =  { Gen::generate(Indices)...};


    };


	/**
	* @brief  Generator for linear range array, derive from these struct to make a new unique static range!
	*/
	template<size_t START = 0>
	struct RangeGenerator{
		static constexpr unsigned int generate(size_t index) {
			return index + START;
		}
	};

	namespace internal{
	   // Helper to dissallow certain instantiations of Array!
        template <typename T>
		struct is_allowed_instantiation { static const bool value = true; };
        template <size_t S>
		struct is_allowed_instantiation< CompileTimeArray::RangeGenerator<S> > { static const bool value = false; };

    };


	/**
	* @brief The exposed static compile-time array
	*/
	template <typename Gen, size_t N>
    struct Array : internal::ArrayHolder< Gen,
										  N,
										  typename internal::MakeIndices<N-1,internal::indices<> >::type
										 >{
												// Dissallow instatiation with certain generators:
												static_assert( internal::is_allowed_instantiation<Gen>::value, "This instantiation is not allowed!" );
										};


};


#endif // CompileTimeArray_hpp
