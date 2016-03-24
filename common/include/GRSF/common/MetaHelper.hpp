#ifndef GRSF_common_MetaHelper_hpp
#define GRSF_common_MetaHelper_hpp

#include <meta/meta.hpp>

//namespace meta = meta::v1;

namespace metaAdd {

    namespace details {
        template<typename T, T N, T R, T C, T L, typename = std::integral_constant <T, L>  >
        struct toRowColSym_impl {

            static_assert(L < N*N, "linear offset needs to be smaller or equal max index N*N-1");

            using res =         typename std::conditional< (L >= N) ,
                  toRowColSym_impl<T, N-1,R+1,C+1,L-N>,
                  toRowColSym_impl<T, N,R,C+L,0>
                  >::type;
            static const T i = res::i;
            static const T j = res::j;

        };

        template<typename T, T N, T R, T C, T L>
        struct toRowColSym_impl<T, N, R, C, L, std::integral_constant <T,0> > {
            static const T i = R;
            static const T j = C;
        };
        template<typename T, T R, T C, T N>
        struct toLinearIdxSym_impl {
            // Switch indices if R,C in lower part of matrix
            static_assert(R < N && C < N, "row and col smaller then N");
            using type = meta::size_t< (R<=C)?            R*N + C - R*(R+1)/2     :         C*N + R - C*(C+1)/2 >;
        };
    };

    /// Get the row (::i) and column (::j) of the linear index \p L of the upper part of a symetric matrix of size \p N.
    template<std::size_t L, std::size_t N>
    using toRowColSym_c = typename details::toRowColSym_impl<std::size_t, N,0,0,L>;
    /// Get the row (::i) and column (::j) of the linear index \p L (meta::size_t) of the upper part of a symetric matrix of size \p N  (meta::size_t).
    template<typename L, typename N>
    using toRowColSym = typename details::toRowColSym_impl<std::size_t, N::type::value ,0,0,L::type::value>;

    /// Get the linear index of the element at row \p R and col \p C of the
    /// linear stored upper part of a symetric matrix of size \p N
    template<std::size_t R, std::size_t C, std::size_t N>
    using toLinearIdxSym_c = details::toLinearIdxSym_impl<std::size_t,R,C,N>;
    /// Get the linear index of the element at row \p R (meta::size_t) and col \p C (meta::size_t) of the
    /// linear stored upper part of a symetric matrix of size \p N (meta::size_t).
    template<typename R, typename C, typename N>
    using toLinearIdxSym = details::toLinearIdxSym_impl<std::size_t,R::type::value,C::type::value,N::type::value>;


// Supported by meta::find_index<T,List>
//    namespace details {
//
//        template<typename T, typename List>
//        struct getIdx_impl {
//            static const std::size_t i = meta::find< List, T >::size();
//            static_assert( i > 0, "Idx not found!");
//            using type = meta::size_t< List::size() - i >;
//        };
//    };
//    /// Return the index (std::integral_constant) for an element type \p T in a meta::list \p List.
//    template<typename T, typename List>
//    using getIdx = meta::eval<details::getIdx_impl<T,List>>;


    /// Return the type T* of the input type \p T.
    template<typename T>
    using addPointer = meta::eval<std::add_pointer<T> >;


    /// Returns a meta::list where all types equal to \p T in list \p List are removed.
    template<typename T, typename List>
    using remove = meta::filter< List, meta::compose<
                   meta::quote<meta::not_> ,
                   meta::bind_front< meta::quote<std::is_same>, T>
                   >>;




    namespace details {

        template<typename List, std::size_t I>
        struct find_ind_state {
            using L = List;
            static const std::size_t Idx = I;
        };

        template<typename Type>
        struct find_indices {
            template<typename State, typename T>
            using apply = find_ind_state<
                                              meta::if_< std::is_same<Type,T> ,
                                                          meta::push_back< typename State::L, meta::size_t<State::Idx> >,
                                                          typename State::L
                                              >,
                                              State::Idx+1
                                          >;
        };
    };

    /// Returns a meta::list of all indices of types equal to \p T in list \p List.
    template<typename T, typename List>
    using find_indices =  typename  meta::accumulate< List,
                                                      details::find_ind_state< meta::list<> , 0 >,
                                                      /* accumulate the index list and the current index of the element*/
                                                      details::find_indices<T>
                                                      >::L;

};

#endif
