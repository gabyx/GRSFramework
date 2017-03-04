// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef CompileTimeTable_hpp
#define CompileTimeTable_hpp

#include <cstddef>

// One methode
const int ARRAY_SIZE = 5;

template <int N, int I = N - 1>
class Table : public Table<N, I - 1>
{
public:
    static const int dummy;
};

template <int N>
class Table<N, 0>
{
public:
    static const int dummy;
    static int array[N];
};

template <int N, int I>
const int Table<N, I>::dummy = Table<N, 0>::array[I] = I* I + 0 * Table<N, I - 1>::dummy;

template <int N>
int Table<N, 0>::array[N];

template class Table<ARRAY_SIZE>;

// Second Method
// template<unsigned... args> struct ArrayHolder {
//    static const unsigned data[sizeof...(args)];
//};
//
// template<unsigned... args>
// const unsigned ArrayHolder<args...>::data[sizeof...(args)] = { args... };
//
// template<size_t N, template<size_t> class F, unsigned... args>
// struct generate_array_impl {
//    typedef typename generate_array_impl<N-1, F, F<N>::value, args...>::result result;
//};
//
// template<template<size_t> class F, unsigned... args>
// struct generate_array_impl<0, F, args...> {
//    typedef ArrayHolder<F<0>::value, args...> result;
//};
//
// template<size_t N, template<size_t> class F>
// struct generate_array {
//    typedef typename generate_array_impl<N-1, F>::result result;
//};
//
//

template <unsigned int index>
struct MetaFunc
{
    static const unsigned int value = index * 0.5;
};

template <>
struct MetaFunc<0>
{
    static const unsigned int value = 1;
};

// http://stackoverflow.com/questions/2978259/programmatically-create-static-arrays-at-compile-time-in-c
namespace CompileTimeArray
{
namespace internal
{
template <typename T, T... values>
struct ArrayHolder
{
    static T data[sizeof...(values)];
};

template <typename T, T... values>
T ArrayHolder<T, values...>::data[] = {values...};

template <unsigned int N, typename T, template <unsigned int> class Func, T... values>
struct generate_array_impl
{
    typedef typename generate_array_impl<N - 1, T, Func, Func<N - 1>::value, values...>::result result;
};

template <typename T, template <unsigned int> class Func, T... values>
struct generate_array_impl<0, T, Func, values...>
{
    typedef ArrayHolder<T, values...> result;
};
};

// Main function
template <typename T, unsigned int N, template <unsigned int> class Func>
struct generate_array
{
    typedef typename internal::generate_array_impl<N, T, Func>::result result;
};
};

namespace CompileTimeArray2
{
template <size_t...>
struct indices
{
};

template <size_t N, typename T>
struct MakeIndices;

template <size_t... Indices>
struct MakeIndices<0, indices<Indices...>>
{
    typedef indices<0, Indices...> type;
};

template <size_t N, size_t... Indices>
struct MakeIndices<N, indices<Indices...>>
{
    typedef typename MakeIndices<N - 1, indices<N, Indices...>>::type type;
};

template <typename Gen, size_t N, typename T>
struct ArrayHolder;

// Sepcialization
template <typename Gen, size_t N, size_t... Indices>
struct ArrayHolder<Gen, N, indices<Indices...>>
{
    static decltype(Gen::generate(size_t())) values[N];
};

// Static Initialization
template <typename Gen, size_t N, size_t... Indices>
decltype(Gen::generate(size_t())) ArrayHolder<Gen, N, indices<Indices...>>::values[N] = {Gen::generate(Indices)...};

template <typename Gen, size_t N>
struct Array : ArrayHolder<Gen, N, typename MakeIndices<N - 1, indices<>>::type>
{
};
};

struct init_sin
{
    static constexpr unsigned int generate(size_t index)
    {
        // return 3*std::sin(index * 2.0 * 3.1415 / 20.0);
        return index;
    }
};

struct special
{
    static constexpr unsigned int generate(size_t index)
    {
        return (index == 1) ? 3 : ((index == 3) ? 100 : ((index == 0) ? 40 : 0));
    }
};
namespace mySpecialTable
{
struct special2
{
    static constexpr bool generate(size_t index)
    {
        return (index == 1) ? 1 : ((index == 3) ? 0 : ((index == 0) ? 1 : 0));
    }
};

typedef CompileTimeArray2::Array<special2, 4> Array;
};

void printCompileTimeTable()
{
    std::cout << "Method 1" << std::endl;
    const int* compilerFilledArray = Table<ARRAY_SIZE>::array;
    for (int i = 0; i < ARRAY_SIZE; ++i)
        std::cout << compilerFilledArray[i] << std::endl;

    std::cout << "Method 2" << std::endl;
    const unsigned int count = 10;
    typedef CompileTimeArray::generate_array<unsigned int, count, MetaFunc>::result A;

    for (size_t i = 0; i < count; ++i)
        std::cout << A::data[i] << "\n";

    std::cout << "Method 3" << std::endl;
    typedef CompileTimeArray2::Array<init_sin, 4> Array;
    for (int i = 0; i != 4; ++i)
    {
        std::cout << Array::values[i] << std::endl;
    }

    std::cout << "Method 4" << std::endl;
    typedef CompileTimeArray2::Array<special, 4> Array2;
    std::cout << "pointer:" << Array2::values << std::endl;

    for (int i = 0; i != 4; ++i)
    {
        std::cout << Array2::values[i] << std::endl;
        Array2::values[i] = i;
    }

    std::cout << "Method 4 B" << std::endl;
    typedef CompileTimeArray2::Array<special, 4> ArrayGAGA;
    for (int i = 0; i != 4; ++i)
    {
        std::cout << ArrayGAGA::values[i] << std::endl;
    }

    std::cout << "Method 5" << std::endl;
    typedef mySpecialTable::Array Array3;
    for (int i = 0; i != 4; ++i)
    {
        std::cout << Array3::values[i] << std::endl;
    }
}

#endif  // CompileTimeTable_hpp
