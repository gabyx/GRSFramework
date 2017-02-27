// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef rangeTest1_hpp
#define rangeTest1_hpp

#include <cassert>
#include <chrono>
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>

#include <boost/variant.hpp>

namespace Test1
{
struct get_pointer : boost::static_visitor<void*>
{
    template <class T>
    void* operator()(T& element) const
    {
        return &element;
    }
};

template <typename T>
class Range
{
public:
    template <typename TYPE, typename TYPE2 = TYPE>
    struct ListOrRangeTypes
    {
        using ListType                         = std::set<TYPE>;  ///< ordered list!
        static const unsigned int ListTypeIdx  = 0;
        using RangeType                        = std::pair<TYPE2, TYPE2>;  ///< range start to end values
        static const unsigned int RangeTypeIdx = 1;
        using VariantType = boost::variant<ListType, RangeType>;  // Indexes into the variants are defined above!
    };

    typedef ListOrRangeTypes<T>                   ListOrRangeType;
    typedef typename ListOrRangeType::VariantType VariantType;

    Range(typename ListOrRangeType::ListType& l) : m_rangeVariant(l){};
    Range(typename ListOrRangeType::RangeType& r) : m_rangeVariant(r){};
    //    T diffLast() const{
    //        return 0;
    //    }
    //
    //    void setDiffValue(T v){
    //        m_diffValue = v;
    //    }
    //
    //    T getCurrentValue() const{
    //
    //    }

    template <typename T1>
    class iterator
    {
    public:
        iterator(VariantType& var, bool end = false)
        {
            m_atEnd    = end;
            m_listPtr  = nullptr;
            m_rangePtr = nullptr;
            m_currVal  = T1();

            get_pointer vis;
            void*       ptr = var.apply_visitor(vis);
            switch (var.which())
            {
                case (ListOrRangeType::ListTypeIdx):
                {
                    m_listPtr = static_cast<typename ListOrRangeType::ListType*>(ptr);
                    if (!m_atEnd)
                    {
                        m_listIt = m_listPtr->begin();
                    }
                    else
                    {
                        m_listIt = m_listPtr->end();
                    }
                    break;
                }
                case (ListOrRangeType::RangeTypeIdx):
                {
                    m_rangePtr = static_cast<typename ListOrRangeType::RangeType*>(ptr);
                    if (!m_atEnd)
                    {
                        m_currVal = m_rangePtr->first;
                    }
                    else
                    {
                        m_currVal = m_rangePtr->second;
                    }
                }
            }
        };

        // Standart copy constructor

        // Standart assignment constructor

        /** pre-increment ++it
        * Allow to iterate over the end of the sequence
        */
        iterator& operator++()
        {
            if (m_listPtr != nullptr)
            {
                if (++m_listIt == m_listPtr->end())
                {
                    m_atEnd = true;
                }
            }
            else if (m_rangePtr != nullptr)
            {
                if (++m_currVal > m_rangePtr->second)
                {
                    m_atEnd = true;
                }
            }
            return *this;
        }
        /** post-increment it++
        *
        */
        iterator operator++(int)
        {
            iterator it(*this);
            operator++();
            return it;
        }

        bool operator==(const iterator& rhs)
        {
            if (m_listPtr)
            {
                if (rhs.m_listPtr == m_listPtr)
                {  // if sequences are not the same
                    return m_listIt == rhs.m_listIt;
                }
            }
            else if (m_rangePtr)
            {
                if (rhs.m_rangePtr == m_rangePtr)
                {  // if sequences are not the same
                    return m_currVal == rhs.m_currVal || m_atEnd == rhs.m_atEnd;
                }
            }
            return false;
        }

        // Return false if the same!
        bool operator!=(const iterator& rhs)
        {
            return !(*this == rhs);
        }

        // get current value;
        T operator*()
        {
            if (m_listPtr)
            {
                return *m_listIt;
            }
            else
            {
                return m_currVal;
            }
        }

    private:
        typename ListOrRangeType::ListType*          m_listPtr;
        typename ListOrRangeType::ListType::iterator m_listIt;

        typename ListOrRangeType::RangeType* m_rangePtr;
        T1                                   m_currVal;  // For Range and All types
        bool                                 m_atEnd;
    };

    iterator<T> begin()
    {
        return iterator<T>(m_rangeVariant, false);
    }

    iterator<T> end()
    {
        return iterator<T>(m_rangeVariant, true);
    }

private:
    VariantType m_rangeVariant;
};

#define INIT_TIMER auto start = std::chrono::high_resolution_clock::now();
#define START_TIMER start     = std::chrono::high_resolution_clock::now();
#define STOP_TIMER(name)                                                                                      \
    double count =                                                                                            \
        std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count(); \
    std::cout << "RUNTIME of " << name << ": " << count << " ms " << std::endl;

void doTest()
{
    typedef unsigned int Type;
    std::set<Type>       s;
    unsigned int         max = 10000;
    std::vector<Type>    vec(max);
    std::pair<Type, Type> a(0, max);

    for (int j = 0; j < vec.size(); j++)
    {
        vec[j] = j;
    }
    std::copy(vec.begin(), vec.end(), std::inserter(s, s.begin()));

    std::cout << "VECTOR:",
        // td::copy(vec.begin(),vec.end(),std::ostream_iterator<Type>(std::cout,","));

        std::cout << std::endl
                  << "SET:";
    // std::copy(s.begin(),s.end(),std::ostream_iterator<Type>(std::cout,","));
    // std::cout <<std::endl;

    //    std::cout << "Range: Range" <<std::endl;
    //    Range<Type> r1(a);
    //    for(auto it=r1.begin(); it != r1.end(); ++it) {
    //        std::cout << *it << std::endl;
    //    }
    //
    //    std::cout << "Range: Set" <<std::endl;
    //    Range<Type> r2(s);
    //    for(auto it=r2.begin(); it != r2.end(); ++it) {
    //        std::cout << *it << std::endl;
    //    }
    // std::copy(r2.begin(),r2.end(),std::ostream_iterator<Type>(std::cout,","));

    int loops = 100;
    int n     = 0;
    {
        Range<Type> range(a);
        INIT_TIMER
        START_TIMER
        for (int i = 1; i < loops; i++)
        {
            for (auto it = range.begin(); it != range.end(); ++it)
            {
                n += *it;
            }
        }
        STOP_TIMER("Range: Range Loop")
    }

    {
        Range<Type> range(s);
        INIT_TIMER
        START_TIMER
        for (int i = 1; i < loops; i++)
        {
            for (auto it = range.begin(); it != range.end(); ++it)
            {
                n += *it;
            }
        }
        STOP_TIMER("Range: Set Loop")
    }

    {
        INIT_TIMER
        START_TIMER
        for (int i = 1; i < loops; i++)
        {
            for (auto it = vec.begin(); it != vec.end(); ++it)
            {
                n += *it;
            }
        }
        STOP_TIMER("std::vector Loop")
    }

    {
        INIT_TIMER
        START_TIMER
        for (int i = 1; i < loops; i++)
        {
            for (int j = 0; j < vec.size(); j++)
            {
                n += vec[j];
            }
        }
        STOP_TIMER("std::vector Loop: no iterator")

        std::cout << "n:" << n << std::endl;
    }
}
};

#undef INIT_TIMER
#undef START_TIMER
#undef STOP_TIMER
#endif
