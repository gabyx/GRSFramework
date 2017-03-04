// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef rangeTest2_hpp
#define rangeTest2_hpp

#include <cassert>
#include <chrono>
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>

#include <boost/mpl/at.hpp>
#include <boost/variant.hpp>

#define LOOPS 100
#define MAXEL 100000

namespace Test2
{
struct get_pointer : boost::static_visitor<void*>
{
    template <class T>
    void* operator()(T& element) const
    {
        return &element;
    }
};

namespace ListOrRangeTypes
{
template <typename Type>
struct ListTypes
{
    using ListType      = std::set<Type>;
    using ContainerType = std::set<Type>;  ///< ordered list!
};

template <typename Type>
struct RangeTypes
{
    using RangeType = std::pair<Type, Type>;

    class Iterator
    {
    public:
        Iterator() : m_rangePtr(nullptr), m_currVal(0){};
        ~Iterator(){};
        Iterator(RangeType& range, bool atEnd = false) : m_rangePtr(&range)
        {
            if (atEnd)
            {
                m_currVal = m_rangePtr->second;
            }
            else
            {
                m_currVal = m_rangePtr->first;
            }
        };

        /** pre-increment ++it
        * Allow to iterate over the end of the sequence
        */
        Iterator& operator++()
        {
            ++m_currVal;
            return *this;
        }

        /** post-increment it++ */
        Iterator operator++(int)
        {
            Iterator it(*this);
            operator++();
            return it;
        }

        bool operator==(const Iterator& rhs)
        {
            if (m_rangePtr == rhs.m_rangePtr && m_rangePtr)
            {  // if sequences are the same
                return m_currVal == rhs.m_currVal;
            }
            return false;
        }

        // Return false if the same!
        bool operator!=(const Iterator& rhs)
        {
            return !(*this == rhs);
        }

        // get current value;
        Type operator*()
        {
            return m_currVal;
        }

    private:
        RangeType* m_rangePtr;
        Type m_currVal;
    };

    class ContainerType : public RangeType
    {
    public:
        typedef Iterator iterator;

        ContainerType(RangeType& range) : RangeType(range)
        {
            // if range wrong, set to no range!
            if (this->first > this->second)
            {
                this->first  = 0;
                this->second = 0;
            }
        }
        iterator begin()
        {
            return iterator(*this);
        };
        iterator end()
        {
            return iterator(*this, true);
        };
    };
};

template <typename Type>
struct ComboTypes
{
    using ComboType =
        boost::variant<typename ListTypes<Type>::ContainerType,
                       typename RangeTypes<Type>::ContainerType>;  // Indexes into the variants are defined above!

    template <long int N>
    struct Get
    {
        using type = typename boost::mpl::at_c<typename ComboType::types, N>::type;
    };

    class Iterator
    {
    public:
        Iterator(ComboType& comboRange, bool atEnd = false)
        {
            get_pointer vis;
            m_comboRangePtr = comboRange.apply_visitor(vis);
            m_which         = comboRange.which();
            switch (m_which)
            {
                case (0):
                {
                    auto ptr    = static_cast<typename Get<0>::type*>(m_comboRangePtr);
                    auto* itPtr = new typename Get<0>::type::iterator();
                    if (atEnd)
                    {
                        *itPtr = ptr->end();
                    }
                    else
                    {
                        *itPtr = ptr->begin();
                    }
                    m_comboRangeItPtr = static_cast<void*>(itPtr);
                    break;
                }
                case (1):
                {
                    auto ptr    = static_cast<typename Get<1>::type*>(m_comboRangePtr);
                    auto* itPtr = new typename Get<1>::type::iterator();
                    if (atEnd)
                    {
                        *itPtr = ptr->end();
                    }
                    else
                    {
                        *itPtr = ptr->begin();
                    }
                    m_comboRangeItPtr = static_cast<void*>(itPtr);
                    break;
                }
            }
        };

        // Delete assignment operator
        Iterator& operator=(const Iterator&) = delete;
        Iterator& operator=(Iterator&) = delete;

        ~Iterator()
        {
            // Delete iterator pointer
            if (m_comboRangeItPtr)
            {
                switch (m_which)
                {
                    case (0):
                    {
                        auto ptrIt = static_cast<typename Get<0>::type::iterator*>(m_comboRangeItPtr);
                        delete ptrIt;
                        break;
                    }
                    case (1):
                    {
                        auto ptrIt = static_cast<typename Get<1>::type::iterator*>(m_comboRangeItPtr);
                        delete ptrIt;
                        break;
                    }
                }
                m_comboRangeItPtr = nullptr;
            }
        }

        Iterator(const Iterator& it)
        {
            m_comboRangePtr = it.m_comboRangePtr;

            // Delete iterator pointer
            if (m_comboRangeItPtr)
            {
                switch (m_which)
                {
                    case (0):
                    {
                        auto ptrIt = static_cast<typename Get<0>::type::iterator*>(m_comboRangeItPtr);
                        delete ptrIt;
                        break;
                    }
                    case (1):
                    {
                        auto ptrIt = static_cast<typename Get<1>::type::iterator*>(m_comboRangeItPtr);
                        delete ptrIt;
                        break;
                    }
                }
                m_comboRangeItPtr = nullptr;
            }

            m_which = it.m_which;
            m_atEnd = it.m_atEnd;
            if (it.m_comboRangeItPtr)
            {
                switch (m_which)
                {
                    case (0):
                    {
                        auto ptrIt        = static_cast<typename Get<0>::type::iterator*>(it.m_comboRangeItPtr);
                        auto ptrItNew     = new typename Get<0>::type::iterator;
                        *ptrItNew         = *ptrIt;  // copy value of iterator
                        m_comboRangeItPtr = static_cast<void*>(ptrItNew);
                        break;
                    }
                    case (1):
                    {
                        auto ptrIt        = static_cast<typename Get<1>::type::iterator*>(it.m_comboRangeItPtr);
                        auto ptrItNew     = new typename Get<1>::type::iterator;
                        *ptrItNew         = *ptrIt;  // copy value of iterator
                        m_comboRangeItPtr = static_cast<void*>(ptrItNew);
                        break;
                    }
                }
            }
        }

        /** pre-increment ++it
        * Allow to iterate over the end of the sequence
        */
        Iterator& operator++()
        {
            if (m_comboRangeItPtr)
            {
                switch (m_which)
                {
                    case (0):
                    {
                        auto itPtr = static_cast<typename Get<0>::type::iterator*>(m_comboRangeItPtr);
                        (*itPtr)++;
                        break;
                    }
                    case (1):
                    {
                        auto itPtr = static_cast<typename Get<1>::type::iterator*>(m_comboRangeItPtr);
                        (*itPtr)++;
                        break;
                    }
                }
            }
            return *this;
        }
        /** post-increment it++
        *
        */
        Iterator operator++(int)
        {
            Iterator it(*this);
            operator++();
            return it;
        }

        bool operator==(const Iterator& rhs)
        {
            // if wrong type or if sequence pointers are not the same or one pointer is NULL, the iterators can not be
            // the same!
            if (m_which != rhs.m_which || m_comboRangePtr != rhs.m_comboRangePtr || m_comboRangePtr == nullptr ||
                rhs.m_comboRangePtr == nullptr || m_comboRangeItPtr == nullptr || rhs.m_comboRangeItPtr == nullptr)
            {
                return false;
            }

            switch (m_which)
            {
                case (0):
                {
                    auto itPtr1 = static_cast<typename Get<0>::type::iterator*>(m_comboRangeItPtr);
                    auto itPtr2 = static_cast<typename Get<0>::type::iterator*>(rhs.m_comboRangeItPtr);
                    return (*itPtr1) == (*itPtr2);
                    break;
                }
                case (1):
                {
                    auto itPtr1 = static_cast<typename Get<1>::type::iterator*>(m_comboRangeItPtr);
                    auto itPtr2 = static_cast<typename Get<1>::type::iterator*>(rhs.m_comboRangeItPtr);
                    return (*itPtr1) == (*itPtr2);
                    break;
                }
            }
        }

        // Return false if the same!
        bool operator!=(const Iterator& rhs)
        {
            return !(*this == rhs);
        }

        Type operator*()
        {
            switch (m_which)
            {
                case (0):
                {
                    return *(*static_cast<typename Get<1>::type::iterator*>(m_comboRangeItPtr));
                    break;
                }
                case (1):
                {
                    return *(*static_cast<typename Get<1>::type::iterator*>(m_comboRangeItPtr));
                }
            }
        }

    private:
        void* m_comboRangePtr   = nullptr;
        void* m_comboRangeItPtr = nullptr;
        unsigned int m_which    = 0;
        bool m_atEnd            = false;
    };

    class ContainerType : public ComboType
    {
    public:
        typedef Iterator iterator;

        ContainerType() : ComboType()
        {
        }
        // Copy Constructor
        ContainerType(ContainerType& c)
        {
            *this = c;
        }
        // Assigment
        template <typename T>
        ContainerType& operator=(const T& v)
        {
            ComboType::operator=(v);
        }

        iterator begin()
        {
            return iterator(*this);
        };

        iterator end()
        {
            return iterator(*this, true);
        };
    };
};
};

#define INIT_TIMER auto start = std::chrono::high_resolution_clock::now();
#define START_TIMER start     = std::chrono::high_resolution_clock::now();
#define STOP_TIMER(name)                                                                                      \
    double count =                                                                                            \
        std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count(); \
    std::cout << "RUNTIME of " << name << ": " << count << " ms " << std::endl;

void doTest()
{
    std::cout << "Test ============================" << std::endl;
    using namespace ListOrRangeTypes;

    typedef unsigned int Type;
    std::set<Type> s;
    unsigned int max = MAXEL;
    std::vector<Type> vec(max);
    for (int j = 0; j < vec.size(); j++)
    {
        vec[j] = j * j;
    }
    while (s.size() < max)
    {
        s.insert(rand());
    }
    std::pair<Type, Type> a(0, max);

    // std::cout << "VECTOR:";
    // std::copy(vec.begin(),vec.end(),std::ostream_iterator<Type>(std::cout,","));

    // std::cout <<std::endl << "SET:";
    // std::copy(s.begin(),s.end(),std::ostream_iterator<Type>(std::cout,","));
    // std::cout <<std::endl;

    int loops = LOOPS;
    int n     = 0;
    {
        RangeTypes<Type>::ContainerType range(a);
        INIT_TIMER
        START_TIMER
        auto itEnd = range.end();
        for (int i = 1; i < loops; i++)
        {
            for (auto it = range.begin(); it != itEnd; ++it)
            {
                n += *it * i;
            }
        }
        STOP_TIMER("Range: [Pair]")
    }

    {
        ListTypes<Type>::ContainerType range(s);
        INIT_TIMER
        START_TIMER
        auto itEnd = range.end();
        for (int i = 1; i < loops; i++)
        {
            for (auto it = range.begin(); it != itEnd; ++it)
            {
                n += *it * i;
            }
        }
        STOP_TIMER("Range: [Set]")
    }

    {
        auto rr = RangeTypes<Type>::ContainerType(a);

        ComboTypes<Type>::ContainerType combo;
        combo = rr;
        INIT_TIMER
        START_TIMER
        auto itEnd = combo.end();
        for (int i = 1; i < loops; i++)
        {
            for (auto it = combo.begin(); it != itEnd; ++it)
            {
                // std::cout << "it:" << *it << std::endl;
                n += *it * i;
            }
        }
        STOP_TIMER("Combo [Pair]:")
    }

    {
        ComboTypes<Type>::ContainerType combo;
        combo = typename ListTypes<Type>::ContainerType(s);
        INIT_TIMER
        START_TIMER
        auto itEnd = combo.end();
        for (int i = 1; i < loops; i++)
        {
            for (auto it = combo.begin(); it != itEnd; ++it)
            {
                n += *it * i;
            }
        }
        STOP_TIMER("Combo [Set]:")
    }

    {
        INIT_TIMER
        START_TIMER
        auto itEnd = vec.end();
        for (int i = 1; i < loops; i++)
        {
            for (auto it = vec.begin(); it != vec.end(); ++it)
            {
                n += *it * i;
            }
        }
        STOP_TIMER("std:.vector Loop")
    }

    {
        INIT_TIMER
        START_TIMER
        auto endIt = s.end();
        for (int i = 1; i < loops; i++)
        {
            for (auto it = s.begin(); it != endIt; ++it)
            {
                n += *it * i;
            }
        }
        STOP_TIMER("std::set Loop")
    }

    {
        INIT_TIMER
        START_TIMER
        for (int i = 1; i < loops; i++)
        {
            for (int j = 0; j < vec.size(); j++)
            {
                n += vec[j] * i;
            }
        }
        STOP_TIMER("Vector Loop: no iterator")
    }

    std::cout << "n:" << n << std::endl;
};

template <typename Type>
class Range
{
public:
    typedef std::pair<Type, Type> Pair;
    typedef std::set<Type> Set;

    Range(const Pair& pair)
    {
        m_ptr   = static_cast<void*>(new Pair(pair));
        m_which = 0;
    }
    Range(Pair&& pair)
    {
        m_ptr   = static_cast<void*>(new Pair(std::move(pair)));
        m_which = 0;
    }

    Range(const Set& set)
    {
        m_which = 1;
        m_ptr   = static_cast<void*>(new Set(set));
    }
    Range(Set&& set)
    {
        m_which = 1;
        m_ptr   = static_cast<void*>(new Set(std::move(set)));
    }

    Range(const Range& r)
    {
        *this = r;
    }

    /// Move Constructor
    Range(Range&& r)
    {
        *this = std::move(r);
    }

    /// Move Assigment
    Range& operator=(Range&& r)
    {
        std::cout << "Move Ass:" << std::endl;
        assert(r.m_ptr);
        if (m_ptr != r.m_ptr && this != &r)
        {                    // Prevent self-assignment
            this->~Range();  // delete resources
            m_ptr     = std::move(r.m_ptr);
            m_which   = std::move(r.m_which);
            r.m_ptr   = nullptr;
            r.m_which = 0;
        }
    }

    /// Assigment
    Range& operator=(const Range& r)
    {
        if (m_ptr != r.m_ptr && this != &r)
        {  // Prevent self-assignment
            if (r.m_which == 0)
            {
                if (m_which == 0 && m_ptr)
                {
                    *static_cast<Pair*>(m_ptr) = *static_cast<Pair*>(r.m_ptr);  // Copy
                }
                m_ptr = static_cast<void*>(new Pair(*static_cast<Pair*>(r.m_ptr)));  // Make new and Copy
            }
            else
            {
                if (m_which == 1 && m_ptr)
                {
                    *static_cast<Set*>(m_ptr) = *static_cast<Set*>(r.m_ptr);  // Copy
                }
                m_ptr = static_cast<void*>(new Set(*static_cast<Set*>(r.m_ptr)));  // Make new and Copy
            }
            m_which = r.m_which;
        }
    }

    ~Range()
    {
        if (m_ptr)
        {
            if (m_which == 0)
            {
                auto p = static_cast<Pair*>(m_ptr);
                delete p;
            }
            else if (m_which == 1)
            {
                auto p = static_cast<Set*>(m_ptr);
                delete p;
            }
            m_which = 0;
        }
    }

    class iterator
    {
    public:
        iterator() : m_r(nullptr), m_cur(0){};

        iterator(Range* r, bool atEnd = false) : m_r(r)
        {
            if (!m_r->m_ptr)
            {
                m_cur = 0;
                return;
            }
            if (m_r->m_which == 0)
            {
                auto p = static_cast<Pair*>(m_r->m_ptr);
                if (atEnd)
                {
                    m_cur = p->second;
                }
                else
                {
                    m_cur = p->first;
                }
            }
            else
            {
                auto p = static_cast<Set*>(m_r->m_ptr);
                if (atEnd)
                {
                    m_it = p->end();
                }
                else
                {
                    m_it = p->begin();
                }
            }
        };

        // Delete assignment operator
        iterator& operator=(const iterator&) = delete;
        iterator& operator=(iterator&) = delete;

        ~iterator()
        {
        }

        iterator(const iterator& it) : m_r(it.m_r), m_cur(it.m_cur)
        {
        }

        /** pre-increment ++it, allow to iterate over the end of the sequences*/
        iterator& operator++()
        {
            if (m_r->m_which == 0)
            {
                ++m_cur;
            }
            else
            {
                ++m_it;
            }
            return *this;
        }
        /** post-increment it++ */
        iterator operator++(int)
        {
            iterator it(*this);
            operator++();
            return it;
        }

        bool operator==(const iterator& rhs)
        {
            if (m_r->m_which == 0)
            {
                return m_cur == rhs.m_cur;  // Two iterators for differente ranges, might compare equal!
            }
            else
            {
                return m_it == rhs.m_it;
            }
        }

        // Return false if the same!
        bool operator!=(const iterator& rhs)
        {
            return !(*this == rhs);
        }

        Type operator*()
        {
            if (m_r->m_which == 0)
            {
                return m_cur;
            }
            else
            {
                return *m_it;
            }
        }

    private:
        Range* m_r;
        typename Set::iterator m_it;
        Type m_cur;
    };

    iterator begin()
    {
        return iterator(this);
    }

    iterator end()
    {
        return iterator(this, true);
    }

private:
    unsigned int m_which = 0;
    void* m_ptr          = nullptr;
};

#define INIT_TIMER auto start = std::chrono::high_resolution_clock::now();
#define START_TIMER start     = std::chrono::high_resolution_clock::now();
#define STOP_TIMER(name)                                                                                      \
    double count =                                                                                            \
        std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count(); \
    std::cout << "RUNTIME of " << name << ": " << count << " ms " << std::endl;

int doAsserts()
{
    std::cout << "Assert Test ============================" << std::endl;
    unsigned int max = 100;
    typedef Range<int> T;
    std::set<int> s;
    while (s.size() < max)
    {
        s.insert(rand());
    }
    T t(s);
    T t1 = t;
    t1   = std::move(t);
    t    = t1;
    std::cout << "Assert Test ============================" << std::endl;
}

int doTest2()
{
    doAsserts();

    std::cout << "Test2 ============================" << std::endl;
    typedef unsigned int Type;
    std::set<Type> s;
    unsigned int max = MAXEL;
    std::vector<Type> vec(max);
    for (int j = 0; j < vec.size(); j++)
    {
        vec[j] = j * j;
    }
    while (s.size() < max)
    {
        s.insert(rand());
    }
    std::pair<Type, Type> a(0, max);

    int loops = LOOPS;
    int n     = 0;
    {
        Range<Type> range(a);
        INIT_TIMER
        START_TIMER
        auto itEnd = range.end();
        for (int i = 1; i < loops; i++)
        {
            for (auto it = range.begin(); it != itEnd; ++it)
            {
                n += *it * i;
                // std::cout << *it << std::endl;
            }
        }
        STOP_TIMER("Range: [Pair] ")
    }
    {
        Range<Type> range(s);
        INIT_TIMER
        START_TIMER
        auto itEnd = range.end();
        for (int i = 1; i < loops; i++)
        {
            for (auto it = range.begin(); it != itEnd; ++it)
            {
                n += *it * i;
                // std::cout << *it << std::endl;
            }
        }
        STOP_TIMER("Range: [Set]")
    }

    {
        INIT_TIMER
        START_TIMER
        for (unsigned int i = 1; i < loops; ++i)
        {
            for (unsigned int j = 0; j < max; ++j)
            {
                n += i * j;
            }
        }
        STOP_TIMER("Range: For loop")
    }

    {
        INIT_TIMER
        START_TIMER
        auto s1    = s;
        auto itEnd = s1.end();
        for (int i = 1; i < loops; i++)
        {
            for (auto it = s1.begin(); it != itEnd; ++it)
            {
                n += *it * i;
                // std::cout << *it << std::endl;
            }
        }
        STOP_TIMER("Std: Set")
    }

    std::cout << "Finished" << std::endl;
}
};

#undef INIT_TIMER
#undef START_TIMER
#undef STOP_TIMER

#endif
