#ifndef GRSF_common_LinearReusableStorage_hpp
#define GRSF_common_LinearReusableStorage_hpp

#include <utility>
#include <algorithm>
#include <random>
#include <vector>
#include <type_traits>

#include "GRSF/common/TypeDefs.hpp"


//template<typename T>
//using LinearReusableStorage = MyContainers::StdVecAligned<T>;


namespace GrowthPolicy{

    /**
    * The OptimalGrowth policy class implements the optimal growth strategy suggested by Andrew
    * Koenig for the std::vector class (see Andrew Koenig's column in the September 1998 issue of
    * JOOP (Journal of Object-Oriented Programming), or the Dr. Dobb's article 'C++ Made Easier:
    * How Vectors Grow', 2001). It applies an exponential growth strategy using a factor of 1.5
    * and additionally ensures that the sizes returns are always multiples of four.
    */
    struct OptimalGrowth{
        inline std::size_t operator()(std::size_t oldCap, std::size_t minCap) {
            auto newCap = std::max<std::size_t>(1.5*oldCap,minCap);
            return ( newCap )?  4*( (newCap-1)/4 + 1 ) :  0;
        }
    };


    struct ExponentialGrowth{
        ExponentialGrowth(double g = 2.0):m_capacityFactor(g){}

        inline std::size_t operator()(std::size_t oldCap, std::size_t minCap) {
            auto newCap = std::max<std::size_t>(m_capacityFactor*oldCap,minCap);
            return ( newCap )?  4*( (newCap-1)/4 + 1 ) :  0;
        }

        const double m_capacityFactor;
    };
};



/**
*   A continous (continuous only for allocated junks) storage similar to std::vector<TElement *>
*   but manages the memory and does not reallocate,
*   If capacity needs to increase, another memory junk is added.
*   The capacity is increased by m_reallocFactor * capacity()
*   This is needed if an element is added and its pointer is stored somewhere else
*   If reallocation would happen, then the stored pointers would be invalid.
*   The container also supports fast shuffling, (shuffles only pointers internally)
*/
template<typename TElement,
         typename TAllocator= Eigen::aligned_allocator<TElement>,
         typename TGrowthPolicy = GrowthPolicy::OptimalGrowth
>
class LinearReusableStorage {
public:
    using ValueType          = TElement;
    using PointerType        = TElement *;
    using ConstPointerType   = const TElement *;
    using ReferenceType      = TElement &;
    using ConstReferenceType = const TElement &;
    using SizeType           = std::size_t;

    using AllocatorType = TAllocator;
    using GrowthPolicyType = TGrowthPolicy;
    using StorageType = std::vector<ValueType*>;


    using value_type      = ValueType;           ///< Type of the underlying values.
    using pointer         = PointerType;         ///< Pointer to a non-const object.
    using const_pointer   = ConstPointerType;    ///< Pointer to a const object.
    using reference       = ReferenceType;       ///< Reference to a non-const object.
    using const_reference = ConstReferenceType ; ///< Reference to a const object.
    using size_type       = SizeType;            ///< Size type of the pointer vector.


    LinearReusableStorage()
        : m_insertIdx(0){}

    explicit LinearReusableStorage(const AllocatorType & alloc)
        : m_allocator(alloc), m_insertIdx(0) {}

    ~LinearReusableStorage() {
        deleteAll();
    };

    inline std::size_t size() const {
        return m_insertIdx;
    }
    inline std::size_t capacity() const {
        return m_storage.size();
    }

    template<typename... T>
    ValueType * emplace_back(T &&... t) {

        if(m_insertIdx == capacity()) {
            // container too small -> increase size by expanding
            expand( m_growth(m_insertIdx,m_insertIdx+1 ) );
        }
        //construct element
        m_allocator.construct(m_storage[ m_insertIdx ],t...);

        return m_storage[ m_insertIdx++ ];
    }

    ValueType * push_back(const ValueType & t) {
        if(m_insertIdx == capacity()) {
            // container too small -> increase size by expanding
            expand( m_growth( m_insertIdx , m_insertIdx+1 ) );
        }
        ValueType * p = m_storage[ m_insertIdx ];
        //construct element
        m_allocator.construct(m_storage[ m_insertIdx ],t);

        return m_storage[ m_insertIdx++ ];
    }

    inline ValueType * back(){
        return (m_storage[m_insertIdx-1]);
    }

    inline const ValueType * back() const {
        return (m_storage[m_insertIdx-1]);
    }

    inline ValueType * front(){
        return (m_storage[0]);
    }

    template<typename T>
    class iterator {
        public:

            using  iterator_category = std::random_access_iterator_tag;  //!< The iterator category.
            using  value_type = T*;         //!< Type of the underlying pointers.
            using  pointer = T*;            //!< Pointer return type.
            using  reference = T* &;          //!< Reference return type.
            using  difference_type = std::size_t;    //!< Difference between two iterators.

            using IteratorType = typename std::conditional< std::is_const<T>::value,
                                    typename StorageType::const_iterator ,
                                    typename StorageType::iterator
                                 >::type;

            template<typename Other>
            friend class iterator;

            iterator(){};

            explicit iterator(const IteratorType & it): m_it(it){}
            explicit iterator(const IteratorType && it) : m_it(std::move(it)){}

            /** pre-increment ++it */
            iterator & operator++() {
                ++m_it;
                return *this;
            }
            /** post-increment it++ */
            iterator operator++(int) {
                iterator it(*this);
                operator++();
                return it;
            }

            /** pre-decrement ++it */
            iterator & operator--() {
                --m_it;
                return *this;
            }
            /** post-decrement ++it */
            iterator & operator--(int) {
                iterator it(*this);
                operator--();
                return it;
            }
            bool operator<=(const iterator &rhs) {return m_it <= rhs.m_it;}
            bool operator>=(const iterator &rhs) {return m_it >= rhs.m_it;}
            bool operator>(const iterator &rhs) {return m_it > rhs.m_it;}
            bool operator<(const iterator &rhs) {return m_it < rhs.m_it;}
            bool operator==(const iterator &rhs) {return m_it == rhs.m_it;}
            bool operator!=(const iterator &rhs) {return m_it != rhs.m_it;}

            difference_type operator-(const iterator & rhs){return m_it - rhs.m_it;}

            iterator & operator+=(  difference_type d){ m_it += d; return *this;}
            iterator & operator+(  difference_type d){
                iterator it(*this);
                it+= d;
                return it;
            }
            iterator & operator-=(  difference_type d){ m_it -= d; return *this;}
            iterator & operator-(  difference_type d){
                iterator it(*this);
                it-= d;
                return it;
            }

            iterator & operator=(const iterator & rhs) = default;
            template<typename Other>
            iterator & operator=(const iterator<Other> & rhs){ m_it = rhs.m_it;};

            iterator( const iterator & r )             = default;
            template<typename Other>
            iterator(const iterator<Other> & rhs): m_it(rhs.m_it){};

            /** non-const access
            *   return a reference to T*
            */
            template<typename U = T, typename = typename std::enable_if<!std::is_const<U>::value>::type >
            reference operator*() {
                return *m_it;
            }
            /** access for const iterator: iterator<const int>
            *   return a copy of the pointer T*
            */
            template<typename U = T, typename = typename std::enable_if<std::is_const<U>::value>::type >
            pointer operator*() {
                return *m_it;
            }

            /** Acces operator to directly access members it->m_value = 3 */
            pointer  operator->() {
                return *m_it;
            }

        private:

        IteratorType  m_it;
    };

    using Iterator = iterator<ValueType>;
    using ConstIterator = iterator<const ValueType>;

    inline Iterator begin() {
        return Iterator(m_storage.begin());
    }

    inline Iterator end() {
        return Iterator(m_storage.begin()+m_insertIdx);
    }

    inline ConstIterator begin() const{
        return ConstIterator(m_storage.begin());
    }

    inline ConstIterator end() const {
        return ConstIterator(m_storage.begin()+m_insertIdx);
    }

    inline ConstIterator cbegin() const{
        return ConstIterator(m_storage.cbegin());
    }

    inline ConstIterator cend() {
        return ConstIterator(m_storage.cbegin()+m_insertIdx);
    }

//    inline typename StorageType::iterator begin() {
//        return m_storage.begin();
//    }
//
//    inline typename StorageType::iterator end() {
//        return m_storage.begin()+m_insertIdx;
//    }
//
//    /** Inconsitency arise with const_iterators, the underlying type is const: value_type * const
//    *   meaning we can still modify the object , only the pointer is const.
//    *   we should wrap iterator with the above uncommented code.
//    */
//    inline typename StorageType::const_iterator begin() const {
//        return m_storage.begin();
//    }
//
//    inline typename StorageType::const_iterator end() const {
//        return m_storage.begin()+m_insertIdx;
//    }

    void clear(){
        // Destruct all elements!
        visit(DestructElement{});
        m_insertIdx = 0;
    }

    void shrinkToFit() {
        // deallocate all junks which are currently behind insertion index
        // (they have not been constructed yet)

        // find the first junk j after m_insertIdx which is fully not used
        auto j = m_junkSizes.begin(); // if end() -> m_insertIdx == 0
        auto e = m_junkSizes.end();
        std::size_t accumIdx = 0;
        while(accumIdx < m_insertIdx && j != m_junkSizes.end() ) {
            accumIdx += j->second;
            ++j;
        }
        // deallocate all junks from junkIt till the end
        for(auto it = j; it!=e; ++it) {
            m_allocator.deallocate(it->first,it->second);
        }
        // remove junk info
        m_junkSizes.erase(j,m_junkSizes.end());

        m_storage.resize(accumIdx); // remove all ptr, leaving only the valid  accumIdx pointers
    };

    void deleteAll() {
        clear();

        for(auto & p : m_junkSizes) {
            m_allocator.deallocate(p.first,p.second);
        }
        m_junkSizes.clear();
        m_storage.clear();
    }

    /** in case of shrinkage , the vector is unshuffled,
        all iterators before n-elements stay valid
        copied pointers from m_storage (emplace_back,push_back) , or reference to objects pointed to in m_storage
        are invalidated!
    */
    void resize(std::size_t n, const ValueType & val = ValueType() ) {
        if(n<m_insertIdx) {
            // we might have shuffled pointers in the valid range from [0,m_insertIdx)
            // if we stupidedly resize m_storage, we have pointers which point to deallocated storage
            // like so:
            // lets say pointers with values [ 0 , 4, 3 | 1 , 2, 5 )  ----> [ 0, 4, 3 ,1 )
            //                                                   *                     * insert_idx
            // this is wrong it should be [ 0, 1 ,2 , 3 )
            // so first modify all pointers in m_storage to realign linearly over all junks :
            // [ 0 , 1 , 2  3 | 5 , 6 , 7 , 8 | 13, 14 | 17 ] (4 junks)
            //                      * insert_idx
            //
            // now resize(3) -> [ 0, 1 ,2 , 3 )  (all 3 continous memory junks afterwards are deleted)
            //                              * insert_idx
            // we dont have invalid pointers now

            unshuffle();

            //now resize, destruct all elements after n till m_insertIdx

            for(std::size_t i = n ; i<m_insertIdx; ++i) {
                m_allocator.destroy(m_storage[i]);
            }
            // reset m_insertIdx
            m_insertIdx = n;
        } else {
            for(std::size_t i = m_insertIdx ; i<n; ++i) {
                push_back(val); // changes m_insert_idx
            }
        }
    }

    /** Realigns all pointers linearly over the memory junks
    * (only up to m_insertIdx, because afterwards all pointers are guaranteed to be ,in sync with the junks
    */
    void unshuffle() {
        std::size_t accumIdx = 0;
        for(auto & j : m_junkSizes) {
            for(std::size_t i = 0; i<j.second; ++i) {
                m_storage[accumIdx+i] = j.first + i;
            }
            accumIdx += j.second;
            if(m_insertIdx<= accumIdx) {
                break;
            }
        }
    }

    void reserve(std::size_t cap) {
        expand(cap);
    }

    template<typename Visitor>
    inline void visit(Visitor && v) {
        std::size_t s = m_insertIdx;
        for(std::size_t i = 0; i< s; ++i) {
            v(*m_storage[i]);
        }
    }

    inline ValueType & operator[](std::size_t i) {
        return *m_storage[i];
    }

    template<template<typename > class Dist = std::uniform_int_distribution, typename RandomEngine = std::default_random_engine >
    void shuffleUniformly() {
        static RandomEngine g;
        Dist<std::size_t> r(0,m_insertIdx-1);

        for( std::size_t i=0; i<m_insertIdx; ++i) {
            //swap the pointers with random chosen element i [0,m_insertIdx)
            std::swap(m_storage[r(g)],m_storage[i]);
        }
    }

private:

    struct DestructElement {
        inline void operator()(ValueType & t) {
            t.~ValueType();
        }
    };


    /** Expand by allocating new elements (not initializing) and storing all pointers */
    inline void expand(std::size_t cap) {

        if(cap <= capacity()) {
            return;
        }
        // expand to e elements
        auto e = cap-capacity();

        // allocate junk (e elements)
        auto * p = m_allocator.allocate(e);
        // store the junk information
        m_junkSizes.emplace_back(p,e);

        // add pointers
        m_storage.reserve(cap);
        for(std::size_t i=0; i<e; ++i) {
            m_storage.push_back(p+i);
        }

    }

    AllocatorType m_allocator;

    std::vector< std::pair<ValueType*,std::size_t> > m_junkSizes;
    GrowthPolicyType m_growth;

    StorageType m_storage;

    /** Insertion idx, is always smaller then m_storage.size() */
    std::size_t  m_insertIdx = 0;
};


#endif
