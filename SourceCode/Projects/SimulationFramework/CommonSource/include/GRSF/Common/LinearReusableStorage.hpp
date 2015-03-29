#ifndef GRSF_Common_LinearReusableStorage_hpp
#define GRSF_Common_LinearReusableStorage_hpp

#include <utility>
#include <random>
#include <vector>

/// Special and simple effective element container which is reusable.
/// This means the storage is a pointer list with allocated elements and
/// calls to clear only resets the index and does not delete the allocated memory, thus re-insertion is very fast.
/// Take care that insertion does not fully initialize the element, it only returns a valid pointer, initialization needs to be
/// beformed seperately if the node has not been newly allocated (return flag in insert function)
template<typename TElement>
class LinearReusableStorage {
public:
    using ElementType = TElement;
    using StorageType = std::vector<ElementType *>;

    ~LinearReusableStorage() {
        deleteAll();
    };

    inline std::size_t size() const{ return m_storage.size();}

    template<typename... T>
    std::pair<ElementType *,bool> insert(T &&... t) {
        ElementType * p;
        bool inserted = false;

        if(m_insertIdx < m_storage.size()) {
            p = m_storage[ m_insertIdx ]; // get pointer from storage
        } else {
            // allocate new node, and emplace
            p = new ElementType{ m_insertIdx, std::forward<T>(t)... };
            inserted = true;
            m_storage.emplace_back( p );
        }
        ++m_insertIdx;
        return std::make_pair(p,inserted);
    }

    inline typename StorageType::iterator begin(){
        return m_storage.begin();
    }

    inline typename StorageType::iterator end(){
        return m_storage.end();
    }

    void clear() {
        // Clear all elements!
        ClearElement c;
        visit(c);
        m_insertIdx = 0;
    }

    void shrinkToFit(){
        // delete all allocated elements which are currently behind insertion index
        auto s = m_storage.size();
        for(std::size_t i = m_insertIdx ; i < s; ++i){
            delete m_storage[i];
        }
        m_storage.resize(m_insertIdx); // remove all ptr, leaving only the valid  m_insert_idx pointers
    };

    void deleteAll() {
        m_insertIdx = 0;
        for(auto & n: m_storage ) {
            delete n;
        }
    }

    void reserve(std::size_t nodes) {
        std::size_t idx = m_insertIdx;
        std::size_t s = m_storage.size();
        if(nodes > s  ) {
            for(std::size_t i=0; i<(nodes-s); ++i) {
                insert();
            }
        }
        m_insertIdx = idx;
    }

    template<typename Visitor>
    inline void visit(Visitor & v) {
        std::size_t s = m_insertIdx;
        for(std::size_t i = 0; i< s; ++i) {
            v(*m_storage[i]);
        }
    }

    inline ElementType * operator[](std::size_t i){
        return m_storage[i];
    }

    inline ElementType * getElement(std::size_t i){
        return m_storage[i];
    }

    template<template<typename > class Dist = std::uniform_int_distribution, typename RandomEngine = std::default_random_engine >
    void shuffleUniformly() {
        static RandomEngine g;
        Dist<std::size_t> r(0,m_storage.size()-1);

        ElementType * temp;
        for( ElementType* & p : m_storage) {
            //swap the pointers with random chosen element
            temp = p;
            ElementType * & p2 = m_storage[r(g)];
            p = p2;
            p2 = temp;
        }
    }

private:

    struct ClearElement{
        template<typename T>
        inline void operator()(T & t){
            t.clear();
        }
    };

    StorageType m_storage;
    // Take care, if you want to switch this to a value-type all returned pointer in insertNode
    // are invalidate if reallocation happens

    std::size_t  m_insertIdx = 0; ///< Insertion idx counter, to fast clear and re-insert
};
#endif
