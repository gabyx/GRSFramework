// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <cassert>
#include <iostream>
#include <type_traits>
#include <utility>
#include <algorithm>
#include <chrono>
#include <set>
#include <map>
#include <vector>
#include <list>
#include <iterator>

#define LOOPS 200
#define MAXEL 1000000

namespace test4{
    /**
* A fast sorted integral range which can be constructed from std::pair or std::set
*/
template<typename IntType, typename Allocator = std::allocator<IntType>  >
class Range{
	public:



	    typedef std::list<IntType,Allocator> RangeType;

	    //STATIC_ASSERT2( std::is_integral<IntType>::value , IntType_Needs_to_be_Integeral);
        Range(){};

        // default copy cosntructor and assignment operator;
	    Range( const Range & r ) = default;
	    Range & operator = ( const Range & r) = default;

        // Move constructor
        Range( Range && r ): m_v(std::move(r.m_v)){}
        // Move assignment
        Range & operator = ( Range && r ){ m_v = std::move(r.m_v); }

        //Constructor for std::vector,
        // if called with rvalue (temporary) it gets moved already into v, then we move again into m_v
        Range( RangeType v) : m_v(std::move(v)) {  init(); }

        template<typename Iterator>
        Range(Iterator begin, Iterator end) : m_v(begin,end) {  init(); }

        //if called with rvalue (temporary) it gets moved already into v, then we move again into m_v
		Range & operator=( RangeType v ){
            m_v=std::move(v); init();
		}

        /*
        * Constructs a list in the range [p.first, p.second)
        */
        template<typename T>
		Range(const std::pair<T,T> & p){
            m_linear = true;
		    //STATIC_ASSERT2( std::is_integral<T>::value , T_Needs_to_be_Integeral);
		    unsigned int N;
		    if(p.second < p.first){
                N = 0; // leads to empty m_v
		    }else{
                N = p.second-p.first;
		    }
			m_v.resize(N);
			unsigned int i=0;
			for(auto & v : m_v){
				v=p.first+i;
				i++;
			}
		}

        template<typename T>
		Range & operator=( const std::pair<T,T> & p){
            this->operator=( Range(p) );
		}


		class iterator : public std::iterator_traits<typename RangeType::iterator >{
			public:
			    typedef std::iterator_traits<typename RangeType::iterator > iterator_traits;

			    iterator():m_diffValue(0){};

				iterator(const typename RangeType::iterator & it)
                    : m_it(it), m_diffValue(0){}

				iterator(const typename RangeType::iterator && it)
                    : m_it(std::move(it)), m_diffValue(0){}

				void setDiffValue(IntType v){ m_diffValue=v; }
				IntType getCurrentDiff(){ return *m_it-m_diffValue;}

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

                bool operator==(const iterator &rhs) {return m_it == rhs.m_it;}
                bool operator!=(const iterator &rhs) {return m_it != rhs.m_it;}

                typename iterator_traits::difference_type operator-(const iterator & rhs){return m_it - rhs.m_it;}
                iterator & operator+=( typename iterator_traits::difference_type d){ m_it += d;}

                iterator & operator=(const iterator & rhs) = default;
                iterator( const iterator & r ) = default;

                IntType operator*() {
                    return *m_it;
                }

			private:
			IntType m_diffValue;
			typename RangeType::iterator m_it;
		};

		iterator begin(){ return iterator(m_v.begin());}
		iterator end()  { return iterator(m_v.end())  ;}

        bool empty(){
            return m_v.empty();
        }

        void clear(){
            m_v.clear();
        }


    private:
        bool m_linear;
        RangeType m_v;

        inline void init(){
            m_v.sort();
            m_v.unique();
            m_linear = (m_v.size()>0) && ( m_v.size() == (m_v.back()-m_v.front() + 1) );
        }
};
};


namespace test3{
template<typename Type>
class Range{
	public:

	    Range( const Range & r): m_v(r.m_v){}
        Range( Range && r): m_v(std::move(r.m_v)){}

        //Constructor for std::vector
		Range( const std::vector<Type> & v) : m_v(v)      {  sort(); }
        Range( std::vector<Type> && v) : m_v(std::move(v)) {  sort(); }

		Range & operator=( const std::vector<Type> & v ){
            m_v=v; sort();
		}
		Range & operator=( std::vector<Type> && v ){
            m_v=std::move(v); sort();
		}

		Range(const std::pair<Type,Type> & p){
			unsigned int N= p.second-p.first;
			m_v.resize(N);
			for(Type i=0; i<N; i++){
				m_v[i]=p.first+i;
			}
		}

		Range & operator=( const std::pair<Type,Type> & p){
            this->operator=( Range(p) );
		}

        Range & operator=( const Range & r){
            m_v=r.m_v;
        }
        Range & operator=( Range && r){
            m_v=std::move(r.m_v);
        }

		class iterator{
			public:
				iterator(const typename std::vector<Type>::iterator & it)
                    : m_it(it), m_diffValue(0){}

				iterator(const typename std::vector<Type>::iterator && it)
                    : m_it(std::move(it)), m_diffValue(0){}

				void setDiffValue(Type v){ m_diffValue=v; }
				Type getCurrentDiff(){ return *(*this)-m_diffValue;}

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
                bool operator==(const iterator &rhs) {
                    return m_it == rhs.m_it;
                }

                bool operator!=(const iterator &rhs) {
                    return m_it != rhs.m_it;
                }

                Type operator*() {
                    return *m_it;
                }

			private:
			Type m_diffValue;
			typename std::vector<Type>::iterator m_it;
		};

		iterator begin(){ return iterator(m_v.begin());}
		iterator end()  { return iterator(m_v.end())  ;}

    private:
        std::vector<Type> m_v;
        void sort(){
            std::sort(m_v.begin(),m_v.end());
        }
};
};



namespace test2{
template<typename Type>
class Range: public std::vector<Type> {
	public:

        Range():std::vector<Type>(){};
	    Range( const Range & v): std::vector<Type>(v){}
        Range( Range && v): std::vector<Type>(std::move(v)){}

        //Constructor for std::vector
		Range( const std::vector<Type> & v) : std::vector<Type>(v)      {  sort(); }
        Range( std::vector<Type> && v) : std::vector<Type>(std::move(v)) {  sort(); }

		Range & operator=( const std::vector<Type> & v ){
            std::vector<Type>::operator=(v); sort();
		}
		Range & operator=( std::vector<Type> && v ){
            std::vector<Type>::operator=(std::move(v)); sort();
		}

		Range(const std::pair<Type,Type> & p){
			unsigned int N= p.second-p.first;
			this->resize(N);
			for(Type i=0; i<N; i++){
				(*this)[i]=p.first+i;
			}
		}

		Range & operator=( const std::pair<Type,Type> & p){
            this->operator=( Range(p) );
		}

        Range & operator=( const Range & p){
            std::vector<Type>::operator=(p);
        }
        Range & operator=( Range && p){
            std::vector<Type>::operator=(std::move(p));
        }

		class iterator : public std::vector<Type>::iterator{
			public:
				iterator(const typename std::vector<Type>::iterator & v)
                    : std::vector<Type>::iterator(v), m_diffValue(0){}

				iterator(const typename std::vector<Type>::iterator && v)
                    : std::vector<Type>::iterator(std::move(v)), m_diffValue(0){}

				void setDiffValue(Type v){ m_diffValue=v; }
				Type getCurrentDiff(){ return *(*this)-m_diffValue;}

			private:
			Type m_diffValue;
		};

		iterator begin(){ return iterator(std::vector<Type>::begin());}
		iterator end()  { return iterator(std::vector<Type>::end())  ;}

    private:
        void sort(){
            std::sort(std::vector<Type>::begin(),std::vector<Type>::end());
        }
};
};

template<typename T = std::string>
int foo(T&& a = T("asd")){
    std::cout << a << std::endl;
}

struct A{
    template<typename T>
    A(T&& a):m_a(std::forward<T>(a)){}
    test2::Range<int> m_a;
};
namespace test1{

template<typename Type>
class Range{

	public:
	typedef std::pair<Type,Type> Pair;
	typedef std::vector<Type> Set;

	Range(const Pair & pair){
		m_which = 0;
		m_ptr = static_cast<void * >( new Pair(pair) );
	}
	Range(Pair && pair){
		m_which = 0;
		m_ptr = static_cast<void * >( new Pair( std::move(pair) ) );
	}


	Range(const Set & set){
		m_which = 1;
		m_ptr = static_cast<void * >( new Set(set) );
	}
	Range(Set && set){
		m_which = 1;
		m_ptr = static_cast<void * >( new Set( std::move(set) ) );
	}

	// Copy Constructor
	Range(Range const& r)
	  : m_which(r.m_which)
	  , m_ptr(  r.m_which == 0 ?
	  			static_cast<void*>(new Pair(*static_cast<Pair*>(r.m_ptr))) :
	  			static_cast<void*>(new Set(*static_cast<Set*>(r.m_ptr)))
	         )
	{}

	// Move Constructor (rip the guts of temporary r)
	Range(Range && r)
	  : m_which(r.m_which)
	  , m_ptr(r.m_ptr)
	{
		r.m_ptr =  nullptr;
	}



	// Move Assigment
	Range & operator=(Range && r){
		assert(r.m_ptr);
		if(this != &r ){ // Prevent self-assignment ( each object has a different m_ptr, it cannot happen that two objects have the same m_ptr)
			deleteResources();
			m_ptr  =  std::move(r.m_ptr);
			m_which = std::move(r.m_which);
			r.m_ptr = nullptr;
		}
		return *this;
	}

	// Assignment (Copy and Swap)
	Range & operator=(Range r){ // Call by Value for Copy
		r.swap(*this); // Swap our resources with the copy
		// Destruction of r -> cleans our previous resources
		return *this;
	}


	~Range(){
		deleteResources();
	}

	class iterator {
    public:
        iterator():m_r(nullptr),m_cur(0){};

        iterator(Range * r, bool atEnd = false):m_r(r) {
        	if(!m_r->m_ptr){
				m_cur=0;
				return;
        	}
            if(m_r->m_which == 0){
				auto p = static_cast<Pair * >(m_r->m_ptr);
				if(atEnd){
					m_cur = p->second;
				}else{
					m_cur = p->first;
				}
			}else{
				auto p = static_cast<Set * >(m_r->m_ptr);
				if(atEnd){
					m_it = p->end();
				}else{
					m_it = p->begin();
				}
			}
        };

        //Delete assignment operator
        iterator & operator=(const iterator&) = delete;
        iterator & operator=(iterator&) = delete;

        ~iterator() {
        }

        iterator( const iterator & it ): m_r(it.m_r), m_cur(it.m_cur), m_it(it.m_it) {
        }

        /** pre-increment ++it
        * Allow to iterate over the end of the sequence
        */
        iterator & operator++() {
	            if(m_r->m_which == 0){
					++m_cur;
				}else {
					++m_it;
				}
            return *this;
        }
        /** post-increment it++
        *
        */
        iterator operator++(int) {
            iterator it(*this);
            operator++();
            return it;
        }


        bool operator==(const iterator &rhs) {
            if(m_r->m_which == 0){
				return m_cur == rhs.m_cur; // Two iterators for differente ranges, might compare equal!
			}else {
				return m_it == rhs.m_it;
			}

        }

        // Return false if the same!
        bool operator!=(const iterator &rhs) {
            return !(*this==rhs);
        }

        Type operator*() {
            if(m_r->m_which == 0){
				return m_cur ;
			}else {
				return *m_it;
			}
        }
    private:
        Range * m_r;
        typename Set::iterator m_it;
        Type m_cur;
    };

	iterator begin(){
		return iterator(this);
	}

	iterator end(){
		return iterator(this,true);
	}

	private:
	unsigned int m_which = 0;
	void * m_ptr = nullptr;

	void swap(Range& other) noexcept
	{
	    std::swap(m_which, other.m_witch);
	    std::swap(m_ptr,   other.m_ptr);
	}

	void deleteResources(){
		if(m_ptr){
			if(m_which == 0){
				delete static_cast<Pair * >(m_ptr);
			}else{
				delete static_cast<Set * >(m_ptr);
			}
		}
		m_ptr =  nullptr;
	}
};
};


#define INIT_TIMER auto start = std::chrono::high_resolution_clock::now();
#define START_TIMER  start = std::chrono::high_resolution_clock::now();
#define STOP_TIMER(name)  \
    double count = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-start ).count(); \
    std::cout << "RUNTIME of " << name << ": " << count << " ms " << std::endl;

int runRangeTest(){

    foo(2);


    std::cout << "Speed Test ============================" << std::endl;
    typedef int Type;
    std::set<Type> s;
    unsigned int max = MAXEL;

    // Filling vector and set
    std::vector<Type> vec(max);
    for(int j = 0; j< vec.size(); j++) {
        vec[max-j-1] = j*j;
    }
    std::list<Type> list(vec.begin(),vec.end());

    while(s.size()<max){
        s.insert(rand());
    }
    // Making Pair
    std::pair<Type, Type> a(0,max);


    int loops = LOOPS;
    int n = 0;
	{
	    INIT_TIMER
        START_TIMER
        test1::Range<Type> range(a);
        auto itEnd = range.end();
        for(int i=1; i<loops; i++) {
            for(auto it=range.begin(); it != itEnd; ++it) {
                n +=  *it*i;
                //std::cout << *it << std::endl;
            }
        }
        STOP_TIMER("test1::Range: [std::pair] ")
	}
	{
	    INIT_TIMER
        START_TIMER
        test1::Range<Type> range(vec);
        auto itEnd = range.end();
        for(int i=1; i<loops; i++) {
            for(auto it=range.begin(); it != itEnd; ++it) {
                n +=  *it*i;
                //std::cout << *it << std::endl;
            }
        }
        STOP_TIMER("test1::Range: [std::vector]")
	}



	{

        INIT_TIMER
        START_TIMER
        auto s2 = s;
        auto itEnd = s2.end();
        for(int i=1; i<loops; i++) {
            for(auto it=s2.begin(); it != itEnd; ++it) {
                n +=  *it*i;
                //std::cout << *it << std::endl;
            }
        }
        STOP_TIMER("std::set")
	}

		{

        INIT_TIMER
        START_TIMER
        auto itEnd = vec.end();
        for(int i=1; i<loops; i++) {
            for(auto it=vec.begin(); it != itEnd; ++it) {
                n +=  *it*i;
                //std::cout << *it << std::endl;
            }
        }
        STOP_TIMER("std::vector")
	}

	{
        INIT_TIMER
        START_TIMER
		for(unsigned int i=1; i<loops; ++i) {
	        for(unsigned int j = 0; j<max; ++j){
				n+=i*j;
			}
		}
        STOP_TIMER("Normal For Loop")
	}

	{

        INIT_TIMER
        START_TIMER
		test2::Range<Type> range(vec);
        auto itEnd = range.end();
		for(int i=1; i<loops; i++) {
		for(auto it=range.begin(); it != itEnd; ++it) {
	                //std::cout << *it << std::endl;
	                n+=i**it;
	    }
		}
		STOP_TIMER("test2::Range: [std::vector]")
	}
	{

        INIT_TIMER
        START_TIMER
		test2::Range<Type> range{ a };
        auto itEnd = range.end();
		for(int i=1; i<loops; i++) {
		for(auto it=range.begin(); it != itEnd; ++it) {
					n+=i**it;
					//it.setDiffValue(3);
	                //std::cout << it.getCurrentDiff() << std::endl;
	    }
		}
		STOP_TIMER("test2::Range: [std::pair]")

	}

    {
        INIT_TIMER
        START_TIMER
		test3::Range<Type> range{ vec };

        auto itEnd = range.end();
		for(int i=1; i<loops; i++) {
		for(auto it=range.begin(); it != itEnd; ++it) {
					n+=i**it;
					//it.setDiffValue(3);
	                //std::cout << it.getCurrentDiff() << std::endl;
	    }
		}
		STOP_TIMER("test3::Range: [std::vector]")

	}
    {
        INIT_TIMER
        START_TIMER
		test3::Range<Type> range{ a };

        auto itEnd = range.end();
		for(int i=1; i<loops; i++) {
		for(auto it=range.begin(); it != itEnd; ++it) {
					n+=i**it;
					//it.setDiffValue(3);
	                //std::cout << it.getCurrentDiff() << std::endl;
	    }
		}
		STOP_TIMER("test3::Range: [std::pair]")

	}

	{

		test4::Range<Type> range{ list };
        INIT_TIMER
        START_TIMER
        auto itEnd = range.end();
		for(int i=1; i<loops; i++) {
		for(auto it=range.begin(); it != itEnd; ++it) {
					n+=i**it;
					//it.setDiffValue(3);
	                //std::cout << it.getCurrentDiff() << std::endl;
	    }
		}
		STOP_TIMER("test4::Range: [std::list]")

	}

	 {
        INIT_TIMER
        START_TIMER
		test4::Range<Type> range{ a };

        auto itEnd = range.end();
		for(int i=1; i<loops; i++) {
		for(auto it=range.begin(); it != itEnd; ++it) {
					n+=i**it;
					//it.setDiffValue(3);
	                //std::cout << it.getCurrentDiff() << std::endl;
	    }
		}
		STOP_TIMER("test4::Range: [std::pair]")

	}


    {
        typedef unsigned int Type;
        INIT_TIMER
        START_TIMER
        std::set<Type> s;
        while(s.size()<MAXEL){
            s.insert(rand());
        }
        {
            STOP_TIMER("make set")
            std::cout << "Size of set: " << (double) (sizeof(s) + s.size()*sizeof(Type)) / (1 << 20) <<std::endl;
        }


        START_TIMER
        auto s2 = s;
        auto itEnd = s2.end();
        for(int i=1; i<loops; i++) {
            for(auto it=s2.begin(); it != itEnd; ++it) {
                n +=  *it*i;
                //std::cout << *it << std::endl;
            }
        }
        {STOP_TIMER("iterate set")}


        START_TIMER
        std::vector<Type> v;
        v.resize(s.size());
        std::copy(s.begin(),s.end(),v.begin());
        {
            auto itEnd = vec.end();
                for(int i=1; i<loops; i++) {
                    for(auto it=vec.begin(); it != itEnd; ++it) {
                        n +=  *it*i;
                        //std::cout << *it << std::endl;
                    }
                }
        }
        {STOP_TIMER("reassign to std::vector and iterate")}

    }

    std::cout << "n:" << n << std::endl;
}

