#ifndef GMSF_Common_Range_hpp
#define GMSF_Common_Range_hpp

#include <type_traits>

#include "StaticAssert.hpp"

/**
* A fast sorted integral range which can be constructed from std::pair or std::set
*/
template<typename IntType, typename Allocator = std::allocator<IntType>  >
class Range{
	public:

	    using RangeType = std::vector<IntType,Allocator>;

	    STATIC_ASSERTM( std::is_integral<IntType>::value , IntType_Needs_to_be_Integeral );
        Range(): m_linear(false) {};

        // default copy cosntructor and assignment operator;
	    Range( const Range & r ) = default;
	    Range & operator = ( const Range & r) = default;

        // Move constructor
        Range( Range && r ) = default;
        // Move assignment
        Range & operator = ( Range && r ) = default;

        //Constructor for RangeType = std::vector,
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
		    STATIC_ASSERTM( std::is_integral<T>::value , T_Needs_to_be_Integeral);
		    unsigned int N = p.second-p.first;
		    if(p.second < p.first){ N = 0;} // leads to empty m_v

			m_v.resize(N);
			unsigned int i=0;
			for(auto & v : m_v){
				v=p.first+i; ++i;
			}
		}

        template<typename T>
		Range & operator=( const std::pair<T,T> & p){
            this->operator=( Range(p) );
            return *this;
		}


		/*
        * Constructs a list in from the sorted set
        */
        template<typename T>
		Range(const std::set<T> & p){
		    m_linear = true; // init
		    STATIC_ASSERTM( std::is_integral<T>::value , T_Needs_to_be_Integeral);
			m_v.resize(p.size());
			auto it = p.begin();
			for(auto & v : m_v){
				v=*it;

				if(m_linear && std::next(it) != p.end() && *std::next(it) - *it != 1){
                    m_linear = false;
				}
				++it;
			}
		}

        template<typename T>
		Range & operator=( const std::set<T> & p){
            this->operator=( Range(p) );
            return *this;
		}



		class iterator : public std::iterator_traits<typename RangeType::iterator >{
			public:
			    using iterator_traits = std::iterator_traits<typename RangeType::iterator >;

			    iterator(){};

				iterator(const typename RangeType::iterator & it)
                    : m_it(it){}

				iterator(const typename RangeType::iterator && it)
                    : m_it(std::move(it)){}

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
                iterator & operator+=( typename iterator_traits::difference_type d){ m_it += d; return *this;}

                iterator & operator=(const iterator & rhs) = default;
                iterator( const iterator & r ) = default;

                IntType & operator*() {
                    return *m_it;
                }

			private:
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

        typename RangeType::size_type size(){
            return m_v.size();
        }

        bool isLinear(){return m_linear;}

    private:
        bool m_linear;
        RangeType m_v;

        inline void init(){
            std::sort(m_v.begin(),m_v.end());
            ASSERTMSG( std::adjacent_find(m_v.begin(),m_v.end()) == m_v.end() , "Elements in Range<IntType> are not unique!")
            m_linear = (m_v.size()>0) && ( m_v.size() == (m_v.back()- m_v.front() + 1) );
        }
};


#endif
