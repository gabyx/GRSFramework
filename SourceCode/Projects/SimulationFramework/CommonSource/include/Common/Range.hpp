#ifndef Range_hpp
#define Range_hpp

#include <type_traits>

#include "StaticAssert.hpp"

/**
* A fast sorted integral range which can be constructed from std::pair or std::set
*/
template<typename IntType, typename Allocator = std::allocator<IntType>  >
class Range{
	public:



	    typedef std::vector<IntType,Allocator> RangeType;

	    STATIC_ASSERT2( std::is_integral<IntType>::value , IntType_Needs_to_be_Integeral);
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
        Range( RangeType v) : m_v(std::move(v)) {  sort(); }

        template<typename Iterator>
        Range(Iterator begin, Iterator end) : m_v(begin,end) {  sort(); }

        //if called with rvalue (temporary) it gets moved already into v, then we move again into m_v
		Range & operator=( RangeType v ){
		    using std::swap;
            m_v=std::move(v); sort();
		}

        template<typename T>
		Range(const std::pair<T,T> & p){

		    STATIC_ASSERT2( std::is_integral<T>::value , T_Needs_to_be_Integeral);
		    unsigned int N;
		    if(p.second < p.first){
                N = 0; // leads to empty m_v
		    }else{
                N = p.second-p.first;
		    }
			m_v.resize(N);
			for(T i=0; i<N; i++){
				m_v[i]=p.first+i;
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
        RangeType m_v;

        inline void sort(){
            std::sort(m_v.begin(),m_v.end());
        }
};


#endif
