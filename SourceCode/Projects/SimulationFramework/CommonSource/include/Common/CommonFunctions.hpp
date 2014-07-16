/*
 *  CommonFunctions.h
 *
 *  Created by Gabriel NÃ¼tzi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef CommonFunctions_h
#define CommonFunctions_h

// Includes =================================
#include <iostream>
#include <iterator>
#include <set>
#include <string>
#include <sstream>
#include <cmath>
#include <limits>
#include <vector>

#include "StaticAssert.hpp"
#include "AssertionDebug.hpp"
#include "MyMatrixDefs.hpp"


namespace Utilities {

/**
* @ingroup Common
* @defgroup CommonFunctions Common Functions
* @brief
*/
/* @{ */


inline bool operator == (const std::string & a, const std::string & b) {
    return    ( a.size() == b.size() )				// optimization on some platforms
              && ( a.compare(b) == 0 );	            // actual compare
}

inline bool operator == (const std::string & a, const char * b) {
    return  strcmp(a.c_str(), b) == 0;
}
inline bool operator == (const char* a, const std::string & b) {
    return b == a;
}


template<typename T> bool stringToType(T & t, const std::string& s);

namespace details {
    namespace stringToTypeImpl {
        /**
        * @brief This functions casts a string into another template specifix type.
        * @param t The ouput where the casted string is put into.
        * @param s The string to cast.
        * @return true if conversion worked, false if not.
        */
        template<typename T>
        bool convert(T& t, const std::string& s) {
            ASSERTMSG( s.find(",") == std::string::npos , "Found comma in string!");
            std::istringstream iss(s);
            return !(iss >> t).fail();
        }

        bool convert(bool & t, const std::string& s);
    };


    /**
    * @brief Helper to convert a string with whitespace-seperated numbers into numbers and passes it to the Functor.
    * N=-1, loops till the end of the string and extracts as many numbers as possible
    */
    template <int N, typename PREC, typename Functor>
    bool stringToTypeFunctorImpl( Functor & f, const std::string & s) {

            unsigned int i=0, j;
            PREC number;
            if(s.empty()) {
                return false;
            }


            unsigned int l = s.size();
            unsigned int extVal = 0;
            while(extVal < N || N==-1) {

                if(i>=l) {
                    return N==-1 ?  true :  false; // if we parse till the end, return true if we hit the end
                }
                // Skip white spaces
                while(s[i]==' ') {
                    i++;
                    if(i >= l) {
                        return N==-1 ?  true :  false; // if we parse till the end, return true if we hit the end
                    }
                }
                // determine range of valid character
                j=i;
                while(s[j]!=' ') {
                    j++;
                    if(j >= l) {
                        return N==-1 ?  true :  false; // if we parse till the end, return true if we hit the end
                    }
                }
                // extract substring and convert
                if(!stringToType(number,s.substr(i,j-i))) {
                    return false;
                }
                f(extVal,number);
                i = j;
            }
            return true;
    };

    /**
    * @brief Helper to convert a string with whitespace-seperated numbers into numbers in a vector (operator() needs to available).
    * N=-1, loops till the end of the string and extracts as many numbers as possible
    */
    template <unsigned int N, typename TVector>
    inline bool stringToVectorImpl( TVector & v, const std::string & s) {

        unsigned int i=0, j;
        using PREC = typename TVector::Scalar;
        PREC number;
        if(s.empty()) {
            return false;
        }

        unsigned int l = s.size();
        unsigned int extVal = 0;
        while(extVal < N || N==-1) {

            if(i>=l) {
                return false;
            }
            // Skip whit spaces
            while(s[i]==' ') {
                i++;
                if(i >= l) {
                    return false;
                }
            }
            // determine range of valid character
            j=i;
            while(s[j]!=' ') {
                j++;
                if(j >= l) {
                    break;
                }
            }
            // extract substring and convert
            if(!stringToType(number,s.substr(i,j-i))) {
                return false;
            }
            v(extVal) = number;
            i = j;
            extVal++;
        }
        return true;
    }


    template <typename PREC, typename Comp, typename Alloc>
    inline bool stringToStdSetImpl( std::set<PREC,Comp,Alloc> & m,
                                    const std::string & s) {
        std::function<void(unsigned int,PREC)> func = [&](unsigned int i, PREC n) {
            m.insert(n);
        };
        return stringToTypeFunctorImpl< -1, PREC, decltype(func) >(func,s);
    }

    template <typename PREC, typename Alloc>
    inline bool stringToStdVectorImpl( std::vector<PREC,Alloc> & v,
                                       const std::string & s) {
        auto func = [&](unsigned int i, PREC n) {
            v.push_back(n);
        };
        return stringToTypeFunctorImpl<-1, PREC, decltype(func) >(func,s);
    }

};


template<typename T>
bool stringToType(T & t, const std::string& s) {
    return details::stringToTypeImpl::convert(t,s);
}

/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a std::set<T>.
*/
template <typename T, typename Comp, typename Alloc>
bool stringToType( std::set<T,Comp,Alloc> & v, const std::string & s) {
    return details::stringToStdSetImpl(v,s);
}

/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a std::vector<T>.
*/
template <typename T,typename Alloc>
bool stringToType( std::vector<T,Alloc> & v, const std::string & s) {
    return details::stringToStdVectorImpl(v,s);
}

/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a std::vector<T>.
*/
template <typename T>
bool stringToType( std::pair<T,T> & p, const std::string & s) {
    typename MyMatrix<T>::Vector2 v;
    auto r = details::stringToVectorImpl<2>(v,s);
    p.first = v(0);
    p.second = v(1);
    return r;
}


/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a Vector2.
*/
template <typename TVector2>
bool stringToVector2( TVector2 & v, const std::string & s) {
    using PREC = typename TVector2::Scalar;
    STATIC_ASSERT2( (std::is_same< typename MyMatrix<PREC>::Vector2 , TVector2>::value) , "VECTOR_WRONG_TYPE" );
    return details::stringToVectorImpl<2>(v,s);
}

/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a Vector3.
*/
template <typename TVector3> bool stringToVector3( TVector3 & v, const std::string & s) {
    using PREC = typename TVector3::Scalar;
    STATIC_ASSERT2( (std::is_same< typename MyMatrix<PREC>::Vector3 , TVector3>::value), "VECTOR_WRONG_TYPE" );
    return details::stringToVectorImpl<3>(v,s);
}

/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a Vector4.
*/
template <typename TVector4> bool stringToVector4( TVector4 & v, const std::string & s) {
    using PREC = typename TVector4::Scalar;
    STATIC_ASSERT2( (std::is_same< typename MyMatrix<PREC>::Vector4 , TVector4>::value), "VECTOR_WRONG_TYPE" );
    return details::stringToVectorImpl<4>(v,s);
}



/**
* Generates count random value and returns the last one.
*/
template<typename PREC, typename Generator, typename Distribution>
inline PREC genRandomValues(PREC  value, Generator & g, Distribution & d, unsigned int count) {
    for(unsigned int i= 0; i<count; ++i) {
        value = d(g);
    }
    return value;
}


/**
* @brief This functions prints a std::vector, onliner :-)
*/
template<typename Iterator>
void printVector(std::ostream& ostr, const Iterator & itBegin, const Iterator & itEnd, const std::string& delimiter) {
    std::copy(itBegin, itEnd, std::ostream_iterator<typename Iterator::value_type>(ostr, delimiter.c_str()));
}
template<typename Stream, typename Iterator>
void printVectorNoCopy(Stream & ostr, const Iterator & itBegin, const Iterator & itEnd, const std::string& delimiter) {
    ostr << *itBegin;
    Iterator it = itBegin;
    for(++it; it != itEnd; ++it) {
        ostr << delimiter << *it;
    }
}


/**
* @brief Converts a std::vector with column vectors from Eigen into a Eigen Matrix.
*/
template <class PREC, std::size_t M, std::size_t N>
void vec2Mat(const std::vector<Eigen::Matrix<PREC,M,1> > &vec, Eigen::Matrix<PREC,M,N> &A) {
    for(int i=0; i<vec.size(); i++) {
        A.col(i) = vec[i];
    }
};

/**
* @brief Converts a std::vector with scalar values in it into a Eigen Vector.
*/
template <class PREC, std::size_t M>
void vec2Vec(const std::vector<PREC> &vec, Eigen::Matrix<PREC,M,1> &V) {
    for(int i=0; i<vec.size(); i++) {
        V[i] = vec[i];
    }
};


};

/** @} */

#endif

