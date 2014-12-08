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

#include <stdarg.h>  // for va_start, etc
#include <memory>    // for std::unique_ptr

#include "TinyFormatInclude.hpp"

#include "StaticAssert.hpp"
#include "AssertionDebug.hpp"
#include "MyMatrixDefs.hpp"
#include "FastStringConversion.hpp"

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


// Prototypes
struct StdTypeConverter{};
template<typename T, typename TypeConverter = StdTypeConverter> bool stringToType(T & t, const std::string& s);



namespace details {
    namespace stringToTypeImpl {


        /**
        * This is the standard type converter function
        * @brief This functions casts a string into the template specific type.
        * @param t The ouput where the casted string is put into.
        * @param s The string to cast.
        * @return true if conversion worked, false if not.
        */
        template<typename T,typename TypeConverter>
        inline
        typename std::enable_if< std::is_same<TypeConverter,StdTypeConverter>::value, bool>::type
        convert(T& t, const std::string& s) {
            //this is a huge times faster then the below stringstream stuff;
            return StringConversion::toType(t,s);
            //std::istringstream iss(s);
            //return !(iss >> t).fail();
        }
        /**
        * This is the custom type converter function, which takes the TypeConverter type to convert the string into the type
        */
        template<typename T,typename TypeConverter>
        inline
        typename std::enable_if< !std::is_same<TypeConverter,StdTypeConverter>::value, bool>::type
        convert(T& t, const std::string& s) {
            return TypeConverter::convert(t,s);
        }
    };


    /**
    * @brief Helper to convert a string with whitespace-seperated numbers into numbers and passes it to the Functor.
    * N=-1, loops till the end of the string and extracts as many numbers as possible
    */
    template <int N, typename PREC, typename Functor, typename TypeConverter>
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
                    return (N==-1) ?  true :  false; // if we parse till the end, return true if we hit the end
                }
                // Skip white spaces
                while(s[i]==' ') {
                    i++;
                    if(i >= l) {
                        return (N==-1) ?  true :  false; // if we parse till the end, return true if we hit the end
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
                if(!stringToType<PREC,TypeConverter>(number,s.substr(i,j-i))) {
                    return false;
                }
                //std::cout << "number:" << number << std::endl;
                f(extVal,number);
                i = j;
                extVal++;
            }
            return true;
    };

    /**
    * @brief Helper to convert a string with whitespace-seperated numbers into numbers in a vector (operator() needs to available).
    * N=-1, loops till the end of the string and extracts as many numbers as possible
    */
    template <unsigned int N, typename TVector, typename TypeConverter = StdTypeConverter>
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
                return (N==-1) ?  true :  false; // if we parse till the end, return true if we hit the end
            }
            // Skip whit spaces
            while(s[i]==' ') {
                i++;
                if(i >= l) {
                    return (N==-1) ?  true :  false; // if we parse till the end, return true if we hit the end
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
            if(!stringToType<PREC,TypeConverter>(number,s.substr(i,j-i))) {
                return false;
            }
            v(extVal) = number;
            i = j;
            extVal++;
        }
        return true;
    }

    /** Standart dispatch for single types: double,floats, int*/
    template <typename TypeConverter, typename T>
    inline bool stringToTypeDispatch( T & t, const std::string & s){
        return stringToTypeImpl::convert<T,TypeConverter>(t,s);
    }

    /** Custom dispatch for container types: std::set<T> */
    template <typename TypeConverter, typename T, typename Comp, typename Alloc>
    inline bool stringToTypeDispatch( std::set<T,Comp,Alloc> & m,
                                    const std::string & s) {
        std::function<void(unsigned int,T)> func = [&](unsigned int i, T n) {
            m.insert(n);
        };
        return stringToTypeFunctorImpl< -1, T, decltype(func),TypeConverter >(func,s);
    }
    /** Custom dispatch for container types: std::vector<T> */
    template <typename TypeConverter, typename T, typename Alloc>
    inline bool stringToTypeDispatch( std::vector<T,Alloc> & v,
                                       const std::string & s) {
        auto func = [&](unsigned int i, T n) {
            v.push_back(n);
        };
        return stringToTypeFunctorImpl<-1, T, decltype(func), TypeConverter >(func,s);
    }

     /** Custom dispatch for container types: std::pair<T> */
    template <typename TypeConverter, typename T>
    inline bool stringToTypeDispatch( std::pair<T,T> & v,
                                       const std::string & s) {
        auto func = [&](unsigned int i, T n) {
            i==0? v.first = n: v.second = n;
        };
        return stringToTypeFunctorImpl<2, T, decltype(func), TypeConverter >(func,s);
    }

};

/**
* This functor extracts two comma-seperated values (nowhitespaces) , e.g "3,1" and converts it
*/
template<typename T, typename THalf>
struct CommaSeperatedPairBinShift{
    STATIC_ASSERTM( ( sizeof(T) / sizeof(THalf) == 2) , "THalf should contain half the bytes of T");
    STATIC_ASSERTM( std::is_integral<T>::value && std::is_integral<T>::value, "Needs to be integral")
    inline static bool convert(T& t, const std::string& s){
//        std::cout << "convert format:" <<std::endl;
        if(s.empty()) { return false;}
        t = 0;
        THalf temp;
        unsigned int l = s.size();

        // search the comma
        unsigned int i = s.find(',');
        if(i==0 || i == l-1){ return false;} // ',' is at the beginning!
        //extract first number
        if(!details::stringToTypeImpl::convert<THalf,StdTypeConverter>(temp,s.substr(0,i))) {
            return false;
        }
        //
        t |= static_cast<T>(temp); t <<= sizeof(THalf)*8;

        //exctract second number
        if(!details::stringToTypeImpl::convert<THalf,StdTypeConverter>(temp,s.substr(i+1,std::string::npos))) {
            return false;
        }
        t |= static_cast<T>(temp);
        return true;
    }
};

template<typename T, typename TypeConverter>
inline bool stringToType(T & t, const std::string& s) {
    return details::stringToTypeDispatch<TypeConverter>(t,s);
}

/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a Vector2.
*/
template <typename TVector2>
inline bool stringToVector2( TVector2 & v, const std::string & s) {
    using PREC = typename TVector2::Scalar;
    //STATIC_ASSERTM( (std::is_same< typename MyMatrix<PREC>::Vector2 , TVector2>::value) , "VECTOR_WRONG_TYPE" );
    return details::stringToVectorImpl<2>(v,s);
}

/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a Vector3.
*/
template <typename TVector3>
inline bool stringToVector3( TVector3 & v, const std::string & s) {
    using PREC = typename TVector3::Scalar;
    //STATIC_ASSERTM( (std::is_same< typename MyMatrix<PREC>::Vector3 , TVector3>::value), "VECTOR_WRONG_TYPE" );
    return details::stringToVectorImpl<3>(v,s);
}

/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a Vector4.
*/
template <typename TVector4>
inline bool stringToVector4( TVector4 & v, const std::string & s) {
    using PREC = typename TVector4::Scalar;
    //STATIC_ASSERTM( (std::is_same< typename MyMatrix<PREC>::Vector4 , TVector4>::value), "VECTOR_WRONG_TYPE" );
    return details::stringToVectorImpl<4>(v,s);
}


/**
* @brief Convert Type to string if ostream operator << is supported
*/
template<typename T>
std::string  typeToString(const T & t){
    std::stringstream ss;
    ss << t;
    return ss.str(); // return copy of the string;
}



/**
* Generates count random value and returns the last one.
*/
template<typename PREC, typename Generator, typename Distribution, typename Integral>
inline PREC genRandomValues(PREC value, Generator & g, Distribution & d,Integral count) {
    for(unsigned int i= 0; i<count; ++i) {
        value = d(g);
    }
    return value;
}

/**
* Generates count random vectors and returns the last one.
*/
template<typename PREC, typename Generator, typename Distribution, typename Integral>
inline typename MyMatrix<PREC>::Vector3 genRandomVec(typename MyMatrix<PREC>::Vector3 value, Generator & g, Distribution & d, Integral count) {
    for(unsigned int i= 0; i<count; ++i) {
        value(0) = d(g);
        value(1) = d(g);
        value(2) = d(g);
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
void vec2Mat(const typename MyMatrix<PREC>::template StdVecAligned<Eigen::Matrix<PREC,M,1> > &vec,
             Eigen::Matrix<PREC,M,N> &A) {
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


template<typename Derived>
inline bool isFinite(const Eigen::MatrixBase<Derived>& x)
{
   return ( (x - x).array() == (x - x).array()).all();
}


template<typename Derived>
inline bool isNaN(const Eigen::MatrixBase<Derived>& x)
{
   return ((x.array() == x.array())).all();
}





    template<typename... Args>
    std::string stringFormat(const std::string fmt_str, const Args&... args) {

    /** http://stackoverflow.com/a/8098080/293195 */
    //    int final_n, n = ((int)fmt_str.size()) * 2; /* reserve 2 times as much as the length of the fmt_str */
    //    std::string str;
    //    std::unique_ptr<char[]> formatted;
    //    va_list ap;
    //    while(1) {
    //        formatted.reset(new char[n]); /* wrap the plain char array into the unique_ptr */
    //        strcpy(&formatted[0], fmt_str.c_str());
    //        va_start(ap, fmt_str);
    //        final_n = vsnprintf(&formatted[0], n, fmt_str.c_str(), ap);
    //        va_end(ap);
    //        if (final_n < 0 || final_n >= n)
    //            n += abs(final_n - n + 1);
    //        else
    //            break;
    //    }
    //    return std::string(formatted.get());

          return tfm::format(fmt_str.c_str(), args...);
    }


};

/** @} */

#endif

