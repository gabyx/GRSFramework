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
#include <string>
#include <sstream>
#include <cmath>
#include <limits>
#include <vector>


#include "MyMatrixDefs.hpp"


namespace Utilities{

/**
* @ingroup Common
* @defgroup CommonFunctions Common Functions
* @brief
*/
/* @{ */




inline bool operator == (const std::string & a, const std::string & b)
{
	return    ( a.size() == b.size() )				// optimization on some platforms
	       && ( a.compare(b) == 0 );	            // actual compare
}

inline bool operator == (const std::string & a, const char * b){	return  strcmp(a.c_str(), b) == 0; }
inline bool operator == (const char* a, const std::string & b) {  return b == a; }

namespace details{

    template<typename T>
    struct stringToTypeImpl{
        /**
        * @brief This functions casts a string into another template specifix type.
        * @param t The ouput where the casted string is put into.
        * @param s The string to cast.
        * @param f The format (e.g std::dec).
        * @return true if conversion worked, false if not.
        */
        bool operator()(T& t, const std::string& s)
        {
              std::istringstream iss(s);
              return !(iss >> t).fail();
        }
    };

    template<>
    struct stringToTypeImpl< bool >
    {
        bool operator()(bool & t, const std::string& s)
        {
           int a;
           if( stringToTypeImpl<int>()(a, s)){
              if(a){
                 t = true;
                 return true;
              }else{
                 t = false;
                 return true;
              }
           }

           if( s == "true" || s =="True" || s=="TRUE"){
              t = true;
              return true;
           }
           else if( s == "false" || s =="False" || s=="FALSE"){
              t = false;
              return true;
           }

           t = false;
           return false;
        }
    };


// SAD that this spezialization does not work for typename MyMatrix<PREC>::Vector3
//    template<typename PREC>
//    struct stringToTypeImpl< Eigen::Matrix<PREC,2,1>  >{
//        bool operator()(typename MyMatrix<PREC>::Vector2 & t, const std::string& s)
//        {
//          stringToVector2<PREC>(t,s);
//        }
//    };
//
//
//    template<typename PREC>
//    struct stringToTypeImpl< typename MyMatrix<PREC>::Vector3  >{
//        bool operator()(typename MyMatrix<PREC>::Vector3 & t, const std::string& s)
//        {
//          stringToVector3<PREC>(t,s);
//        }
//    };
//    template<typename PREC>
//    struct stringToTypeImpl< typename MyMatrix<PREC>::Vector4  >{
//        bool operator()(typename MyMatrix<PREC>::Vector4 & t, const std::string& s)
//        {
//          stringToVector4<PREC>(t,s);
//        }
//    };

}


template<typename T> bool stringToType(T & t, const std::string& s){
   return details::stringToTypeImpl<T>()(t,s);
}



/**
* @brief This functions returns a uniformally distributed random number:
* @param low Lower bound of random number.
* @param high Upper bound of random number.
* @return Random number in range [low,high]
*/
double randd(double low, double high);


/**
* @brief This functions prints a std::vector, onliner :-)
*/
template<typename Iterator>
void printVector(std::ostream& ostr, const Iterator & itBegin, const Iterator & itEnd, const std::string& delimiter){
        std::copy(itBegin, itEnd, std::ostream_iterator<typename Iterator::value_type>(ostr, delimiter.c_str()));
}
template<typename Stream, typename Iterator>
void printVectorNoCopy(Stream & ostr, const Iterator & itBegin, const Iterator & itEnd, const std::string& delimiter){
        ostr << *itBegin;
        Iterator it = itBegin;
        for(it++; it != itEnd; it++){
            ostr << delimiter << *it;
        }
}


/**
* @brief Converts a std::vector with column vectors from Eigen into a Eigen Matrix.
*/
template <class PREC, std::size_t M, std::size_t N>
void vec2Mat(const std::vector<Eigen::Matrix<PREC,M,1> > &vec, Eigen::Matrix<PREC,M,N> &A){
    for(int i=0;i<vec.size();i++){
        A.col(i) = vec[i];
    }
};

/**
* @brief Converts a std::vector with scalar values in it into a Eigen Vector.
*/
template <class PREC, std::size_t M>
void vec2Vec(const std::vector<PREC> &vec, Eigen::Matrix<PREC,M,1> &V){
    for(int i=0;i<vec.size();i++){
		V[i] = vec[i];
    }
};

/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a Vector3.
*/
template <typename PREC> bool stringToVector2( typename MyMatrix<PREC>::Vector2 & vector2, const std::string & s){
	unsigned int i=0, j;
	PREC number;

   if( s.empty()){ return false;}

	while(s[i]==' ') {
		i++;
		if(i > s.length()-1){
			break;
		}
	}
	if (i==s.length()) {
		return false;
	}
	else {
		j=i;
		while(s[j]!=' ') {
			j++;
			if(j > s.length()-1){
				break;
			}
		}
		if (!stringToType<PREC>(number,s.substr(i,j-i))) {
			return false;
		}
		vector2(0) = number;////////////////////////x
		i = j;

		while(s[i]==' ') {
			i++;
			if(i > s.length()-1){
				break;
			}
		}
		if (i==s.length()) {
			return false;
		}
		else {
			j=i;
			while(s[j]!=' ') {
				j++;
				if(j > s.length()-1){
					break;
				}
			}
			if (!stringToType<PREC>(number,s.substr(i,j-i))){
				return false;
			}
			vector2(1) = number;////////////////////////y
		}
	}
	return true;
}




/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a Vector3.
*/
template <typename PREC> bool stringToVector3( typename MyMatrix<PREC>::Vector3 & vector3, const std::string & s){
	unsigned int i=0, j;
	PREC number;

   if( s.empty()){ return false;}

	while(s[i]==' ') {
		i++;
		if(i > s.length()-1){
			break;
		}
	}
	if (i==s.length()) {
		return false;
	}
	else {
		j=i;
		while(s[j]!=' ') {
			j++;
			if(j > s.length()-1){
				break;
			}
		}
		if (!stringToType<PREC>(number,s.substr(i,j-i))) {
			return false;
		}
		vector3(0) = number;////////////////////////x
		i = j;

		while(s[i]==' ') {
			i++;
			if(i > s.length()-1){
				break;
			}
		}
		if (i==s.length()) {
			return false;
		}
		else {
			j=i;
			while(s[j]!=' ') {
				j++;
				if(j > s.length()-1){
					break;
				}
			}
			if (!stringToType<PREC>(number,s.substr(i,j-i))){
				return false;
			}
			vector3(1) = number;////////////////////////y
			i = j;

			while(s[i]==' ') {
				i++;
				if(i > s.length()-1){
					break;
				}
			}
			if (i==s.length()) {
				return false;
			}
			else {
				j=i;
				while(s[j]!=' ') {
					j++;
					if(j > s.length()-1){
						break;
					}
				}
				if (!stringToType<PREC>(number,s.substr(i,j-i))) {
					return false;
				}
				vector3(2) = number;////////////////////////z

			}
		}
	}
	return true;
}




/**
* @brief Helper to convert a string with three whitespace-seperated numbers into a Vector4.
*/
template <class PREC> bool stringToVector4(typename MyMatrix<PREC>::Vector4 & vector4, const std::string & s){
	unsigned int i=0, j;
	PREC number;
   if( s.empty()){ return false;}
	while(s[i]==' ') {
		i++;
		if(i > s.length()-1){
			break;
		}
	}
	if (i==s.length()) {
		return false;
	}
	else {
		j=i;
		while(s[j]!=' ') {
			j++;
			if(j > s.length()-1){
				break;
			}
		}
		if (!stringToType<PREC>(number,s.substr(i,j-i))) {
			return false;
		}
		vector4(0) = number;////////////////////////x
		i = j;

		while(s[i]==' ') {
			i++;
			if(i > s.length()-1){
				break;
			}
		}
		if (i==s.length()) {
			return false;
		}
		else {
			j=i;
			while(s[j]!=' ') {
				j++;
				if(j > s.length()-1){
					break;
				}
			}
			if (!stringToType<PREC>(number,s.substr(i,j-i))){
				return false;
			}
			vector4(1) = number;////////////////////////y
			i = j;

			while(s[i]==' ') {
				i++;
				if(i > s.length()-1){
					break;
				}
			}
			if (i==s.length()) {
				return false;
			}
			else {
				j=i;
				while(s[j]!=' ') {
					j++;
					if(j > s.length()-1){
						break;
					}
				}
				if (!stringToType<PREC>(number,s.substr(i,j-i))) {
					return false;
				}
				vector4(2) = number;////////////////////////z
				i = j;

				while(s[i]==' ') {
					i++;
					if(i > s.length()-1){
						break;
					}
				}
				if (i==s.length()) {
					return false;
				}
				else {
					j=i;

					while(s[j]!=' ') {
						j++;
						if(j > s.length()-1){
							break;
						}
					}
					if (!stringToType<PREC>(number,s.substr(i,j-i))) {
						return false;
					}
					vector4(3) = number;////////////////////////w
				}
			}
		}
	}

	return true;
}

};

/** @} */

#endif
