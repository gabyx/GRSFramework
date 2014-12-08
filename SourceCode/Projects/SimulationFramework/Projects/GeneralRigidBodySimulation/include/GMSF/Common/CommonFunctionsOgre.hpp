#ifndef GMSF_Common_CommonFunctionsOgre_hpp
#define GMSF_Common_CommonFunctionsOgre_hpp

#include <string>
#include <vector>

#include <OGRE/Ogre.h>

#include "MyMatrixDefs.hpp"
#include "AssertionDebug.hpp"


namespace OgreUtilities {


Ogre::StringVector convertToOgreStringVector(const std::vector<std::string> & vec);


/**
* @brief Converts a vector 3x1 from Ogre to an Eigen 3x1 Vector.
*/
template<typename PREC>
typename MyMatrix<PREC>::Vector3 vectorFromOgre(const Ogre::Vector3& v) {
    Eigen::Matrix<double,3,1> vec;
    vec(0) = v.x;
    vec(1) = v.y;
    vec(2) = v.z;
    return vec;
}


/**
* @brief Converts a quaternion 4x1 from Ogre to an Eigen 4x1 Vector.
*/
template<typename PREC>
typename MyMatrix<PREC>::Vector4 vectorFromOgre(const Ogre::Quaternion & v) {
    typename MyMatrix<PREC>::Vector4 vec;
    vec(0) = v.w;
    vec(1) = v.x;
    vec(2) = v.y;
    vec(3) = v.z;
    return vec;
}

};
#endif
