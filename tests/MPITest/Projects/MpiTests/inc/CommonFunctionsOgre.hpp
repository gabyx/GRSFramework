#ifndef OgreUtilities_hpp
#define OgreUtilities_hpp

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <OGRE/Ogre.h>

#include "AssertionDebug.hpp"

namespace OgreUtilities {


        Ogre::StringVector convertToOgreStringVector(const std::vector<std::string> & vec);


        /**
        * @brief Converts a vector 3x1 from Ogre to an Eigen 3x1 Vector.
        */
        template<typename PREC>
        Eigen::Matrix<PREC,3,1> vectorFromOgre(const Ogre::Vector3& v)
        {
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
        Eigen::Matrix<PREC,4,1> vectorFromOgre(const Ogre::Quaternion & v){
          Eigen::Matrix<PREC,4,1> vec;
	        vec(0) = v.w;
	        vec(1) = v.x;
	        vec(2) = v.y;
          vec(3) = v.z;
	        return vec;
        }

};
#endif
