﻿#ifndef OgreSceneManagerDeleter_hpp
#define OgreSceneManagerDeleter_hpp

#include <OgreSceneManager.h>
#include "Contexts/RenderContext.hpp"

/** 
* @ingroup Common
* @defgroup OgreSceneManagerDeleter Functor for SceneManager deletion 
*/ 
/* @{ */

/** @brief This class is used as a functor for the boost::shared_ptr of type Ogre::SceneManager
*  which is deleted by a call to this function! (and not with a simple delete!).
*/
class OgreSceneManagerDeleter{
public:
/** @brief Functor which deletets the Ogre::SceneManager.
*/
	void operator()(Ogre::SceneManager * p){
		RenderContext::getSingletonPtr()->m_pRoot->destroySceneManager(p);
		p = NULL;
	};
};
/* @} */

#endif
