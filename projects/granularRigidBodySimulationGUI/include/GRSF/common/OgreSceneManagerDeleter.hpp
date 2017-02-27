// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_OgreSceneManagerDeleter_hpp
#define GRSF_common_OgreSceneManagerDeleter_hpp

#include <OgreSceneManager.h>
#include "GRSF/singeltons/contexts/RenderContext.hpp"

/**
* @ingroup Common
* @defgroup OgreSceneManagerDeleter Functor for SceneManager deletion
*/
/* @{ */

/** @brief This class is used as a functor for the boost::shared_ptr of type Ogre::SceneManager
*  which is deleted by a call to this function! (and not with a simple delete!).
*/
class OgreSceneManagerDeleter
{
public:
    /** @brief Functor which deletets the Ogre::SceneManager.
*/
    void operator()(Ogre::SceneManager* p)
    {
        RenderContext::getSingletonPtr()->m_pRoot->destroySceneManager(p);
        p = nullptr;
    };
};
/* @} */

#endif
