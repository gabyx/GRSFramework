// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_AxisObject_hpp
#define GRSF_common_AxisObject_hpp

#include <Ogre.h>

class AxisObject
{
    enum BoxParts
    {
        BOX_NONE  = 0x00,
        BOX_TOP   = 0x01,
        BOX_BOT   = 0x02,
        BOX_FRONT = 0x04,
        BOX_BACK  = 0x08,
        BOX_LEFT  = 0x10,
        BOX_RIGHT = 0x20,
        BOX_ALL   = 0xFF
    };

private:
    void addMaterial(const Ogre::String& mat, const Ogre::ColourValue& clr, Ogre::SceneBlendType sbt);
    void addBox(Ogre::ManualObject* obj, Ogre::Vector3 dim, Ogre::Vector3 pos, Ogre::ColourValue color, short boxMask);

public:
    Ogre::ManualObject* createAxis(Ogre::SceneManager* scene, const Ogre::String& name, Ogre::Real scale);
};

#endif  //--_AXIS_OBJECT_H_
