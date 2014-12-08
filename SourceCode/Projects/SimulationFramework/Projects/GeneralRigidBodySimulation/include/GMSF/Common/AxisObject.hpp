//
//
//
//    Filename : GMSF/Common/AxisObject.hpp

#ifndef GMSF_Common_AxisObject_hpp
#define GMSF_Common_AxisObject_hpp

class AxisObject
{
    enum BoxParts
    {
        BOX_NONE    = 0x00,
        BOX_TOP        = 0x01,
        BOX_BOT        = 0x02,
        BOX_FRONT    = 0x04,
        BOX_BACK    = 0x08,
        BOX_LEFT    = 0x10,
        BOX_RIGHT    = 0x20,
        BOX_ALL        = 0xFF
    };
private:
    void    addMaterial(const Ogre::String& mat, const Ogre::ColourValue &clr, Ogre::SceneBlendType sbt);
    void    addBox(Ogre::ManualObject* obj, Ogre::Vector3 dim, Ogre::Vector3 pos, Ogre::ColourValue color, short boxMask);
public:
    Ogre::ManualObject*createAxis(Ogre::SceneManager *scene, const Ogre::String &name, Ogre::Real scale);
};

#endif //--_AXIS_OBJECT_H_
