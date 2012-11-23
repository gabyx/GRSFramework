//
//
//
//    Filename : AxisObject.cpp

#include "Ogre.h"
#include "OgreMaterial.h"
#include "AxisObject.hpp"

using namespace Ogre;

void AxisObject::addBox(ManualObject* obj, Vector3 dim, Vector3 pos, ColourValue color, short boxMask)
{
    if(!obj)
        return;

    obj->begin("Axis", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    dim/=2;

    Ogre::Real l = dim.x;
    Ogre::Real h = dim.y;
    Ogre::Real w = dim.z;

    obj->position(Ogre::Vector3(-l, h, w) + pos);
    obj->colour(color);
    obj->position(Ogre::Vector3(-l, -h, w) + pos);
    obj->colour(color);
    obj->position(Ogre::Vector3(l, -h, w) + pos);
    obj->colour(color);
    obj->position(Ogre::Vector3(l, h, w) + pos);

    obj->position(Ogre::Vector3(-l, h, -w) + pos);
    obj->colour(color);
    obj->position(Ogre::Vector3(-l, -h, -w) + pos);
    obj->colour(color);
    obj->position(Ogre::Vector3(l, -h, -w) + pos);
    obj->colour(color);
    obj->position(Ogre::Vector3(l, h, -w) + pos);

    // front back
    if(boxMask & BOX_FRONT)
        obj->quad(0, 1, 2, 3);
    if(boxMask & BOX_BACK)
        obj->quad(7, 6, 5, 4);

    // top bottom
    if(boxMask & BOX_TOP)
        obj->quad(0, 3, 7, 4);
    if(boxMask & BOX_BOT)
        obj->quad(2, 1, 5, 6);

    // end caps
    if(boxMask & BOX_RIGHT)
        obj->quad(1, 0, 4, 5);
    if(boxMask & BOX_LEFT)
        obj->quad(3, 2, 6, 7);

    obj->end();
}

void AxisObject::addMaterial(const Ogre::String &mat, const Ogre::ColourValue &clr, Ogre::SceneBlendType sbt)
{
    static int init=false;
    if(init)
        return;
    else
        init=true;

    Ogre::MaterialPtr matptr = Ogre::MaterialManager::getSingleton().create(mat, "General");
    matptr->setReceiveShadows(false);
    matptr->getTechnique(0)->setLightingEnabled(true);
    matptr->getTechnique(0)->getPass(0)->setDiffuse(clr);
    matptr->getTechnique(0)->getPass(0)->setAmbient(clr);
    matptr->getTechnique(0)->getPass(0)->setSelfIllumination(clr);
    matptr->getTechnique(0)->getPass(0)->setSceneBlending(sbt);
    matptr->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    matptr->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);
}

Ogre::ManualObject* AxisObject::createAxis(Ogre::SceneManager *scene, const Ogre::String &name, Ogre::Real scale)
{
    addMaterial("Axis", Ogre::ColourValue(1,1,1,.75), Ogre::SBT_TRANSPARENT_ALPHA);

    Ogre::ManualObject* axis    = scene->createManualObject(name);

    Ogre::Real len=scale;
    Ogre::Real scl=len*.1;
    Ogre::Real loc=len/2+scl/2;
    Ogre::Real fade=.5;
    Ogre::Real solid=.8;

    addBox(axis, Vector3(len, scl, scl), Vector3(loc,0,0), ColourValue(0, 0, solid, solid), (BOX_ALL & ~BOX_RIGHT));
    addBox(axis, Vector3(len, scl, scl), Vector3(-loc,0,0), ColourValue(0, 0, fade, fade), (BOX_ALL & ~BOX_LEFT));

    addBox(axis, Vector3(scl, len, scl), Vector3(0,loc,0), ColourValue(0, solid, 0, solid), (BOX_ALL & ~BOX_BOT));
    addBox(axis, Vector3(scl, len, scl), Vector3(0,-loc,0), ColourValue(0, fade, 0, fade), (BOX_ALL & ~BOX_TOP));

    addBox(axis, Vector3(scl, scl, len), Vector3(0,0,loc), ColourValue(solid, 0, 0, solid), (BOX_ALL & ~BOX_BACK));
    addBox(axis, Vector3(scl, scl, len), Vector3(0,0,-loc), ColourValue(fade, 0, 0, fade), (BOX_ALL & ~BOX_FRONT));

    axis->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);

    return axis;
}
