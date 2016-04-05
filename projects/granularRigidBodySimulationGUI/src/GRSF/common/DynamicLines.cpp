// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/common/DynamicLines.hpp"

/// Includes =================================
#include <Ogre.h>
#include <cassert>
#include <cmath>
#include <type_traits>
/// ==========================================

using namespace Ogre;

enum {
  POSITION_BINDING = 0,
  TEXCOORD_BINDING = 1
};

DynamicLines::DynamicLines(OperationType opType)
{
  initialize(opType,false);
  setMaterial("BaseWhiteNoLighting");
  mDirty = true;
}

DynamicLines::~DynamicLines()
{
}

void DynamicLines::setOperationType(OperationType opType)
{
  mRenderOp.operationType = opType;
}

RenderOperation::OperationType DynamicLines::getOperationType() const
{
  return mRenderOp.operationType;
}

void DynamicLines::reserve(unsigned int n){
    mPoints.reserve(n);
}

void DynamicLines::addPoint(const Vector3 &p)
{
   mPoints.push_back(p);
   mDirty = true;
}
void DynamicLines::addPoint(Real x, Real y, Real z)
{
   mPoints.push_back(Vector3(x,y,z));
   mDirty = true;
}
const Vector3& DynamicLines::getPoint(unsigned short index) const
{
   assert(index < mPoints.size() && "Point index is out of bounds!!");
   return mPoints[index];
}
unsigned short DynamicLines::getNumPoints(void) const
{
  return (unsigned short)mPoints.size();
}
void DynamicLines::setPoint(unsigned short index, const Vector3 &value)
{
  assert(index < mPoints.size() && "Point index is out of bounds!!");

  mPoints[index] = value;
  mDirty = true;
}
void DynamicLines::clear()
{
  mPoints.clear();
  mDirty = true;
}

void DynamicLines::update()
{
  if (mDirty) fillHardwareBuffers();
}

void DynamicLines::createVertexDeclaration()
{
  VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration;
//  if( std::is_same<Real,double>::value){
//    decl->addElement(POSITION_BINDING, 0, Ogre::VET_DOUBLE3, Ogre::VES_POSITION);
//  }else{
    decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
//  }
}

void DynamicLines::fillHardwareBuffers()
{
  int size = mPoints.size();

  prepareHardwareBuffers(size,0);

  if (!size) {
    mBox.setExtents(Vector3::ZERO,Vector3::ZERO);
    mDirty=false;
    return;
  }

  Vector3 vaabMin = mPoints[0];
  Vector3 vaabMax = mPoints[0];

  HardwareVertexBufferSharedPtr vbuf =
    mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);

  float *prPos = static_cast<float*>(vbuf->lock(HardwareBuffer::HBL_DISCARD));
  {
   for(int i = 0; i < size; i++)
   {
      *(prPos++) = mPoints[i].x;
      *(prPos++) = mPoints[i].y;
      *(prPos++) = mPoints[i].z;

      if(mPoints[i].x < vaabMin.x)
         vaabMin.x = mPoints[i].x;
      if(mPoints[i].y < vaabMin.y)
         vaabMin.y = mPoints[i].y;
      if(mPoints[i].z < vaabMin.z)
         vaabMin.z = mPoints[i].z;

      if(mPoints[i].x > vaabMax.x)
         vaabMax.x = mPoints[i].x;
      if(mPoints[i].y > vaabMax.y)
         vaabMax.y = mPoints[i].y;
      if(mPoints[i].z > vaabMax.z)
         vaabMax.z = mPoints[i].z;
   }
  }
  vbuf->unlock();

  mBox.setExtents(vaabMin, vaabMax);

  mDirty = false;
}

/*
void DynamicLines::getWorldTransforms(Matrix4 *xform) const
{
   // return identity matrix to prevent parent transforms
   *xform = Matrix4::IDENTITY;
}
*/
/*
const Quaternion &DynamicLines::getWorldOrientation(void) const
{
   return Quaternion::IDENTITY;
}

const Vector3 &DynamicLines::getWorldPosition(void) const
{
   return Vector3::ZERO;
}
*/

