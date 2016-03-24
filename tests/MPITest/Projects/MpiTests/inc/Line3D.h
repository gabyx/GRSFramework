// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef Line3D_h
#define Line3D_h

/// Includes =================================
#include <Ogre.h>
#include <vector>
///===========================================


using namespace Ogre;
using namespace std;

#define POSITION_BINDING 0
#define TEXCOORD_BINDING 1

class Line3D:public SimpleRenderable
{
public:
   Line3D(void);
   ~Line3D(void);

   void addPoint(const Vector3 &p);
   const Vector3 &getPoint(unsigned short index) const;
   unsigned short getNumPoints(void) const;
   void updatePoint(unsigned short index, const Vector3 &value);
   void drawLine(Vector3 &start, Vector3 &end);
   void drawLines(void);

   Real getSquaredViewDepth(const Camera *cam) const;
   Real getBoundingRadius(void) const;
protected:
   //void getWorldTransforms(Matrix4 *xform) const;
   const Quaternion &getWorldOrientation(void) const;
   const Vector3 &getWorldPosition(void) const;

   std::vector<Vector3> mPoints;
   bool mDrawn;
};

#endif /* __LINE3D_H__ */

