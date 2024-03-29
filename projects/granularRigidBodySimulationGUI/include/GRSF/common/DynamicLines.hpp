// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_DynamicLines_hpp
#define GRSF_common_DynamicLines_hpp

/// Includes =================================
#include <Ogre.h>
#include <vector>
#include "GL/gl.h"

#include "GRSF/common/DynamicRenderable.hpp"
/// ==========================================

class DynamicLines : public DynamicRenderable
{
public:
    using Vector3       = Ogre::Vector3;
    using Quaternion    = Ogre::Quaternion;
    using Camera        = Ogre::Camera;
    using Real          = Ogre::Real;
    using OperationType = Ogre::RenderOperation::OperationType;

public:
    /// Constructor - see setOperationType() for description of argument.
    DynamicLines(OperationType opType = Ogre::RenderOperation::OT_LINE_STRIP);
    virtual ~DynamicLines();

    /// Add a point to the point list
    void addPoint(const Ogre::Vector3& p);
    /// Add a point to the point list
    void addPoint(Real x, Real y, Real z);

    /// Change the location of an existing point in the point list
    void setPoint(unsigned short index, const Vector3& value);

    /// Return the location of an existing point in the point list
    const Vector3& getPoint(unsigned short index) const;

    /// Return the total number of points in the point list
    unsigned short getNumPoints(void) const;

    /// Reserve n points
    void reserve(unsigned int n);

    /// Remove all points from the point list
    void clear();

    /// Call this to update the hardware buffer after making changes.
    void update();

    /** Set the type of operation to draw with.
   * @param opType Can be one of
   *    - RenderOperation::OT_LINE_STRIP
   *    - RenderOperation::OT_LINE_LIST
   *    - RenderOperation::OT_POINT_LIST
   *    - RenderOperation::OT_TRIANGLE_LIST
   *    - RenderOperation::OT_TRIANGLE_STRIP
   *    - RenderOperation::OT_TRIANGLE_FAN
   *    The default is OT_LINE_STRIP.
   */
    void setOperationType(OperationType opType);
    OperationType getOperationType() const;

protected:
    /// Implementation DynamicRenderable, creates a simple vertex-only decl
    virtual void createVertexDeclaration();
    /// Implementation DynamicRenderable, pushes point list out to hardware memory
    virtual void fillHardwareBuffers();

    //  bool preRender(Ogre::SceneManager *sm, Ogre::RenderSystem *rsys){
    //
    //  }
    //  void postRender(Ogre::SceneManager *sm, Ogre::RenderSystem *rsys){
    //
    //  }

private:
    using Vector3Vec = std::vector<Vector3>;
    Vector3Vec mPoints;
    bool mDirty;
};

#endif
