// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_OgrePointCloud_hpp
#define GRSF_common_OgrePointCloud_hpp

#include "GRSF/common/AssertionDebug.hpp"

#include <OgreSimpleRenderable.h>
#include <OgreMovableObject.h>
#include <OgreString.h>
#include <OgreAxisAlignedBox.h>
#include <OgreVector3.h>
#include <OgreMaterial.h>
#include <OgreColourValue.h>
#include <OgreRoot.h>
#include <OgreHardwareBufferManager.h>
#include <OgreSharedPtr.h>

#include <stdint.h>

#include <vector>
#include <memory>

namespace Ogre {
class SceneManager;
class ManualObject;
class SceneNode;
class RenderQueue;
class Camera;
class RenderSystem;
class Matrix4;
}

namespace OgreColorConversion {
inline uint32_t colorToHandle(Ogre::PixelFormat fmt, uint32_t col) {
    uint32_t handle = 0;
    if (fmt == Ogre::PF_A8R8G8B8 || fmt == Ogre::PF_X8R8G8B8) {
        handle = col & 0x00ffffff;
    } else if (fmt == Ogre::PF_R8G8B8A8) {
        handle = col >> 8;
    } else {
        ERRORMSG("Incompatible pixel format " << fmt);
    }
    return handle;
}


typedef uint32_t CollObjectHandle;
inline CollObjectHandle colorToHandle( const Ogre::ColourValue & color ) {
    return (int(color.r * 255) << 16) | (int(color.g * 255) << 8) | int(color.b * 255);
}

};


class OgrePointCloud;
class OgrePointCloudRenderable : public Ogre::SimpleRenderable {
public:
    OgrePointCloudRenderable(OgrePointCloud* parent, int num_points, bool use_tex_coords);
    ~OgrePointCloudRenderable();

    Ogre::RenderOperation* getRenderOperation() {
        return &mRenderOp;
    }
    Ogre::HardwareVertexBufferSharedPtr getBuffer();

    virtual Ogre::Real getBoundingRadius(void) const;
    virtual Ogre::Real getSquaredViewDepth(const Ogre::Camera* cam) const;
    virtual void _notifyCurrentCamera(Ogre::Camera* camera);
    virtual unsigned short getNumWorldTransforms() const {
        return 1;
    }
    virtual void getWorldTransforms(Ogre::Matrix4* xform) const;
    virtual const Ogre::LightList& getLights() const;

private:
    Ogre::MaterialPtr material_;
    OgrePointCloud* parent_;
};
typedef std::shared_ptr<OgrePointCloudRenderable> OgrePointCloudRenderablePtr;
typedef std::vector<OgrePointCloudRenderablePtr> V_OgrePointCloudRenderable;

/**
 * \class OgrePointCloud
 * \brief A visual representation of a set of points.
 *
 * Displays a set of points using any number of Ogre BillboardSets.  OgrePointCloud is optimized for sets of points that change
 * rapidly, rather than for large clouds that never change.
 *
 * Most of the functions in OgrePointCloud are not safe to call from any thread but the render thread.  Exceptions are clear() and addPoints(), which
 * are safe as long as we are not in the middle of a render (ie. Ogre::Root::renderOneFrame, or Ogre::RenderWindow::update)
 */
class OgrePointCloud : public Ogre::MovableObject {
public:
    enum RenderMode {
        RM_POINTS,
        RM_SQUARES,
        RM_FLAT_SQUARES,
        RM_SPHERES,
        RM_TILES,
        RM_BOXES,
    };

    OgrePointCloud();
    ~OgrePointCloud();

    /**
     * \brief Clear all the points
     */
    void clear();

    /**
     * \struct Point
     * \brief Representation of a point, with x/y/z position and r/g/b color
     */
    struct Point {
        inline void setColor(Ogre::Real r, Ogre::Real g, Ogre::Real b, Ogre::Real a=1.0) {
            color=Ogre::ColourValue(r, g, b, a);
        }

        Ogre::Vector3 position;
        Ogre::ColourValue color;
    };

    /**
     * \brief Add points to this point cloud
     *
     * @param points An array of Point structures
     * @param num_points The number of points in the array
     */
    void addPoints( Point* points, uint32_t num_points );

    /**
     * \brief Get point idx of this point cloud
     * This function to modify the points should only be called by the render thread!
     */
    Point & getPoint( uint32_t idx );

    /**
     * \brief Remove a number of points from this point cloud
     * \param num_points The number of points to pop
     */
    void popPoints( uint32_t num_points );

    /**
     * \brief Set what type of rendering primitives should be used, currently points, billboards and boxes are supported
     */
    void setRenderMode(RenderMode mode);
    /**
     * \brief Set the dimensions of the billboards used to render each point
     * @param width Width
     * @param height Height
     * @note width/height are only applicable to billboards and boxes, depth is only applicable to boxes
     */
    void setDimensions( Ogre::Real width, Ogre::Real height, Ogre::Real depth );

    /*
     * If set to true, the size of each point will be multiplied by it z component.
     * (Used for depth image based point clouds)
     */
    void setAutoSize(bool auto_size);

    /// See Ogre::BillboardSet::setCommonDirection
    void setCommonDirection( const Ogre::Vector3& vec );
    /// See Ogre::BillboardSet::setCommonUpVector
    void setCommonUpVector( const Ogre::Vector3& vec );

    /// set alpha blending
    /// @param alpha global alpha value
    /// @param per_point_alpha indicates that each point will have an individual alpha value.
    ///                        if true, enables alpha blending regardless of the global alpha.
    void setAlpha( Ogre::Real alpha, bool per_point_alpha = false );

    void setPickColor(const Ogre::ColourValue& color);
    void setColorByIndex(bool set);

    void setHighlightColor( Ogre::Real r, Ogre::Real g, Ogre::Real b );

    virtual const Ogre::String& getMovableType() const {
        return sm_Type;
    }
    virtual const Ogre::AxisAlignedBox& getBoundingBox() const;
    virtual Ogre::Real getBoundingRadius() const;
    virtual void getWorldTransforms( Ogre::Matrix4* xform ) const;
    virtual unsigned short getNumWorldTransforms() const {
        return 1;
    }
    virtual void _updateRenderQueue( Ogre::RenderQueue* queue );
    virtual void _notifyCurrentCamera( Ogre::Camera* camera );
    virtual void _notifyAttached(Ogre::Node *parent, bool isTagPoint=false);
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6)
    virtual void visitRenderables(Ogre::Renderable::Visitor* visitor, bool debugRenderables);
#endif

    virtual void setName ( const std::string& name ) {
        mName = name;
    }

private:

    uint32_t getVerticesPerPoint();
    OgrePointCloudRenderablePtr createRenderable( int num_points );
    void regenerateAll();
    void shrinkRenderables();

    Ogre::AxisAlignedBox bounding_box_;       ///< The bounding box of this point cloud
    Ogre::Real bounding_radius_;                   ///< The bounding radius of this point cloud

    typedef std::vector<Point> V_Point;
    V_Point points_;                          ///< The list of points we're displaying.  Allocates to a high-water-mark.
    uint32_t point_count_;                    ///< The number of points currently in #points_

    RenderMode render_mode_;
    Ogre::Real width_;                             ///< width
    Ogre::Real height_;                            ///< height
    Ogre::Real depth_;                             ///< depth
    Ogre::Vector3 common_direction_;          ///< See Ogre::BillboardSet::setCommonDirection
    Ogre::Vector3 common_up_vector_;          ///< See Ogre::BillboardSet::setCommonUpVector

    Ogre::MaterialPtr point_material_;
    Ogre::MaterialPtr square_material_;
    Ogre::MaterialPtr flat_square_material_;
    Ogre::MaterialPtr sphere_material_;
    Ogre::MaterialPtr tile_material_;
    Ogre::MaterialPtr box_material_;
    Ogre::MaterialPtr current_material_;
    Ogre::Real alpha_;

    bool color_by_index_;

    V_OgrePointCloudRenderable renderables_;

    bool current_mode_supports_geometry_shader_;
    Ogre::ColourValue pick_color_;

    static Ogre::String sm_Type;              ///< The "renderable type" used by Ogre
};

#endif
