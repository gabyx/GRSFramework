/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "GRSF/Common/OgrePointCloud.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreBillboardSet.h>
#include <OgreBillboard.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>

#include <sstream>

#define SIZE_PARAMETER 0
#define ALPHA_PARAMETER 1
#define PICK_COLOR_PARAMETER 2
#define NORMAL_PARAMETER 3
#define UP_PARAMETER 4
#define HIGHLIGHT_PARAMETER 5
#define AUTO_SIZE_PARAMETER 6

#define VERTEX_BUFFER_CAPACITY (36 * 1024 * 10)


static float g_point_vertices[3] =
{
  0.0f, 0.0f, 0.0f
};

static float g_billboard_vertices[6*3] =
{
  -0.5f, 0.5f, 0.0f,
  -0.5f, -0.5f, 0.0f,
  0.5f, 0.5f, 0.0f,
  0.5f, 0.5f, 0.0f,
  -0.5f, -0.5f, 0.0f,
  0.5f, -0.5f, 0.0f,
};

static float g_billboard_sphere_vertices[3*3] =
{
  0.0f, 1.0f, 0.0f,
  -0.866025404f, -0.5f, 0.0f,
  0.866025404f, -0.5f, 0.0f,
};

static float g_box_vertices[6*6*3] =
{
  // front
  -0.5f, 0.5f, -0.5f,
  -0.5f, -0.5f, -0.5f,
  0.5f, 0.5f, -0.5f,
  0.5f, 0.5f, -0.5f,
  -0.5f, -0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,

  // back
  -0.5f, 0.5f, 0.5f,
  0.5f, 0.5f, 0.5f,
  -0.5f, -0.5f, 0.5f,
  0.5f, 0.5f, 0.5f,
  0.5f, -0.5f, 0.5f,
  -0.5f, -0.5f, 0.5f,

  // right
  0.5, 0.5, 0.5,
  0.5, 0.5, -0.5,
  0.5, -0.5, 0.5,
  0.5, 0.5, -0.5,
  0.5, -0.5, -0.5,
  0.5, -0.5, 0.5,

  // left
  -0.5, 0.5, 0.5,
  -0.5, -0.5, 0.5,
  -0.5, 0.5, -0.5,
  -0.5, 0.5, -0.5,
  -0.5, -0.5, 0.5,
  -0.5, -0.5, -0.5,

  // top
  -0.5, 0.5, -0.5,
  0.5, 0.5, -0.5,
  -0.5, 0.5, 0.5,
  0.5, 0.5, -0.5,
  0.5, 0.5, 0.5,
  -0.5, 0.5, 0.5,

  // bottom
  -0.5, -0.5, -0.5,
  -0.5, -0.5, 0.5,
  0.5, -0.5, -0.5,
  0.5, -0.5, -0.5,
  -0.5, -0.5, 0.5,
  0.5, -0.5, 0.5,
};

Ogre::String OgrePointCloud::sm_Type = "OgrePointCloud";

OgrePointCloud::OgrePointCloud()
: bounding_radius_( 0.0f )
, point_count_( 0 )
, common_direction_( Ogre::Vector3::NEGATIVE_UNIT_Z )
, common_up_vector_( Ogre::Vector3::UNIT_Y )
, color_by_index_(false)
, current_mode_supports_geometry_shader_(false)
{
  std::stringstream ss;
  static int count = 0;
  ss << "OgrePointCloudMaterial" << count++;
  point_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudPoint");
  square_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudSquare");
  flat_square_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudFlatSquare");
  sphere_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudSphere");
  tile_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudTile");
  box_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudBox");

  point_material_ = Ogre::MaterialPtr(point_material_)->clone(ss.str() + "Point");
  square_material_ = Ogre::MaterialPtr(square_material_)->clone(ss.str() + "Square");
  flat_square_material_ = Ogre::MaterialPtr(flat_square_material_)->clone(ss.str() + "FlatSquare");
  sphere_material_ = Ogre::MaterialPtr(sphere_material_)->clone(ss.str() + "Sphere");
  tile_material_ = Ogre::MaterialPtr(tile_material_)->clone(ss.str() + "Tiles");
  box_material_ = Ogre::MaterialPtr(box_material_)->clone(ss.str() + "Box");

  point_material_->load();
  square_material_->load();
  flat_square_material_->load();
  sphere_material_->load();
  tile_material_->load();
  box_material_->load();

  setAlpha( 1.0f );
  setRenderMode(RM_SPHERES);
  setDimensions(0.01f, 0.01f, 0.01f);

  clear();
}

static void removeMaterial(Ogre::MaterialPtr& material)
{
  Ogre::ResourcePtr resource(material);
  Ogre::MaterialManager::getSingleton().remove(resource);
}

OgrePointCloud::~OgrePointCloud()
{
  clear();

  point_material_->unload();
  square_material_->unload();
  flat_square_material_->unload();
  sphere_material_->unload();
  tile_material_->unload();
  box_material_->unload();

  removeMaterial(point_material_);
  removeMaterial(square_material_);
  removeMaterial(flat_square_material_);
  removeMaterial(sphere_material_);
  removeMaterial(tile_material_);
  removeMaterial(box_material_);
}

const Ogre::AxisAlignedBox& OgrePointCloud::getBoundingBox() const
{
  return bounding_box_;
}

Ogre::Real OgrePointCloud::getBoundingRadius() const
{
  return bounding_radius_;
}

void OgrePointCloud::getWorldTransforms(Ogre::Matrix4* xform) const
{
  *xform = _getParentNodeFullTransform();
}

void OgrePointCloud::clear()
{
  point_count_ = 0;
  bounding_box_.setNull();
  bounding_radius_ = 0.0f;

  if (getParentSceneNode())
  {
    V_OgrePointCloudRenderable::iterator it = renderables_.begin();
    V_OgrePointCloudRenderable::iterator end = renderables_.end();
    for (; it != end; ++it)
    {
      getParentSceneNode()->detachObject(it->get());
    }
    getParentSceneNode()->needUpdate();
  }

  renderables_.clear();
}

void OgrePointCloud::regenerateAll()
{
  if (point_count_ == 0)
  {
    return;
  }

  V_Point points;
  points.swap(points_);
  uint32_t count = point_count_;

  clear();

  addPoints(&points.front(), count);
}

void OgrePointCloud::setColorByIndex(bool set)
{
  color_by_index_ = set;
  regenerateAll();
}

void OgrePointCloud::setHighlightColor( Ogre::Real r, Ogre::Real g, Ogre::Real b )
{
  Ogre::Vector4 highlight( r, g, b, 0.0f );

  V_OgrePointCloudRenderable::iterator it = renderables_.begin();
  V_OgrePointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(HIGHLIGHT_PARAMETER, highlight);
  }
}

void OgrePointCloud::setRenderMode(RenderMode mode)
{
  render_mode_ = mode;

  if (mode == RM_POINTS)
  {
    current_material_ = Ogre::MaterialPtr(point_material_);
  }
  else if (mode == RM_SQUARES)
  {
    current_material_ = Ogre::MaterialPtr(square_material_);
  }
  else if (mode == RM_FLAT_SQUARES)
  {
    current_material_ = Ogre::MaterialPtr(flat_square_material_);
  }
  else if (mode == RM_SPHERES)
  {
    current_material_ = Ogre::MaterialPtr(sphere_material_);
  }
  else if (mode == RM_TILES)
  {
    current_material_ = Ogre::MaterialPtr(tile_material_);
  }
  else if (mode == RM_BOXES)
  {
    current_material_ = Ogre::MaterialPtr(box_material_);
  }

  current_material_->load();

  //ROS_INFO("Best technique [%s] [gp=%s]", current_material_->getBestTechnique()->getName().c_str(), current_material_->getBestTechnique()->getPass(0)->getGeometryProgramName().c_str());

  bool geom_support_changed = false;
  Ogre::Technique* best = current_material_->getBestTechnique();
  if (best)
  {
    if (current_material_->getBestTechnique()->getName() == "gp")
    {
      if (!current_mode_supports_geometry_shader_)
      {
        geom_support_changed = true;
      }

      current_mode_supports_geometry_shader_ = true;

      //ROS_INFO("Using geometry shader");
    }
    else
    {
      if (current_mode_supports_geometry_shader_)
      {
        geom_support_changed = true;
      }

      current_mode_supports_geometry_shader_ = false;
    }
  }
  else
  {
    geom_support_changed = true;
    current_mode_supports_geometry_shader_ = false;

    ERRORMSG("No techniques available for material " << current_material_->getName().c_str());
  }

  if (geom_support_changed)
  {
    renderables_.clear();
  }

  V_OgrePointCloudRenderable::iterator it = renderables_.begin();
  V_OgrePointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setMaterial(current_material_->getName());
  }

  regenerateAll();
}

void OgrePointCloud::setDimensions(Ogre::Real width, Ogre::Real height, Ogre::Real depth)
{
  width_ = width;
  height_ = height;
  depth_ = depth;

  Ogre::Vector4 size(width_, height_, depth_, 0.0f);

  V_OgrePointCloudRenderable::iterator it = renderables_.begin();
  V_OgrePointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(SIZE_PARAMETER, size);
  }
}

void OgrePointCloud::setAutoSize(bool auto_size)
{
  V_OgrePointCloudRenderable::iterator it = renderables_.begin();
  V_OgrePointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(AUTO_SIZE_PARAMETER, Ogre::Vector4(auto_size));
  }
}

void OgrePointCloud::setCommonDirection(const Ogre::Vector3& vec)
{
  common_direction_ = vec;

  V_OgrePointCloudRenderable::iterator it = renderables_.begin();
  V_OgrePointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(NORMAL_PARAMETER, Ogre::Vector4(vec));
  }
}

void OgrePointCloud::setCommonUpVector(const Ogre::Vector3& vec)
{
  common_up_vector_ = vec;

  V_OgrePointCloudRenderable::iterator it = renderables_.begin();
  V_OgrePointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(UP_PARAMETER, Ogre::Vector4(vec));
  }
}

void setAlphaBlending(const Ogre::MaterialPtr& mat)
{
  if (mat->getBestTechnique())
  {
    mat->getBestTechnique()->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    mat->getBestTechnique()->setDepthWriteEnabled( false );
  }
}

void setReplace(const Ogre::MaterialPtr& mat)
{
  if (mat->getBestTechnique())
  {
    mat->getBestTechnique()->setSceneBlending( Ogre::SBT_REPLACE );
    mat->getBestTechnique()->setDepthWriteEnabled( true );
  }
}

void OgrePointCloud::setAlpha(Ogre::Real alpha, bool per_point_alpha)
{
  alpha_ = alpha;

  if ( alpha < 0.9998 || per_point_alpha )
  {
    setAlphaBlending(point_material_);
    setAlphaBlending(square_material_);
    setAlphaBlending(flat_square_material_);
    setAlphaBlending(sphere_material_);
    setAlphaBlending(tile_material_);
    setAlphaBlending(box_material_);
  }
  else
  {
    setReplace(point_material_);
    setReplace(square_material_);
    setReplace(flat_square_material_);
    setReplace(sphere_material_);
    setReplace(tile_material_);
    setReplace(box_material_);
  }

  Ogre::Vector4 alpha4(alpha_, alpha_, alpha_, alpha_);
  V_OgrePointCloudRenderable::iterator it = renderables_.begin();
  V_OgrePointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(ALPHA_PARAMETER, alpha4);
  }
}

void OgrePointCloud::addPoints(Point* points, uint32_t num_points)
{
  if (num_points == 0)
  {
    return;
  }
  Ogre::Root* root = Ogre::Root::getSingletonPtr();

  if ( points_.size() < point_count_ + num_points )
  {
    points_.resize( point_count_ + num_points );
  }

  Point* begin = &points_.front() + point_count_;
  memcpy( begin, points, sizeof( Point ) * num_points );

  uint32_t vpp = getVerticesPerPoint();
  Ogre::RenderOperation::OperationType op_type;
  if (current_mode_supports_geometry_shader_)
  {
    op_type = Ogre::RenderOperation::OT_POINT_LIST;
  }
  else
  {
    if (render_mode_ == RM_POINTS)
    {
      op_type = Ogre::RenderOperation::OT_POINT_LIST;
    }
    else
    {
      op_type = Ogre::RenderOperation::OT_TRIANGLE_LIST;
    }
  }

  float* vertices = 0;
  if (current_mode_supports_geometry_shader_)
  {
    vertices = g_point_vertices;
  }
  else
  {
    if (render_mode_ == RM_POINTS)
    {
      vertices = g_point_vertices;
    }
    else if (render_mode_ == RM_SQUARES)
    {
      vertices = g_billboard_vertices;
    }
    else if (render_mode_ == RM_FLAT_SQUARES)
    {
      vertices = g_billboard_vertices;
    }
    else if (render_mode_ == RM_SPHERES)
    {
      vertices = g_billboard_sphere_vertices;
    }
    else if (render_mode_ == RM_TILES)
    {
      vertices = g_billboard_vertices;
    }
    else if (render_mode_ == RM_BOXES)
    {
      vertices = g_box_vertices;
    }
  }

  OgrePointCloudRenderablePtr rend;
  Ogre::HardwareVertexBufferSharedPtr vbuf;
  void* vdata = 0;
  Ogre::RenderOperation* op = 0;
  float* fptr = 0;

  Ogre::AxisAlignedBox aabb;
  aabb.setNull();
  uint32_t current_vertex_count = 0;
  bounding_radius_ = 0.0f;
  uint32_t vertex_size = 0;
  uint32_t buffer_size = 0;
  for (uint32_t current_point = 0; current_point < num_points; ++current_point)
  {
    // if we didn't create a renderable yet,
    // or we've reached the vertex limit for the current renderable,
    // create a new one.
    while (!rend || current_vertex_count >= buffer_size)
    {
      if (rend)
      {
        ASSERTMSG(current_vertex_count == buffer_size ,"");

        op->vertexData->vertexCount = rend->getBuffer()->getNumVertices() - op->vertexData->vertexStart;
        ASSERTMSG((op->vertexData->vertexCount + op->vertexData->vertexStart <= rend->getBuffer()->getNumVertices()),"");
        vbuf->unlock();
        rend->setBoundingBox(aabb);
        bounding_box_.merge(aabb);
      }

      buffer_size = std::min<int>( VERTEX_BUFFER_CAPACITY, (num_points - current_point)*vpp );

      rend = createRenderable( buffer_size );
      vbuf = rend->getBuffer();
      vdata = vbuf->lock(Ogre::HardwareBuffer::HBL_NO_OVERWRITE);

      op = rend->getRenderOperation();
      op->operationType = op_type;
      current_vertex_count = 0;

      vertex_size = op->vertexData->vertexDeclaration->getVertexSize(0);
      fptr = (float*)((uint8_t*)vdata);

      aabb.setNull();
    }

    const Point& p = points[current_point];

    uint32_t color;

    if (color_by_index_)
    {
      // convert to ColourValue, so we can then convert to the rendersystem-specific color type
      color = (current_point + point_count_ + 1);
      Ogre::ColourValue c;
      c.a = 1.0f;
      c.r = ((color >> 16) & 0xff) / 255.0f;
      c.g = ((color >> 8) & 0xff) / 255.0f;
      c.b = (color & 0xff) / 255.0f;
      root->convertColourValue(c, &color);
    }
    else
    {
      root->convertColourValue( p.color, &color );
    }

    aabb.merge(p.position);
    bounding_radius_ = std::max( bounding_radius_, p.position.squaredLength() );

    float x = p.position.x;
    float y = p.position.y;
    float z = p.position.z;

    for (uint32_t j = 0; j < vpp; ++j, ++current_vertex_count)
    {
      *fptr++ = x;
      *fptr++ = y;
      *fptr++ = z;

      if (!current_mode_supports_geometry_shader_)
      {
        *fptr++ = vertices[(j*3)];
        *fptr++ = vertices[(j*3) + 1];
        *fptr++ = vertices[(j*3) + 2];
      }

      uint32_t* iptr = (uint32_t*)fptr;
      *iptr = color;
      ++fptr;

      ASSERTMSG((uint8_t*)fptr <= (uint8_t*)vdata + rend->getBuffer()->getNumVertices() * vertex_size,"");
    }
  }

  op->vertexData->vertexCount = current_vertex_count - op->vertexData->vertexStart;
  rend->setBoundingBox(aabb);
  bounding_box_.merge(aabb);
  ASSERTMSG(op->vertexData->vertexCount + op->vertexData->vertexStart <= rend->getBuffer()->getNumVertices(),"");

  vbuf->unlock();

  point_count_ += num_points;

  shrinkRenderables();

  if (getParentSceneNode())
  {
    getParentSceneNode()->needUpdate();
  }
}

OgrePointCloud::Point & OgrePointCloud::getPoint( uint32_t idx){
        return points_[idx];
}

void OgrePointCloud::popPoints(uint32_t num_points)
{
  uint32_t vpp = getVerticesPerPoint();

  ASSERTMSG(num_points <= point_count_,"");
  points_.erase(points_.begin(), points_.begin() + num_points);

  point_count_ -= num_points;

  // Now clear out popped points
  uint32_t popped_count = 0;
  while (popped_count < num_points * vpp)
  {
    OgrePointCloudRenderablePtr rend = renderables_.front();
    Ogre::RenderOperation* op = rend->getRenderOperation();

    uint32_t popped = std::min((size_t)(num_points * vpp - popped_count), op->vertexData->vertexCount);
    op->vertexData->vertexStart += popped;
    op->vertexData->vertexCount -= popped;

    popped_count += popped;

    if (op->vertexData->vertexCount == 0)
    {
      renderables_.erase(renderables_.begin(), renderables_.begin() + 1);

      op->vertexData->vertexStart = 0;
      renderables_.push_back(rend);
    }
  }
  ASSERTMSG(popped_count == num_points * vpp,"");

  // reset bounds
  bounding_box_.setNull();
  bounding_radius_ = 0.0f;
  for (uint32_t i = 0; i < point_count_; ++i)
  {
    Point& p = points_[i];
    bounding_box_.merge(p.position);
    bounding_radius_ = std::max(bounding_radius_, p.position.squaredLength());
  }

  shrinkRenderables();

  if (getParentSceneNode())
  {
    getParentSceneNode()->needUpdate();
  }
}

void OgrePointCloud::shrinkRenderables()
{
  while (!renderables_.empty())
  {
    OgrePointCloudRenderablePtr rend = renderables_.back();
    Ogre::RenderOperation* op = rend->getRenderOperation();
    if (op->vertexData->vertexCount == 0)
    {
      renderables_.pop_back();
    }
    else
    {
      break;
    }
  }
}

void OgrePointCloud::_notifyCurrentCamera(Ogre::Camera* camera)
{
  MovableObject::_notifyCurrentCamera( camera );
}

void OgrePointCloud::_updateRenderQueue(Ogre::RenderQueue* queue)
{
  Ogre::Root* root;
  if(!color_by_index_){
    root = Ogre::Root::getSingletonPtr();
  }
  Ogre::AxisAlignedBox aabb;
  bounding_box_.setNull();

  Ogre::HardwareVertexBufferSharedPtr vbuf;
  void* vdata = 0;
  Ogre::RenderOperation* op = 0;
  float* fptr = 0;
  uint32_t current_point = 0;
  uint32_t vpp = getVerticesPerPoint();
  // uint32_t size = points_.size();


  V_OgrePointCloudRenderable::iterator it = renderables_.begin();
  V_OgrePointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    // update points in vertex buffer
    vbuf = (*it)->getBuffer();
    op = (*it)->getRenderOperation();
    aabb.setNull();

    // Lock buffer
    vdata = vbuf->lock(Ogre::HardwareBuffer::HBL_NO_OVERWRITE);
    fptr = (float*)((uint8_t*)vdata);

    uint32_t vertex_count= op->vertexData->vertexCount;
    uint32_t vertex_idx = 0;
    while( vertex_idx < vertex_count){

        const Point& p = points_[current_point];

        aabb.merge(p.position);

        float x = p.position.x;
        float y = p.position.y;
        float z = p.position.z;

        for (uint32_t j = 0; j < vpp; ++j)
        {
            *fptr++ = x;
            *fptr++ = y;
            *fptr++ = z;

            if (!current_mode_supports_geometry_shader_)
            {
            // skip geometry vertices
              fptr += 3;
            }

            // if color by index, do not write buffer, otherwise write color
            if (!color_by_index_)
            {
              uint32_t color;
              root->convertColourValue( p.color, &color );
              uint32_t* iptr = (uint32_t*)fptr;
              *iptr = color;
            }
            ++fptr;
        }
        ++vertex_idx;
        ++current_point;
    }
    vbuf->unlock();

    // set new AABB
    (*it)->setBoundingBox(aabb);
    bounding_box_.merge(aabb);

    // add to queue
    queue->addRenderable((*it).get());
  }
}

void OgrePointCloud::_notifyAttached(Ogre::Node *parent, bool isTagPoint)
{
  MovableObject::_notifyAttached(parent, isTagPoint);
}

uint32_t OgrePointCloud::getVerticesPerPoint()
{
  if (current_mode_supports_geometry_shader_)
  {
    return 1;
  }

  if (render_mode_ == RM_POINTS)
  {
    return 1;
  }

  if (render_mode_ == RM_SQUARES)
  {
    return 6;
  }

  if (render_mode_ == RM_FLAT_SQUARES)
  {
    return 6;
  }

  if (render_mode_ == RM_TILES)
    {
      return 6;
    }

  if (render_mode_ == RM_SPHERES)
  {
    return 3;
  }

  if (render_mode_ == RM_BOXES)
  {
    return 36;
  }

  return 1;
}

void OgrePointCloud::setPickColor(const Ogre::ColourValue& color)
{
  pick_color_ = color;
  Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);

  V_OgrePointCloudRenderable::iterator it = renderables_.begin();
  V_OgrePointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(PICK_COLOR_PARAMETER, pick_col);
  }
  getUserObjectBindings().setUserAny( "pick_handle", Ogre::Any( OgreColorConversion::colorToHandle( color )));
}

OgrePointCloudRenderablePtr OgrePointCloud::createRenderable( int num_points )
{
  OgrePointCloudRenderablePtr rend(new OgrePointCloudRenderable(this, num_points, !current_mode_supports_geometry_shader_));
  rend->setMaterial(current_material_->getName());
  Ogre::Vector4 size(width_, height_, depth_, 0.0f);
  Ogre::Vector4 alpha(alpha_, 0.0f, 0.0f, 0.0f);
  Ogre::Vector4 highlight(0.0f, 0.0f, 0.0f, 0.0f);
  Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);
  rend->setCustomParameter(SIZE_PARAMETER, size);
  rend->setCustomParameter(ALPHA_PARAMETER, alpha);
  rend->setCustomParameter(HIGHLIGHT_PARAMETER, highlight);
  rend->setCustomParameter(PICK_COLOR_PARAMETER, pick_col);
  rend->setCustomParameter(NORMAL_PARAMETER, Ogre::Vector4(common_direction_));
  rend->setCustomParameter(UP_PARAMETER, Ogre::Vector4(common_up_vector_));
  if (getParentSceneNode())
  {
    getParentSceneNode()->attachObject(rend.get());
  }
  renderables_.push_back(rend);

  return rend;
}

#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6)
void OgrePointCloud::visitRenderables(Ogre::Renderable::Visitor* visitor, bool debugRenderables)
{

}
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

OgrePointCloudRenderable::OgrePointCloudRenderable(OgrePointCloud* parent, int num_points, bool use_tex_coords)
: parent_(parent)
{
  // Initialize render operation
  mRenderOp.operationType = Ogre::RenderOperation::OT_POINT_LIST;
  mRenderOp.useIndexes = false;
  mRenderOp.vertexData = new Ogre::VertexData;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = 0;

  Ogre::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration;
  size_t offset = 0;

  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  if (use_tex_coords)
  {
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES, 0);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  }

  decl->addElement(0, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);

  Ogre::HardwareVertexBufferSharedPtr vbuf =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
      mRenderOp.vertexData->vertexDeclaration->getVertexSize(0),
      num_points,
      Ogre::HardwareBuffer::HBU_DYNAMIC);

  // Bind buffer
  mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vbuf);
}

OgrePointCloudRenderable::~OgrePointCloudRenderable()
{
  delete mRenderOp.vertexData;
  delete mRenderOp.indexData;
}

Ogre::HardwareVertexBufferSharedPtr OgrePointCloudRenderable::getBuffer()
{
  return mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);
}

void OgrePointCloudRenderable::_notifyCurrentCamera(Ogre::Camera* camera)
{
  SimpleRenderable::_notifyCurrentCamera( camera );
}

Ogre::Real OgrePointCloudRenderable::getBoundingRadius(void) const
{
  return Ogre::Math::Sqrt(std::max(mBox.getMaximum().squaredLength(), mBox.getMinimum().squaredLength()));
}

Ogre::Real OgrePointCloudRenderable::getSquaredViewDepth(const Ogre::Camera* cam) const
{
  Ogre::Vector3 vMin, vMax, vMid, vDist;
   vMin = mBox.getMinimum();
   vMax = mBox.getMaximum();
   vMid = ((vMax - vMin) * 0.5) + vMin;
   vDist = cam->getDerivedPosition() - vMid;

   return vDist.squaredLength();
}

void OgrePointCloudRenderable::getWorldTransforms(Ogre::Matrix4* xform) const
{
   parent_->getWorldTransforms(xform);
}

const Ogre::LightList& OgrePointCloudRenderable::getLights() const
{
  return parent_->queryLights();
}

