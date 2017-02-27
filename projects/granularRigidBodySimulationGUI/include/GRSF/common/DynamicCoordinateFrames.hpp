// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_DynamicCoordinateFrames_hpp
#define GRSF_common_DynamicCoordinateFrames_hpp

#include <Ogre.h>

#include <boost/thread.hpp>

#include "GRSF/common/DynamicLines.hpp"

/** Rendering coordinate systems
*   TODO derive from MovableObject, MovableObjectFactory -> register Ogre::Root->addMovableObjectFactory
*/
class DynamicCoordinateFrames
{
    public:
    DynamicCoordinateFrames()
        : m_xAxis(DynamicLines::OperationType::OT_LINE_LIST)
        , m_yAxis(DynamicLines::OperationType::OT_LINE_LIST)
        , m_zAxis(DynamicLines::OperationType::OT_LINE_LIST){};

    void reserveCoordinateSystems(unsigned int nCoordSys)
    {
        m_xAxis.reserve(nCoordSys * 2);
        m_yAxis.reserve(nCoordSys * 2);
        m_zAxis.reserve(nCoordSys * 2);
    }

    void setVisible(bool value, bool cascade = true)
    {
        m_dynCoordFrameNode->setVisible(value, cascade);
    }

    ~DynamicCoordinateFrames()
    {
        m_dynCoordFrameNode->detachObject(&m_xAxis);
        m_dynCoordFrameNode->detachObject(&m_yAxis);
        m_dynCoordFrameNode->detachObject(&m_zAxis);
    }

    /** Attach the coordinate systems to the scene at node
    \p baseFrame.
    @param baseFrame The node where the collection of x,y,z axes are attached.
    The node can be scaled, which only scales the axes, but leaves the origin  of the coordinate system
    at the same place.
    */
    void addToScene(Ogre::SceneNode* baseFrame,
                    std::string      xAxisMat = "BaseWhiteNoLighting",
                    std::string      yAxisMat = "BaseWhiteNoLighting",
                    std::string      zAxisMat = "BaseWhiteNoLighting")
    {
        boost::mutex::scoped_lock l(m_mutexLock);

        m_dynCoordFrameNode = baseFrame;
        m_dynCoordFrameNode->attachObject(&m_xAxis);
        m_dynCoordFrameNode->attachObject(&m_yAxis);
        m_dynCoordFrameNode->attachObject(&m_zAxis);

        m_xAxis.setMaterial(xAxisMat);
        m_yAxis.setMaterial(yAxisMat);
        m_zAxis.setMaterial(zAxisMat);

        m_xAxis.setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
        m_zAxis.setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
        m_yAxis.setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    }

    template <typename TIterator>
    void updateFramesSim(TIterator itBegin, TIterator itEnd)
    {
        boost::mutex::scoped_lock l(m_mutexLock);
        m_xAxis.clear();
        m_yAxis.clear();
        m_zAxis.clear();

        Ogre::Vector3 sc = m_dynCoordFrameNode->getScale();

        for (TIterator it = itBegin; it != itEnd; ++it)
        {
            // do not scale the positions
            // of the coordinate systems by the scale of m_dynCoordFrameNode
            Ogre::Vector3 p(
                (*it)->m_cFrame.m_p(0) / sc.x, (*it)->m_cFrame.m_p(1) / sc.y, (*it)->m_cFrame.m_p(2) / sc.z);

            m_xAxis.addPoint(p);
            m_xAxis.addPoint(
                p.x + (*it)->m_cFrame.m_e_x(0), p.y + (*it)->m_cFrame.m_e_x(1), p.z + (*it)->m_cFrame.m_e_x(2));

            m_yAxis.addPoint(p);
            m_yAxis.addPoint(
                p.x + (*it)->m_cFrame.m_e_y(0), p.y + (*it)->m_cFrame.m_e_y(1), p.z + (*it)->m_cFrame.m_e_y(2));

            m_zAxis.addPoint(p);
            m_zAxis.addPoint(
                p.x + (*it)->m_cFrame.m_e_z(0), p.y + (*it)->m_cFrame.m_e_z(1), p.z + (*it)->m_cFrame.m_e_z(2));
        }
    }

    void updateFramesVis()
    {
        boost::mutex::scoped_lock l(m_mutexLock);
        m_xAxis.update();
        m_yAxis.update();
        m_zAxis.update();
        m_dynCoordFrameNode->needUpdate();
    };

    private:
    boost::mutex     m_mutexLock;  // SimThread locks and updates points, vis thread locks and
    Ogre::SceneNode* m_dynCoordFrameNode;
    DynamicLines     m_xAxis;
    DynamicLines     m_yAxis;
    DynamicLines     m_zAxis;
};

#endif
