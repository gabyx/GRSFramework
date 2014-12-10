#ifndef GMSF_Common_DynamicCoordinateFrames_hpp
#define GMSF_Common_DynamicCoordinateFrames_hpp

#include <Ogre.h>

#include <boost/thread.hpp>

#include "GMSF/Common/DynamicLines.hpp"

class DynamicCoordinateFrames{
public:

    DynamicCoordinateFrames():
        m_xAxis(DynamicLines::OperationType::OT_LINE_LIST),
        m_yAxis(DynamicLines::OperationType::OT_LINE_LIST),
        m_zAxis(DynamicLines::OperationType::OT_LINE_LIST)
    {
        m_scaleFactor = 0.2;
    };

    void reserve(unsigned int nFrames){
        m_xAxis.reserve(nFrames*2);
        m_yAxis.reserve(nFrames*2);
        m_zAxis.reserve(nFrames*2);
    }

    void addToScene(Ogre::SceneNode * baseFrame,
                    std::string xAxisMat = "BaseWhiteNoLighting",
                    std::string yAxisMat = "BaseWhiteNoLighting",
                    std::string zAxisMat = "BaseWhiteNoLighting",
                    double scaleFactor=-1 ){
        boost::mutex::scoped_lock l(m_mutexLock);

        if(scaleFactor > 0){
            m_scaleFactor = scaleFactor;
        }

        m_dynCoordFrameNode = baseFrame->createChildSceneNode("DynamicCoordinateFrames");
        m_dynCoordFrameNode->attachObject(&m_xAxis);
        m_dynCoordFrameNode->attachObject(&m_yAxis);
        m_dynCoordFrameNode->attachObject(&m_zAxis);

        std::cout << "Set Materials Contact Frame:" << std::endl;
        m_xAxis.setMaterial(xAxisMat);
        m_yAxis.setMaterial(yAxisMat);
        m_zAxis.setMaterial(zAxisMat);

        m_xAxis.setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
        m_zAxis.setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
        m_yAxis.setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    }

    template<typename TIterator>
    void updateFramesSim(TIterator itBegin, TIterator itEnd){
        boost::mutex::scoped_lock l(m_mutexLock);
        m_xAxis.clear();
        m_yAxis.clear();
        m_zAxis.clear();
        for(TIterator it = itBegin; it != itEnd; it++){
            m_xAxis.addPoint((*it)->m_cFrame.m_p(0),
                             (*it)->m_cFrame.m_p(1),
                             (*it)->m_cFrame.m_p(2));
            m_xAxis.addPoint((*it)->m_cFrame.m_p(0) + (*it)->m_cFrame.m_e_x(0)*m_scaleFactor,
                             (*it)->m_cFrame.m_p(1) + (*it)->m_cFrame.m_e_x(1)*m_scaleFactor,
                             (*it)->m_cFrame.m_p(2) + (*it)->m_cFrame.m_e_x(2)*m_scaleFactor);

            m_yAxis.addPoint((*it)->m_cFrame.m_p(0),
                             (*it)->m_cFrame.m_p(1),
                             (*it)->m_cFrame.m_p(2));
            m_yAxis.addPoint((*it)->m_cFrame.m_p(0) + (*it)->m_cFrame.m_e_y(0)*m_scaleFactor,
                             (*it)->m_cFrame.m_p(1) + (*it)->m_cFrame.m_e_y(1)*m_scaleFactor,
                             (*it)->m_cFrame.m_p(2) + (*it)->m_cFrame.m_e_y(2)*m_scaleFactor);

            m_zAxis.addPoint((*it)->m_cFrame.m_p(0),
                             (*it)->m_cFrame.m_p(1),
                             (*it)->m_cFrame.m_p(2));
            m_zAxis.addPoint((*it)->m_cFrame.m_p(0) + (*it)->m_cFrame.m_e_z(0)*m_scaleFactor,
                             (*it)->m_cFrame.m_p(1) + (*it)->m_cFrame.m_e_z(1)*m_scaleFactor,
                             (*it)->m_cFrame.m_p(2) + (*it)->m_cFrame.m_e_z(2)*m_scaleFactor);

        }
    }

    void updateFramesVis(){
        boost::mutex::scoped_lock l(m_mutexLock);
        m_xAxis.update();
        m_yAxis.update();
        m_zAxis.update();
        m_dynCoordFrameNode->needUpdate();

    };

private:
    double m_scaleFactor;
    boost::mutex m_mutexLock; // SimThread locks and updates points, vis thread locks and
    Ogre::SceneNode*  m_dynCoordFrameNode;
    DynamicLines m_xAxis;
    DynamicLines m_yAxis;
    DynamicLines m_zAxis;

};

#endif
