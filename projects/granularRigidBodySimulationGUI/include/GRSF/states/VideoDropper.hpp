// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_states_VideoDropper_hpp
#define GRSF_states_VideoDropper_hpp

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/singeltons/contexts/RenderContext.hpp"

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

class VideoDropper
{
    public:
    VideoDropper()
    {
        reset();
        m_fps = 25;  // Standart value
    };

    ~VideoDropper(){};

    // Render threads
    void cancelWait()
    {
        // Dont do frame frop just cancel any wait off threads!
        // Essential function which is used to cancel the whole video drop!
        signalFrameDropped();
    }

    // Render thread
    void tryToDropFrame()
    {
        static int dryloop = 0;

        if (dryloop == 0)
        {
            bool drop;
            {
                boost::mutex::scoped_lock l(m_mutex);
                drop = m_bDropFrame;
            }

            if (drop)
            {
                dryloop = 5;
            }
        }
        else if (dryloop == 1)
        {
            dropFrame();
            signalFrameDropped();
            dryloop = 0;
        }
        else
        {
            dryloop--;
        }
    }

    // Thread which comands the frame drop!
    void tryToWaitForFrameDrop()
    {
        boost::mutex::scoped_lock l(m_mutex);
        if (m_bDropFrame)
        {
            waitFrameDropped(l);  // releases mutex, and if waked locks it again
            m_nFramesDropped++;
            m_bDropFrame  = false;
            m_currentTime = 0;
        }
    }

    void addToCurrentTime(double addTime)
    {
        boost::mutex::scoped_lock l(m_mutex);
        m_currentTime += addTime;
        if (m_currentTime * m_fps >= 1 || m_currentTime == 0.0)
        {  // if more then one frame goes into the timespan
            m_bDropFrame = true;
        }
    }

    void setFPS(double fps)
    {
        boost::mutex::scoped_lock l(m_mutex);
        m_fps = fps;
    }

    void reset()
    {
        boost::mutex::scoped_lock l(m_mutex);
        m_bDropFrame     = true;
        m_currentTime    = 0;
        m_nFramesDropped = 0;
    }

    void setFolderPath(boost::filesystem::path folderPath)
    {
        boost::mutex::scoped_lock l(m_mutex);
        m_folderPath = folderPath;
    }

    private:
    boost::filesystem::path m_folderPath;

    boost::mutex m_mutex;
    bool         m_bDropFrame;

    double m_currentTime;

    double m_fps;  //[frames/second]

    unsigned int m_nFramesDropped;

    boost::condition_variable m_frameDropped;

    void waitFrameDropped(boost::mutex::scoped_lock& lock)
    {
        m_frameDropped.wait(lock);
    }
    void signalFrameDropped()
    {
        m_frameDropped.notify_all();
    }

    void dropFrame()
    {
        std::stringstream filename;
        filename << SIM_VIDEO_PREFIX << m_nFramesDropped << ".tga";
        boost::filesystem::path file = m_folderPath;
        file /= filename.str();
        RenderContext::getSingleton().m_pRenderWnd->writeContentsToFile(file.string());
        // Sleep(560);
    }
};

#endif
