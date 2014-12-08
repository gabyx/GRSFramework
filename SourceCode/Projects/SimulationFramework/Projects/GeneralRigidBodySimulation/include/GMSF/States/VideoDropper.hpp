#ifndef GMSF_States_VideoDropper_hpp
#define GMSF_States_VideoDropper_hpp

#include "GMSF/Common/TypeDefs.hpp"
#include "GMSF/Common/LogDefines.hpp"

#include "GMSF/Singeltons/Contexts/RenderContext.hpp"

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

class VideoDropper{
public:

   VideoDropper(){
      reset();
      m_fps = 25; // Standart value
   };

   ~VideoDropper(){};

   //Render threads
   void cancelWait(){
     // Dont do frame frop just cancel any wait off threads!
     // Essential function which is used to cancel the whole video drop!
     signalFrameDropped();
   }

   // Render thread
   void tryToDropFrame()
   {
      static int dryloop = 0;

      if(dryloop == 0){
         bool drop;
         {
            boost::mutex::scoped_lock l(m_mutex);
            drop = m_bDropFrame;
         }

         if(drop){
            dryloop = 5;
         }
      }else if(dryloop == 1){
          dropFrame();
          signalFrameDropped();
          dryloop = 0;
      }else{
         dryloop--;
      }
   }

   // Thread which comands the frame drop!
   void tryToWaitForFrameDrop(){
      boost::mutex::scoped_lock l(m_mutex);
      if(m_bDropFrame){
         waitFrameDropped(l); // releases mutex, and if waked locks it again
         m_nFramesDropped++;
         m_bDropFrame = false;
         m_currentTime = 0;
      }
   }

   void addToCurrentTime(double addTime){
      boost::mutex::scoped_lock l(m_mutex);
      m_currentTime += addTime;
      if( m_currentTime * m_fps >= 1 || m_currentTime == 0.0){ // if more then one frame goes into the timespan
         m_bDropFrame = true;
      }
   }

   void setFPS(double fps){
      boost::mutex::scoped_lock l(m_mutex);
      m_fps = fps;
   }

   void reset(){
      boost::mutex::scoped_lock l(m_mutex);
      m_bDropFrame = true;
      m_currentTime = 0;
      m_nFramesDropped = 0;
   }

   void setFolderPath(boost::filesystem::path folderPath){
      boost::mutex::scoped_lock l(m_mutex);
      m_folderPath = folderPath;
   }

private:

   boost::filesystem::path m_folderPath;

   boost::mutex m_mutex;
   bool m_bDropFrame;

   double m_currentTime;

   double m_fps; //[frames/second]

   unsigned int m_nFramesDropped;

   boost::condition_variable m_frameDropped;

   void waitFrameDropped(boost::mutex::scoped_lock & lock){
      m_frameDropped.wait(lock);
   }
   void signalFrameDropped(){
      m_frameDropped.notify_all();
   }

   void dropFrame(){
      std::stringstream filename;
      filename << SIM_VIDEO_PREFIX <<m_nFramesDropped<<".tga";
      boost::filesystem::path file = m_folderPath;
      file /=  filename.str();
      RenderContext::getSingleton().m_pRenderWnd->writeContentsToFile(file.string());
      //Sleep(560);
   }
};


#endif
