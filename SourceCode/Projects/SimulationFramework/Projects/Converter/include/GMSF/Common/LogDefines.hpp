/*
 *  LogDefines.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */#ifndef GMSF_Common_LogDefines_hpp
#define GMSF_Common_LogDefines_hpp


/** @name Global Log Macros
* @brief for SimFileConverter
*/
/* @{ */


#define LOG( logptr , message )  ( *(logptr) ) << message ;  ///< Macro to easily write into a SimpleLogger::Log.
#define LOGLEVEL(level,setlevel,logptr,message) if( level <= setlevel ){  LOG(logptr,message); }


#ifndef NDEBUG
    // DEBUG!
    #define LOGSJ(A){ A } // Output SimfileJoiner

    /// SceneParser
    #define SCENEPARSER_LOGLEVEL 3  /// 0 - No output, 1 basic output, 2 medium output, 3 full output

    /// RenderScriptConverter
    #define RENDERCONVERTER_LOGLEVEL 3

    /// MaterialCollectionParser
    #define MATERIALCOLLECTIONPARSER_LOGLEVEL 3

#else

    #define LOGSJ(A)

    /// SceneParser
    #define SCENEPARSER_LOGLEVEL 1  /// 0 - No output, 1 basic output, 2 medium output, 3 full output

    /// RenderScriptConverter
    #define RENDERCONVERTER_LOGLEVEL 1

    /// MaterialCollectionParser
    #define MATERIALCOLLECTIONPARSER_LOGLEVEL 1

#endif
/* @} */


/** SceneParser Log Macros */
#define LOGSC(log , message) LOG(log,message);
#define LOGSCLEVEL(level, logptr , message) LOGLEVEL(level,SCENEPARSER_LOGLEVEL,logptr,message);
#define LOGSCLEVEL1( logptr , message) LOGSCLEVEL( 1 , logptr , message) ;
#define LOGSCLEVEL2( logptr , message) LOGSCLEVEL( 2 , logptr , message) ;
#define LOGSCLEVEL3( logptr , message) LOGSCLEVEL( 3 , logptr , message) ;
#define SKIPLOGSC( logptr , message )  LOGSCLEVEL(1,logptr,message);


/** RenderScriptConverter Log Macros */
#define LOGRC(log , message) LOG(log,message);
#define LOGRCLEVEL(level, logptr , message) LOGLEVEL(level,RENDERCONVERTER_LOGLEVEL,logptr,message);
#define LOGRCLEVEL1( logptr , message) LOGSCLEVEL( 1 , logptr , message) ;
#define LOGRCLEVEL2( logptr , message) LOGSCLEVEL( 2 , logptr , message) ;
#define LOGRCLEVEL3( logptr , message) LOGSCLEVEL( 3 , logptr , message) ;

/** MaterialCollectionParser Log Macros */
#define LOGMC(log , message) LOG(log,message);
#define LOGMCLEVEL(level, logptr , message) LOGLEVEL(level,MATERIALCOLLECTIONPARSER_LOGLEVEL,logptr,message);
#define LOGMCLEVEL1( logptr , message) LOGSCLEVEL( 1 , logptr , message) ;
#define LOGMCLEVEL2( logptr , message) LOGSCLEVEL( 2 , logptr , message) ;
#define LOGMCLEVEL3( logptr , message) LOGSCLEVEL( 3 , logptr , message) ;


#endif
