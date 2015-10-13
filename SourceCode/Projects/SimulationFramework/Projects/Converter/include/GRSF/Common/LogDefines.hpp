/*
 *  GRSF/Common/LogDefines.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef LogDefines_hpp
#define LogDefines_hpp


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
    #define SCENEPARSER_LOGLEVEL 1  /// 0 - No output, 1 basic output, 2 medium output, 3 full output

    /// RenderConverter
    #define RENDERCONVERTER_LOGLEVEL 3

    /// LogicParser
    #define LOGICPARSER_LOGLEVEL 3

    /// GridderParser
    #define GRIDDERPARSER_LOGLEVEL 3
    #define GRIDEXTRACTORRPARSER_LOGLEVEL 3
#else

    #define LOGSJ(A)

    /// SceneParser
    #define SCENEPARSER_LOGLEVEL 1  /// 0 - No output, 1 basic output, 2 medium output, 3 full output

    /// RenderConverter
    #define RENDERCONVERTER_LOGLEVEL 1

    /// LogicParser
    #define LOGICPARSER_LOGLEVEL 3

    /// GridderParser
    #define GRIDDERPARSER_LOGLEVEL 1
    #define GRIDEXTRACTORRPARSER_LOGLEVEL 1
#endif
/* @} */


/** SceneParser Log Macros */
#define LOGSC(log , message) LOG(log,message);
#define LOGSCLEVEL(level, logptr , message) LOGLEVEL(level,SCENEPARSER_LOGLEVEL,logptr,message);
#define LOGSCLEVEL1( logptr , message) LOGSCLEVEL( 1 , logptr , message) ;
#define LOGSCLEVEL2( logptr , message) LOGSCLEVEL( 2 , logptr , message) ;
#define LOGSCLEVEL3( logptr , message) LOGSCLEVEL( 3 , logptr , message) ;
#define SKIPLOGSC( logptr , message )  LOGSCLEVEL(1,logptr,message);


/** RenderConverter Log Macros */
#define LOGRC(log , message) LOG(log,message);
#define LOGRCLEVEL(level, logptr , message) LOGLEVEL(level,RENDERCONVERTER_LOGLEVEL,logptr,message);
#define LOGRCLEVEL1( logptr , message) LOGRCLEVEL( 1 , logptr , message) ;
#define LOGRCLEVEL2( logptr , message) LOGRCLEVEL( 2 , logptr , message) ;
#define LOGRCLEVEL3( logptr , message) LOGRCLEVEL( 3 , logptr , message) ;

/** LogicParser Log Macros */
#define LOGLP(log , message) LOG(log,message);
#define LOGLPLEVEL(level, logptr , message) LOGLEVEL(level,LOGICPARSER_LOGLEVEL,logptr,message);
#define LOGLPLEVEL1( logptr , message) LOGLPLEVEL( 1 , logptr , message) ;
#define LOGLPLEVEL2( logptr , message) LOGLPLEVEL( 2 , logptr , message) ;
#define LOGLPLEVEL3( logptr , message) LOGLPLEVEL( 3 , logptr , message) ;

/** GridderParser Log Macros */
#define LOGGP(log , message) LOG(log,message);
#define LOGGPLEVEL(level, logptr , message) LOGLEVEL(level,GRIDDERPARSER_LOGLEVEL,logptr,message);
#define LOGGPLEVEL1( logptr , message) LOGGPLEVEL( 1 , logptr , message) ;
#define LOGGPLEVEL2( logptr , message) LOGGPLEVEL( 2 , logptr , message) ;
#define LOGGPLEVEL3( logptr , message) LOGGPLEVEL( 3 , logptr , message) ;

/** GridExtractor */
#define LOGGC(log , message) LOG(log,message);
#define LOGGCLEVEL(level, logptr , message) LOGLEVEL(level,GRIDEXTRACTORRPARSER_LOGLEVEL,logptr,message);
#define LOGGCLEVEL1( logptr , message) LOGGCLEVEL( 1 , logptr , message) ;
#define LOGGCLEVEL2( logptr , message) LOGGCLEVEL( 2 , logptr , message) ;
#define LOGGCLEVEL3( logptr , message) LOGGCLEVEL( 3 , logptr , message) ;


/** @name  Deconstructor and Constructor Macros
* @brief Deconstructor and Constructor Macros to Debug correct dealloction of objects.
*/
/* @{ */
#ifndef NDEBUG
#define DECONSTRUCTOR_MESSAGE \
 std::cout << "Destructor: "<< typeid(*this).name()  <<" , @ : "<< this << std::endl;
#define CONSTRUCTOR_MESSAGE \
  std::cout << "Constructor: "<< typeid(*this).name()  <<" , @ : "<< this << std::endl;;
#else
  #define DECONSTRUCTOR_MESSAGE
  #define CONSTRUCTOR_MESSAGE
#endif
/* @} */


#endif
