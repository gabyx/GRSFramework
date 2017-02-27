// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef LoggerTest_hpp
#define LoggerTest_hpp

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "GRSF/common/SimpleLogger.hpp"

#define LOG(_logptr_, _message_) (*(_logptr_)) << _message_;  ///< Macro to easily write into a Ogre::Log.

namespace Logging
{
void loggerTest()
{
    Logging::LogManager logManager;

    Logging::Log* log = Logging::LogManager::getSingleton().createLog(
        "LogTest", true, false, boost::filesystem::path("./LogTest.log"), true);
    log->logMessage("log1");
    log->logMessage("log2");

    for (int j = 0; j < 20; j++)
    {
        LOG(log,
            "Start "
                << "Ids")
        for (unsigned int i = 0; i < 10; i++)
        {
            LOG(log, "id: " << i << ", ");
        }
        LOG(log, std::endl;)
    }

    Logging::Log* log2 = Logging::LogManager::getSingleton().createLog(
        "LogTest2", true, false, boost::filesystem::path("./LogTest.log2"), true);
    log2->logMessage("log1");
    log2->logMessage("log2");

    for (int j = 0; j < 20; j++)
    {
        LOG(log2,
            "Start "
                << "Ids")
        for (unsigned int i = 0; i < 10; i++)
        {
            LOG(log2, "id: " << i << ", ");
        }
        LOG(log2, std::endl;)
    }
    LOG(log2,
        "id: "
            << "asd"
            << ", ");
};
};

#endif
