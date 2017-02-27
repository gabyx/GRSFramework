// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MultiBodySimFileMPI_hpp
#define GRSF_dynamics_general_MultiBodySimFileMPI_hpp

#include <fstream>
#include <sstream>
#include <type_traits>
#include <vector>

#include <boost/filesystem.hpp>

#include "GRSF/common/StaticAssert.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include DynamicsSystem_INCLUDE_FILE

#include "GRSF/common/CommonFunctions.hpp"

#include "GRSF/common/SerializationHelpersEigen.hpp"
#include "GRSF/dynamics/general/AdditionalBodyData.hpp"
#include "GRSF/dynamics/general/MultiBodySimFileIOHelpers.hpp"

#define SIM_FILE_MPI_SIGNATURE_LENGTH 4
#define SIM_FILE_MPI_SIGNATURE \
    {                          \
        'M', 'B', 'S', 'F'     \
    }

#define SIM_FILE_MPI_VERSION 2

#define SIM_FILE_MPI_EXTENSION ".sim"

class MultiBodySimFileMPI
{
    public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    MultiBodySimFileMPI();
    ~MultiBodySimFileMPI();

    bool openWrite(MPI_Comm                       comm,
                   const boost::filesystem::path& file_path,
                   unsigned int                   nDOFqBody,
                   unsigned int                   nDOFuBody,
                   const unsigned int             nSimBodies,
                   bool                           truncate = true);
    void close();

    inline void write(double time, const std::vector<char>& bytes, unsigned int nBodies);

    inline void write(double time, const RigidBodyContainer& bodyList)
    {
        // writeBySharedPtr(time, bodyList);
        writeByOffsets(time, bodyList);
        // writeByOffsets2(time, bodyList);
    }

    std::string getErrorString()
    {
        return m_errorString.str();
    }

    private:
    using RigidBodyIdType = typename RigidBodyType::RigidBodyIdType;

    // Communicator which is used to write the file in parallel, this communicator is duplicated from the one inputed
    // This prevents that accidentaly some other synchronization and stuff is
    MPI_Comm    m_comm;
    RankIdType  m_rank      = 0;  // rank in the communicator;
    std::size_t m_processes = 0;

    MPI_Datatype m_bodyStateTypeMPI;
    MPI_File     m_file_handle;  ///< The file stream which represents the binary data.

    static const char m_simFileSignature[SIM_FILE_MPI_SIGNATURE_LENGTH];  ///< The .sim file header.

    void writeBySharedPtr(double time, const typename DynamicsSystemType::RigidBodySimContainerType& bodyList);
    void writeByOffsets(double time, const typename DynamicsSystemType::RigidBodySimContainerType& bodyList);
    void writeByOffsets2(double time, const typename DynamicsSystemType::RigidBodySimContainerType& bodyList);

    void                    writeHeader();
    boost::filesystem::path m_filePath;

    void            setByteLengths();
    unsigned int    m_nSimBodies     = 0;
    std::streamsize m_nBytesPerState = 0;  ///< m_nSimBodies*(id,q,u) + time
    std::streamsize m_nBytesPerQ = 0, m_nBytesPerU = 0;

    unsigned int m_nStates = 0;

    unsigned int m_nDOFuBody = 0, m_nDOFqBody = 0;

    std::streamsize m_nBytesPerQBody = 0;
    std::streamsize m_nBytesPerUBody = 0;

    static const typename AdditionalBodyData::TypeEnum m_additionalBytesPerBodyType =
        AdditionalBodyData::TypeEnum::PROCESS_MATERIAL_OVERLAP_GLOBGEOMID;
    static constexpr std::streamoff getAdditionalBytesPerBody()
    {
        return AdditionalBodyData::getAdditionalBytesPerBody(m_additionalBytesPerBodyType);
    }
    static const std::streamsize m_nAdditionalBytesPerBody;

    std::streamsize m_nBytesPerBody = 0;  ///< id,q,u + m_nAdditionalBytesPerBody

    static const std::streamsize m_headerLength =
        SIM_FILE_MPI_SIGNATURE_LENGTH * sizeof(char) + sizeof(unsigned int) + 3 * sizeof(unsigned int) +
        2 * sizeof(unsigned int);  ///< 'MBSF' + nBodies, NDOFq, NDOFu, additionalBytesType (0=nothing, 1 = + process
                                   /// rank, etc.), additionalBytesPerBody

    MultiBodySimFileMPI& operator=(const MultiBodySimFileMPI& file);

    std::vector<char> m_writebuffer;

    std::stringstream m_errorString;

    bool mpiSucceded(int err)
    {
        m_errorString.str("");
        if (err != MPI_SUCCESS)
        {
            char* string = nullptr;
            int   length;
            MPI_Error_string(err, string, &length);
            m_errorString << string;
            return false;
        }
        return true;
    }
};

#endif
