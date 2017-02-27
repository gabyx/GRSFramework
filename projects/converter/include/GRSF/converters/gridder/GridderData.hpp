// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_gridder_GridderData_hpp
#define GRSF_converters_gridder_GridderData_hpp

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include <map>
#include <unordered_map>
#include <vector>

#include DynamicsSystem_INCLUDE_FILE

#include "GRSF/converters/gridder/GridExtractionSettings.hpp"

#define DEFINE_GRIDDERCONVERTERDATA_TYPES                                                                      \
    DEFINE_DYNAMICSYSTEM_BASE_TYPES                                                                            \
    using GeometryMapType         = std::unordered_map<RigidBodyIdType, typename RigidBodyType::GeometryType>; \
    using ScalesMap               = StdUMapAligned<RigidBodyIdType, Vector3>;                                  \
    using VisMeshMap              = std::unordered_map<RigidBodyIdType, boost::filesystem::path>;              \
    using GridExtSettingsType     = GridExtractionSettings;                                                    \
    using GridExtSettingsListType = StdVecAligned<GridExtSettingsType>;

class GridderData
{
    public:
    DEFINE_GRIDDERCONVERTERDATA_TYPES

    /** Data storage from the Logic file*/
    GridExtSettingsListType m_gridSettingsList;

    /** Data storage from the SceneFile (not yet needed) */
    GlobalGeometryMapType m_globalGeometries;
    GeometryMapType       m_geometryMap;
    ScalesMap             m_scales;
    VisMeshMap            m_visMeshs;
};

#endif
