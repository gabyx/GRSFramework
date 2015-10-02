#ifndef GRSF_General_GridderData_hpp
#define GRSF_General_GridderData_hpp


#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include <map>
#include <unordered_map>
#include <vector>

#include DynamicsSystem_INCLUDE_FILE

#include "GRSF/General/GridExtractionSettings.hpp"


#define  DEFINE_GRIDDERCONVERTERDATA_TYPES  \
    DEFINE_DYNAMICSYSTEM_BASE_TYPES \
    using GeometryMapType = std::unordered_map< RigidBodyIdType , typename RigidBodyType::GeometryType>; \
    using ScalesMap = std::unordered_map< RigidBodyIdType ,Vector3 >; \
    using VisMeshMap = std::unordered_map< RigidBodyIdType , boost::filesystem::path  >; \
    using GridExtSettingsType = GridExtractionSettings; \
    using GridExtSettingsListType = std::vector<GridExtSettingsType>;


class GridderData {
public:

    DEFINE_GRIDDERCONVERTERDATA_TYPES

    /** Data storage from the Logic file*/
    GridExtSettingsListType m_gridSettingsList;

    /** Data storage from the SceneFile (not yet needed) */
    GlobalGeometryMapType m_globalGeometries;
    GeometryMapType m_geometryMap;
    ScalesMap m_scales;
    VisMeshMap m_visMeshs;

};


#endif

