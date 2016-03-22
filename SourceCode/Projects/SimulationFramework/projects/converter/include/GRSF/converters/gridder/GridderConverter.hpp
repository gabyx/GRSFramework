#ifndef GRSF_Converters_GridderConverter_hpp
#define GRSF_Converters_GridderConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <pugixml.hpp>

#include <boost/filesystem.hpp>

#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Converters/SimFileConverter.hpp"
#include "GRSF/Converters/Gridder/GridderData.hpp"


class GridderConverter : public SimFileConverter {
public:

    DEFINE_LAYOUT_CONFIG_TYPES

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;

    using Base = SimFileConverter;

    GridderConverter(const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path sceneFile,
                  boost::filesystem::path logicFile);

    void convert();


private:

    void setup();

    GridderData m_gridderData; ///< Parsed settings for grids;

    boost::filesystem::path m_logicFile;
    boost::filesystem::path m_sceneFile;

};

#endif // GRSF_Converters_GridConverter_hpp


