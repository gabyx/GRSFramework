#ifndef GRSF_converters_gridder_GridderConverter_hpp
#define GRSF_converters_gridder_GridderConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <pugixml.hpp>

#include <boost/filesystem.hpp>

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/converters/SimFileConverter.hpp"
#include "GRSF/converters/gridder/GridderData.hpp"


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


