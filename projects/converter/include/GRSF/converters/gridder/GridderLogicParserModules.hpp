// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_gridder_GridderLogicParserModules_hpp
#define GRSF_converters_gridder_GridderLogicParserModules_hpp

#include <fstream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/common/Asserts.hpp"

#include "GRSF/common/XMLMacros.hpp"

#include "GRSF/converters/gridder/GridderLogicParserTraitsMacro.hpp"
#include "GRSF/dynamics/general/ParserFunctions.hpp"

namespace GridderLogicParserModules
{
//    template<typename TParserTraits>
//    class MaterialsModule {
//    public:
//        DEFINE_RENDERLOGICPARSER_TYPE_TRAITS(TParserTraits)
//
//        using MaterialMapType = typename DataStorageType::MaterialMapType;
//
//        MaterialsModule(ParserType * p, MaterialMapType * m):m_parser(p),m_materials(m), m_pLog(p->getLog()) {}
//
//        void parse(XMLNodeType & parent) {
//
//            XMLNodeType materialNode = parent.child("Materials");
//            if(!materialNode) {
//                return;
//            }
//
//            auto nodes = materialNode.children("Material");
//            auto itNodeEnd = nodes.end();
//            for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {
//                unsigned int id;
//                if(!Utilities::stringToType(id, itNode->attribute("id").value())) {
//                    GRSF_ERRORMSG("---> String conversion in Material: id failed");
//                }
//
//                GRSF_ASSERTMSG(itNode->child_value()!=""," String in material id: " << id << "is empty!")
//                m_materials->emplace(id, new RenderMaterial(id,itNode->child_value()) );
//
//                LOGLPLEVEL3(m_pLog,"---> Parsed Material with id: " << id << std::endl;)
//            }
//
//        }
//
//        void cleanUp() {
//        }
//
//        MaterialMapType * getMaterialMap() {
//            return m_materials;
//        }
//    private:
//        ParserType * m_parser;
//        LogType * m_pLog;
//        MaterialMapType * m_materials;
//    };
};

namespace GridderLogicParserModules
{
template <typename TParserTraits>
class LogicModule
{
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    DEFINE_GRIDDERLOGICPARSER_TYPE_TRAITS(TParserTraits)

    using GridExtSettingsListType = typename DataStorageType::GridExtSettingsListType;

    LogicModule(ParserType* p, GridExtSettingsListType* gridSettingsList)
        : m_pLog(p->getLog()), m_gridSettingsList(gridSettingsList)
    {
    }

    void parse(XMLNodeType& parent)
    {
        XMLNodeType logicNode = parent.child("Logic");
        if (!logicNode)
        {
            return;
        }
        // Add all tools into the execution list!
        auto nodes     = logicNode.children("GridExtraction");
        auto itNodeEnd = nodes.end();
        for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode)
        {
            parseGridExtraction(*itNode);
        }
    }
    void parseGridExtraction(XMLNodeType& gridExt)
    {
        m_gridSettingsList->emplace_back();
        auto& settings = m_gridSettingsList->back();

        settings.m_fileName = gridExt.attribute("fileName").value();
        if (settings.m_fileName.empty())
        {
            GRSF_ERRORMSG("---> String conversion 'fileName' failed");
        }

        XMLNodeType grid;
        GET_XMLCHILDNODE_CHECK(grid, "Grid", gridExt);

        parseGrid(grid, settings);

        m_extractorNames.clear();

        auto nodes         = gridExt.children("Extract");
        unsigned int extrs = 0;
        auto itNodeEnd     = nodes.end();
        for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode)
        {
            parseExtrationTypes(*itNode, settings);
            ++extrs;
        }
        if (extrs == 0)
        {
            GRSF_ERRORMSG("---> You need at least one 'Extract' node in 'GridExtraction'!")
        }
    }

    template <typename TSettings>
    void parseGrid(XMLNodeType& grid, TSettings& settings)
    {
        Vector3 minPoint;
        if (!Utilities::stringToType(minPoint, grid.attribute("minPoint").value()))
        {
            GRSF_ERRORMSG("---> String conversion 'minPoint' failed");
        }

        Vector3 maxPoint;
        if (!Utilities::stringToType(maxPoint, grid.attribute("maxPoint").value()))
        {
            GRSF_ERRORMSG("---> String conversion 'maxPoint' failed");
        }

        if (!Utilities::stringToType(settings.m_dimension, grid.attribute("dimension").value()))
        {
            GRSF_ERRORMSG("---> String conversion 'dimensions' failed");
        }

        // Parse original min/max points, which are only used for convenience for the output
        // they are not used for computations!
        // as default they are set to the values given for min/max which is perfect
        settings.m_minPointOrig = minPoint;
        auto att                = grid.attribute("minPointOrig");
        if (att)
        {
            if (!Utilities::stringToType(maxPoint, grid.attribute("maxPoint").value()))
            {
                GRSF_ERRORMSG("---> String conversion 'maxPoint' failed");
            }
        }

        settings.m_maxPointOrig = maxPoint;
        att                     = grid.attribute("maxPointOrig");
        if (att)
        {
            if (!Utilities::stringToType(maxPoint, grid.attribute("maxPoint").value()))
            {
                GRSF_ERRORMSG("---> String conversion 'maxPoint' failed");
            }
        }

        Quaternion q_KI;
        Vector3 I_r_IK;
        ParserFunctions::parseTransformSequence(grid, q_KI, I_r_IK);

        settings.m_R_KI = q_KI.toRotationMatrix();

        I_r_IK          = settings.m_R_KI.transpose() * I_r_IK;  // make K_r_IK  = A_KI * I_r_IK
        settings.m_aabb = AABB3d(I_r_IK + minPoint, I_r_IK + maxPoint);

        LOGGPLEVEL1(m_pLog, "---> Parsed GridSettings for file: " << settings.m_fileName << std::endl;)
    }

    template <typename TSettings>
    void parseExtrationTypes(XMLNodeType& extract, TSettings& settings)
    {
        std::string type = extract.attribute("type").value();

        std::string name = extract.attribute("name").value();
        if (name.empty())
        {
            GRSF_ERRORMSG("---> You need to define a unique name for extractor type: " << type)
        }
        if (m_extractorNames.find(name) != m_extractorNames.end())
        {
            GRSF_ERRORMSG("---> You need to define a unique name: " << name << " already exists for Extractor")
        }
        else
        {
            m_extractorNames.insert(name);
        }
        if (type == "TransVelProj2D")
        {
            settings.m_transVelProj2DExtractors.emplace_back(name);
            auto& velProj = settings.m_transVelProj2DExtractors.back();
            parseVelProjExtractor(extract, velProj);
        }
        else if (type == "TransVelProj1D")
        {
            settings.m_transVelProj1DExtractors.emplace_back(name);
            auto& velProj = settings.m_transVelProj1DExtractors.back();
            parseVelProjExtractor(extract, velProj);
        }
        else if (type == "TransVel")
        {
            if (settings.m_transVelExtractor.size() >= 1)
            {
                GRSF_ERRORMSG("---> You specified already a TransVel extractor, only one allowed!")
            }
            settings.m_transVelExtractor.emplace_back(name);
            auto& vel = settings.m_transVelExtractor.back();
            parseVelExtractor(extract, vel);
        }
        else if (type == "BodyMask")
        {
            settings.m_bodyMaskExtractors.emplace_back(name);
            auto& bodyMask = settings.m_bodyMaskExtractors.back();
            parseBodyMaskExtractor(extract, bodyMask);
        }
        else
        {
            GRSF_ERRORMSG("---> No extraction type: " << type)
        }
    }

    template <typename TExtractor>
    void parseVelProjExtractor(XMLNodeType& extract, TExtractor& velProj)
    {
        if (!Utilities::stringToType(velProj.m_transformToGridCoordinates,
                                     extract.attribute("transformToGridCoords").value()))
        {
            GRSF_ERRORMSG("---> String conversion 'transformToGridCoords' failed");
        }

        XMLAttributeType att = extract.attribute("useProjectionMatrix");
        if (att)
        {
            if (!Utilities::stringToType(velProj.m_useProjectionMatrix,
                                         extract.attribute("useProjectionMatrix").value()))
            {
                GRSF_ERRORMSG("---> String conversion 'useProjectionMatrix' failed");
            }
            if (velProj.m_useProjectionMatrix)
            {
                GRSF_ERRORMSG("---> Parsing Projection matrix not implemented yet");
                return;
            }
        }

        att = extract.attribute("indices");
        if (att)
        {
            if (!Utilities::stringToType(velProj.m_projIndices, extract.attribute("indices").value()))
            {
                GRSF_ERRORMSG("---> String conversion 'indices' failed");
            }
            // check indices
            if ((velProj.m_projIndices >= TExtractor::DimOut).any())
            {
                GRSF_ERRORMSG("---> In parseExtrationTypes: 'indices' out of range [0," << TExtractor::DimOut << ") !")
            }
        }
        else
        {
            GRSF_ERRORMSG("---> In parseExtrationTypes: neither 'indices' given nor 'useProjectionMatrix'= true")
        }
    }

    template <typename TExtractor>
    void parseVelExtractor(XMLNodeType& extract, TExtractor& vel)
    {
        if (!Utilities::stringToType(vel.m_transformToGridCoordinates,
                                     extract.attribute("transformToGridCoords").value()))
        {
            GRSF_ERRORMSG("---> String conversion 'transformToGridCoords' failed");
        }
    }

    template <typename TExtractor>
    void parseBodyMaskExtractor(XMLNodeType& extract, TExtractor& vel)
    {
        // Nothing to parse!
    }

    void cleanUp()
    {
        m_extractorNames.clear();
    }

private:
    LogType* m_pLog;
    GridExtSettingsListType* m_gridSettingsList;

    std::unordered_set<std::string> m_extractorNames;
};
};

#undef DEFINE_MAKEColorList
#undef DEFINE_ColorList2
#undef DEFINE_ColorList

#endif
