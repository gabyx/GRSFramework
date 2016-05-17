// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_renderer_RenderExecutionGraph_hpp
#define GRSF_converters_renderer_RenderExecutionGraph_hpp

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/general/SimFileExecutionGraph.hpp"


namespace LogicNodes{
    class RenderScriptWriter;
};

class RenderMaterial;

class RenderExecutionGraph : public SimFileExecutionGraph
{
    public:
        using Base = SimFileExecutionGraph;
        using NodeGroups = Base::NodeGroups;

        RenderExecutionGraph(){};

        void setup();
        void initState(boost::filesystem::path outputFilePath, double time, unsigned int stateNr);
        void finalizeState();
        void addBodyState(RigidBodyStateAdd * s);

    private:
        std::unordered_set<LogicNodes::RenderScriptWriter *> m_scriptWritterNodes;
};


#endif // RenderExecutionGraph_hpp
