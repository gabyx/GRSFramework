#ifndef GRSF_Systems_SceneParserModulesMPI_hpp
#define GRSF_Systems_SceneParserModulesMPI_hpp

#include "GRSF/Dynamics/General/MPITopologyBuilderSettings.hpp"

namespace ParserModules {

template<typename TParserTraits>
class SettingsModuleMPI : public SettingsModule<TParserTraits> {
private:
    DEFINE_PARSER_TYPE_TRAITS(TParserTraits )
    DEFINE_LAYOUT_CONFIG_TYPES

    using RecorderSettingsType = typename SettingsModule<TParserTraits>::RecorderSettingsType ;
    using TimeStepperSettingsType = typename SettingsModule<TParserTraits>::TimeStepperSettingsType     ;
    using InclusionSolverSettingsType = typename SettingsModule<TParserTraits>::InclusionSolverSettingsType ;



public:
    SettingsModuleMPI(ParserType * p, RecorderSettingsType * r, TimeStepperSettingsType * t, InclusionSolverSettingsType * i)
        :SettingsModule<TParserTraits>(p,r,t,i) {};

    void parseOtherOptions(XMLNodeType sceneSettings) {
        LOGSCLEVEL1(this->m_pSimulationLog, "==== SettingsModuleMPI: parsing other options ====================="<<std::endl;)

        XMLNodeType incSet = sceneSettings.child("MPISettings").child("InclusionSolverSettings");
        CHECK_XMLNODE(incSet,"MPISettings/InclusionSolverSettings does not exist");
        // Process special Inclusion solver settings

        if(!Utilities::stringToType(this->m_inclusionSettings->m_splitNodeUpdateRatio,  incSet.attribute("splitNodeUpdateRatio").value())) {
            ERRORMSG("---> String conversion in MPISettings::InclusionSolverSettings: splitNodeUpdateRatio failed");
        }
        if(!Utilities::stringToType(this->m_inclusionSettings->m_convergenceCheckRatio,  incSet.attribute("convergenceCheckRatio").value())) {
            ERRORMSG("---> String conversion in MPISettings::InclusionSolverSettings: convergenceCheckRatio failed");
        }

        XMLAttributeType att = incSet.attribute("reserveSplitNodes");
        if(att) {
            if(!Utilities::stringToType(this->m_inclusionSettings->m_reserveSplitNodes, att.value())) {
                ERRORMSG("---> String conversion in MPISettings::InclusionSolverSettings: m_reserveSplitNodes failed");
            }
        }


        LOGSCLEVEL1(this->m_pSimulationLog, "==================================================================="<<std::endl;)
    }
};

template<typename TParserTraits>
class MPIModule {
private:
    DEFINE_PARSER_TYPE_TRAITS(TParserTraits )
    DEFINE_LAYOUT_CONFIG_TYPES

    LogType * m_pSimulationLog;

    using TopologyBuilderSettingsType = typename DynamicsSystemType::TopologyBuilderSettingsType;
    TopologyBuilderSettingsType * m_topoSettings;

public:
    MPIModule(ParserType * p, BodyModuleType * b, TopologyBuilderSettingsType * t)
        :m_pSimulationLog(p->getSimLog()), m_topoSettings(t) {}

    void parseSceneSettingsPost(XMLNodeType sceneSettings) {
        LOGSCLEVEL1(this->m_pSimulationLog, "==== ModuleMPI: parsing scene settings ============================"<<std::endl;)
        XMLNodeType procTopo = sceneSettings.child("MPISettings").child("ProcessTopology");
        CHECK_XMLNODE(procTopo,"MPISettings/ProcessTopology does not exist");



        std::string type = procTopo.attribute("mode").value();
        if(type=="static") {

            m_topoSettings->m_rebuildSettings.m_mode = TopologyBuilderSettingsType::RebuildSettings::Mode::STATIC;

        } else if(type=="dynamic") {

            m_topoSettings->m_rebuildSettings.m_mode = TopologyBuilderSettingsType::RebuildSettings::Mode::DYNAMIC;

            unsigned int policyCheck = 1;
            if(!Utilities::stringToType(policyCheck, procTopo.attribute("policyCheckEveryXTimeStep").value())) {
                ERRORMSG("---> String conversion in MPISettings::ProcessTopology: policyCheckEveryXTimeStep failed");
            }
            m_topoSettings->m_rebuildSettings.m_policyCheckEachXTimeStep = policyCheck;

            type = procTopo.attribute("policy").value();
            if(type=="alwaysRebuild") {

                m_topoSettings->m_rebuildSettings.m_policy = TopologyBuilderSettingsType::RebuildSettings::Policy::ALWAYS_REBUILD;

            } else if(type=="bodyLimit") {

                m_topoSettings->m_rebuildSettings.m_policy = TopologyBuilderSettingsType::RebuildSettings::Policy::BODY_LIMIT;
                unsigned int bodyLimit = 1;
                if(!Utilities::stringToType(bodyLimit, procTopo.attribute("bodyLimit").value())) {
                    ERRORMSG("---> String conversion in MPISettings::ProcessTopology: bodyLimit failed");
                }
                m_topoSettings->m_rebuildSettings.m_bodyLimit = bodyLimit;

            } else {
                ERRORMSG("---> String conversion in MPISettings:ProcessTopology: policy failed: not a valid setting");
            }

        } else {
            ERRORMSG("---> String conversion in MPISettings:ProcessTopology: mode failed: not a valid setting");
        }

        XMLNodeType topo = procTopo.child("Topology");
        CHECK_XMLNODE(procTopo,"MPISettings/ProcessTopology/Topology does not exist");
        type = topo.attribute("type").value();

        if(type=="grid") {

            m_topoSettings->m_type = TopologyBuilderSettingsType::TopologyBuilderEnumType::GRIDBUILDER;

            if(!Utilities::stringToVector3(m_topoSettings->m_gridBuilderSettings.m_processDim
                                           ,  topo.attribute("dimension").value())) {
                ERRORMSG("---> String conversion in parseMPISettings: dimension failed");
            }

            XMLAttributeType att = topo.attribute("matchProcessDimToExtent");
            bool matchProcessDimToExtent = true;
            if(att) {
               if(!Utilities::stringToType(matchProcessDimToExtent, att.value())) {
                    ERRORMSG("---> String conversion in MPISettings::ProcessTopology: matchProcessDimToExtent failed");
               }
            }
            m_topoSettings->m_gridBuilderSettings.m_matchProcessDimToExtent = matchProcessDimToExtent;



            type = topo.attribute("buildMode").value();
            if(type=="Predefined" || type=="predefined") {

                m_topoSettings->m_gridBuilderSettings.m_buildMode = MPILayer::GridBuilderSettings::BuildMode::BINET_TENSOR;

                if(!Utilities::stringToType(m_topoSettings->m_gridBuilderSettings.m_aligned,
                                            topo.attribute("aligned").value())) {
                    ERRORMSG("---> String conversion in parseMPISettings: aligned failed");
                }

                if(m_topoSettings->m_gridBuilderSettings.m_aligned == false) {
                    ERRORMSG("Parse in here a rotation matrix: not implemented!")
                }
                Vector3 minPoint;
                if(!Utilities::stringToVector3(minPoint
                                               ,  topo.attribute("minPoint").value())) {
                    ERRORMSG("---> String conversion in parseMPISettings: minPoint failed");
                }
                Vector3 maxPoint;
                if(!Utilities::stringToVector3(maxPoint
                                               ,  topo.attribute("maxPoint").value())) {
                    ERRORMSG("---> String conversion in parseMPISettings: maxPoint failed");
                }
                m_topoSettings->m_gridBuilderSettings.m_aabb += minPoint;
                m_topoSettings->m_gridBuilderSettings.m_aabb += maxPoint;

                if(m_topoSettings->m_gridBuilderSettings.m_aabb.isEmpty()) {
                    ERRORMSG("parseMPISettings: Infeasible min/max points");
                }

                if(m_topoSettings->m_rebuildSettings.m_mode == TopologyBuilderSettingsType::RebuildSettings::Mode::DYNAMIC){
                    ERRORMSG("---> You defined a predefined grid as topology and using dynamic rebuilding! You should not do this!")
                }

            } else if(type=="BinetTensor") {
                m_topoSettings->m_gridBuilderSettings.m_buildMode = MPILayer::GridBuilderSettings::BuildMode::BINET_TENSOR;
            } else if(type=="MinimalVolumeBoundingBox" || type=="MVBB") {
                m_topoSettings->m_gridBuilderSettings.m_buildMode = MPILayer::GridBuilderSettings::BuildMode::MVBB;
            } else if(type=="Aligned" || type =="AABB") {
                m_topoSettings->m_gridBuilderSettings.m_buildMode = MPILayer::GridBuilderSettings::BuildMode::ALIGNED;
            } else {
                ERRORMSG("---> String conversion in MPISettings:ProcessTopology:buildMode failed: not a valid setting");
            }


            PREC minGridSize = 0;
            if(!Utilities::stringToType(minGridSize, topo.attribute("minGridSize").value())) {
                ERRORMSG("---> String conversion in MPISettings::ProcessTopology: minGridSize failed");
            }
            m_topoSettings->m_gridBuilderSettings.m_minGridSize = minGridSize;


        } else {
            ERRORMSG("---> String conversion in MPISettings:ProcessTopology: type failed: not a valid setting");
        }


        XMLNodeType massP = procTopo.child("MassPointPrediction");
        if(massP){

            unsigned int points = 5;
            if(!Utilities::stringToType(points, massP.attribute("nPoints").value())) {
                ERRORMSG("---> String conversion in MPISettings::ProcessTopology: nPoints failed");
            }
            m_topoSettings->m_massPointPredSettings.m_nPoints = points;

            PREC deltaT = 0.1;
            if(!Utilities::stringToType(deltaT, massP.attribute("deltaT").value())) {
                ERRORMSG("---> String conversion in MPISettings::ProcessTopology: nPoints failed");
            }
            m_topoSettings->m_massPointPredSettings.m_deltaT = deltaT;
        }

        LOGSCLEVEL1(this->m_pSimulationLog, "==================================================================="<<std::endl;)
    }

};

};
#endif
