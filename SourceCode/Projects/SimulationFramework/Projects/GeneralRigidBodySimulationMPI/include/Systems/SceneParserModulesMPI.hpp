#ifndef SceneParserModulesMPI_hpp
#define SceneParserModulesMPI_hpp

#include "MPITopologyBuilderSettings.hpp"

namespace ParserModules{

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
        PREC splitNodeUpdateRatio;
        if(!Utilities::stringToType(splitNodeUpdateRatio,  incSet.attribute("splitNodeUpdateRatio").value())) {
                THROWEXCEPTION("---> String conversion in MPISettings::InclusionSolverSettings: splitNodeUpdateRatio failed");
        }
        if(splitNodeUpdateRatio <= 0){
            THROWEXCEPTION("---> MPISettings::InclusionSolverSettings: splitNodeUpdateRatio <= 0");
        }

        PREC convergenceCheckRatio;
        if(!Utilities::stringToType(convergenceCheckRatio,  incSet.attribute("convergenceCheckRatio").value())) {
                THROWEXCEPTION("---> String conversion in MPISettings::InclusionSolverSettings: convergenceCheckRatio failed");
        }
        if(convergenceCheckRatio <= 0){
            THROWEXCEPTION("---> MPISettings::InclusionSolverSettings: convergenceCheckRatio <= 0");
        }

        this->m_inclusionSettings->m_splitNodeUpdateRatio = splitNodeUpdateRatio;
        this->m_inclusionSettings->m_convergenceCheckRatio = convergenceCheckRatio;

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

        std::string type = procTopo.attribute("type").value();

        if(type=="grid") {

            m_topoSettings->m_type = TopologyBuilderSettingsType::TopologyBuilderEnumType::GRIDBUILDER;

            if(!Utilities::stringToVector3(m_topoSettings->m_gridBuilderSettings.m_processDim
                                           ,  procTopo.attribute("dimension").value())) {
                THROWEXCEPTION("---> String conversion in parseMPISettings: dimension failed");
            }

            std::string type = procTopo.attribute("mode").value();
            if(type=="static") {
                m_topoSettings->m_gridBuilderSettings.m_mode = MPILayer::GridBuilderSettings::Mode::STATIC;
                if(!Utilities::stringToVector3(m_topoSettings->m_gridBuilderSettings.m_minPoint
                                               ,  procTopo.attribute("minPoint").value())) {
                    THROWEXCEPTION("---> String conversion in parseMPISettings: minPoint failed");
                }
                if(!Utilities::stringToVector3(m_topoSettings->m_gridBuilderSettings.m_maxPoint
                                               ,  procTopo.attribute("maxPoint").value())) {
                    THROWEXCEPTION("---> String conversion in parseMPISettings: maxPoint failed");
                }

                if(!(m_topoSettings->m_gridBuilderSettings.m_minPoint.array() < m_topoSettings->m_gridBuilderSettings.m_maxPoint.array()).all()){
                        THROWEXCEPTION("parseMPISettings: Infeasible min/max points")
                }

            }else if(type=="dynamic"){
                m_topoSettings->m_gridBuilderSettings.m_mode = MPILayer::GridBuilderSettings::Mode::DYNAMIC;
            }else{
                THROWEXCEPTION("---> String conversion in MPISettings:ProcessTopology:mode failed: not a valid setting");
            }

        } else {
            THROWEXCEPTION("---> String conversion in MPISettings:ProcessTopology:type failed: not a valid setting");
        }
        LOGSCLEVEL1(this->m_pSimulationLog, "==================================================================="<<std::endl;)
    }

};

};
#endif
