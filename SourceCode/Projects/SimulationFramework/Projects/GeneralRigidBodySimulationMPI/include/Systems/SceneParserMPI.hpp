#ifndef SceneParserMPI_hpp
#define SceneParserMPI_hpp

#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "AssertionDebug.hpp"


#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/filesystem.hpp>
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "SceneParser.hpp"

#include "MPIInformation.hpp"

#include "DynamicsSystemMPI.hpp"

#include "CommonFunctions.hpp"
#include "QuaternionHelpers.hpp"
#include "InertiaTensorCalculations.hpp"
#include "InitialConditionBodies.hpp"

#include <assimp.hpp>      // C++ importer interface
#include <aiScene.h>       // Output data structure
#include <aiPostProcess.h> // Post processing flags

//#include "OgreMeshExtraction.hpp"

#define TIXML_USE_TICPP
#include "ticpp.h"
//#include "tinyxml.h"

/*
* @Does not work yet, to implement a scene parser, implement everything starting from SimulationState, enter(), we exit the State, delete all Objects, and reinitialize with another system in XML format.
*
*/
template<typename TConfig>
class SceneParserMPI : public SceneParser<TConfig> {
public:

    DEFINE_CONFIG_TYPES_OF(TConfig)

    SceneParserMPI(boost::shared_ptr<DynamicsSystemType> pDynSys, MPILayer::ProcessInformation<LayoutConfigType> & procInfo)
        : SceneParser<TConfig>(pDynSys), m_procInfo(procInfo) {
        m_nGlobalSimBodies = 0;
    }

    bool parseScene( boost::filesystem::path file ) {
        using namespace std;
        m_currentParseFilePath = file;
        m_currentParseFileDir = m_currentParseFilePath.parent_path();

        m_pSimulationLog->logMessage("Parsing Scene...");

        LOG( m_pSimulationLog,"Scene Input file: "  << file.string() <<std::endl; );
        m_pSimulationLog->logMessage("Parsing Scene...");


        //Reset all variables
        m_nSimBodies = 0;
        m_nBodies = 0;
        m_nGlobalSimBodies = 0;

        m_bodyList.clear();
        m_SimBodyInitStates.clear();
        m_SceneMeshs.clear();


        try {
            m_xmlDoc.LoadFile(m_currentParseFilePath.string());

            m_pSimulationLog->logMessage("File successfully loaded ...");

            m_pSimulationLog->logMessage("Try to parse the scene ...");

            // Start off with the gravity!
            m_xmlRootNode = m_xmlDoc.FirstChild("DynamicsSystem");
            if(m_xmlRootNode) {
                ticpp::Node *node = NULL;

                node = node = m_xmlRootNode->FirstChild("MPISettings");
                processMPISettings(node);
                m_pSimulationLog->logMessage("Parsed MPISettings...");

                node = m_xmlRootNode->FirstChild("SceneSettings");
                this->processSceneSettings(node);
                m_pSimulationLog->logMessage("Parsed SceneSettings...");

                node = m_xmlRootNode->FirstChild("SceneObjects");
                this->processSceneObjects(node);
                m_pSimulationLog->logMessage("Parsed SceneObjects...");

                /*ticpp::Node * initialConditionAll = m_xmlRootNode->FirstChild("InitialCondition");
                processinitialConditionAll(initialConditionAll);*/

            } else {
                m_pSimulationLog->logMessage("No DynamicsSystem Node found in XML ...");
                return false;
            }

        } catch(ticpp::Exception& ex) {
            LOG(m_pSimulationLog,"Scene XML error: "  << ex.what() << std::endl;);
            exit(-1);
        }


        //ASSERTMSG(false,"XML parsing...");

        return true;
    }



//    boost::filesystem::path getParsedSceneFile() {
//        return m_currentParseFilePath;
//    }
//
//    const std::vector< DynamicsState<LayoutConfigType> > & getInitialConditionSimBodies() {
//        return m_SimBodyInitStates;
//    }
//
//    unsigned int getNumberOfSimBodies() {
//        return m_nSimBodies;
//    }

    unsigned int getNumberOfGlobalSimBodies() {
        return m_nGlobalSimBodies;
    }


protected:


    void processSceneObjects( ticpp::Node *sceneObjects) {

        LOG(m_pSimulationLog,"Process SceneObjects ..."<<std::endl;);

        ticpp::Iterator< ticpp::Node > child;

        for ( child = child.begin( sceneObjects ); child != child.end(); child++ ) {

            if( child->Value() == "RigidBodies") {
                processRigidBodies( &(*child) );
            }

        }
    }

    void processMPISettings( ticpp::Node *mpiSettings ) {
        ticpp::Element *topo = mpiSettings->FirstChild("ProcessTopology",true)->ToElement();

        std::string type = topo->GetAttribute("type");
        if(type=="grid") {

            Vector3 minPoint, maxPoint;
            if(!Utilities::stringToVector3<PREC>(minPoint,  topo->GetAttribute("minPoint"))) {
                throw ticpp::Exception("String conversion in processMPISettings: minPoint failed");
            }
            if(!Utilities::stringToVector3<PREC>(maxPoint,  topo->GetAttribute("maxPoint"))) {
                throw ticpp::Exception("String conversion in processMPISettings: maxPoint failed");
            }

            MyMatrix<unsigned int>::Vector3 dim;
            if(!Utilities::stringToVector3<unsigned int>(dim,  topo->GetAttribute("dimension"))) {
                throw ticpp::Exception("String conversion in processMPISettings: dimension failed");
            }
            // saftey check
            if(dim(0)*dim(1)*dim(2) != m_procInfo.getNProcesses()) {
                LOG(m_pSimulationLog, "Grid and Process Number do not match!: Grid: ("<< dim.transpose() << ")"<< " with: " << m_procInfo.getNProcesses() <<" Processes"<<std::endl; );
                throw ticpp::Exception("You have launched to many processes for the grid!");
            }

            m_procInfo.setProcTopo( new MPILayer::ProcessTopologyGrid<LayoutConfigType>(minPoint,maxPoint,dim,m_procInfo.getRank()) );

        } else {
            throw ticpp::Exception("String conversion in MPISettings:ProcessTopology:type failed: not a valid setting");
        }

    }

    void processRigidBodies( ticpp::Node * rigidbodies ) {

        //Clear current body list;

        m_bodyList.clear();
        m_bodyListScales.clear();

        int instances = rigidbodies->ToElement()->GetAttribute<int>("instances");


        for(int i=0; i<instances; i++) {
            boost::shared_ptr< RigidBodyType > temp_ptr(new RigidBodyType());
            temp_ptr->m_id = i;
            m_bodyList.push_back(temp_ptr);

            Vector3 scale;
            scale.setOnes();
            m_bodyListScales.push_back(scale);
        }


        ticpp::Node * geometryNode = rigidbodies->FirstChild("Geometry");
        this->processGeometry(geometryNode);


        ticpp::Node * dynPropNode = rigidbodies->FirstChild("DynamicProperties");
        this->processDynamicProperties(dynPropNode);



        //Copy the pointers!

        if(m_eBodiesState == RigidBodyType::SIMULATED) {

            typename std::vector<boost::shared_ptr< RigidBodyType > >::iterator bodyIt;

            for(bodyIt= m_bodyList.begin(); bodyIt!=m_bodyList.end(); bodyIt++) {
                // Check if Body belongs to the topology! // Check CoG!
                if(m_procInfo.getProcTopo().belongsPointToProcess((*bodyIt)->m_r_S)) {

                    //Assign a unique id
                    RigidBodyId::setId(bodyIt->get(),m_nBodies, m_procInfo.getRank());

                    LOG(m_pSimulationLog, "Added Body with ID: (" << RigidBodyId::getProcessNr(bodyIt->get())<<","<<RigidBodyId::getBodyNr(bodyIt->get())<<")"<< std::endl);

                    m_pDynSys->m_SimBodies.push_back((*bodyIt));

                    m_nSimBodies++;
                    m_nBodies++;

                }
            }
            m_nGlobalSimBodies += m_bodyList.size();


        } else if(m_eBodiesState == RigidBodyType::NOT_SIMULATED) {

           typename std::vector<boost::shared_ptr< RigidBodyType > >::iterator bodyIt;

            for(bodyIt= m_bodyList.begin(); bodyIt!=m_bodyList.end(); bodyIt++) {
                //Assign a unique id
                RigidBodyId::setId(bodyIt->get(),m_nBodies, m_procInfo.getRank());
                m_pDynSys->m_Bodies.push_back((*bodyIt));

                m_nBodies++;
            }
        } else {
            throw ticpp::Exception("Adding only simulated and not simulated objects supported!");
        }


        ticpp::Node * visualizationNode = rigidbodies->FirstChild("Visualization");
        this->processVisualization( visualizationNode);



        m_bodyList.clear();
        m_bodyListScales.clear();
    }



    MPILayer::ProcessInformation<LayoutConfigType> & m_procInfo;

    unsigned int m_nGlobalSimBodies;

    //using declerations
    using SceneParser<TConfig>::m_nSimBodies;
    using SceneParser<TConfig>::m_nBodies;
    using SceneParser<TConfig>::m_bParseDynamics;
    using SceneParser<TConfig>::m_pDynSys;
    using SceneParser<TConfig>::m_currentParseFilePath;
    using SceneParser<TConfig>::m_currentParseFileDir;
    using SceneParser<TConfig>::m_xmlDoc;
    using SceneParser<TConfig>::m_xmlRootNode;
    using SceneParser<TConfig>::m_pSimulationLog;
    using SceneParser<TConfig>::logstream;
    // Temprary structures
    using SceneParser<TConfig>::m_eBodiesState;
    using SceneParser<TConfig>::m_bodyList;
    using SceneParser<TConfig>::m_bodyListScales;
    using SceneParser<TConfig>::m_SimBodyInitStates;


    using SceneParser<TConfig>::m_SceneMeshs;

};


#endif
