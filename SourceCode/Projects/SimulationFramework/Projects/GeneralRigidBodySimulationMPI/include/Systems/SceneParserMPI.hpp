#ifndef SceneParserMPI_hpp
#define SceneParserMPI_hpp

#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "AssertionDebug.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/filesystem.hpp>
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "SceneParser.hpp"

#include "MPICommunication.hpp"

#include "DynamicsSystemMPI.hpp"

#include "CommonFunctions.hpp"
#include "QuaternionHelpers.hpp"
#include "InertiaTensorCalculations.hpp"
#include "InitialConditionBodies.hpp"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>       // Output data structure
#include <assimp/postprocess.h> // Post processing flags

//#include "OgreMeshExtraction.hpp"

#define TIXML_USE_TICPP
#include "ticpp/ticpp.h"
//#include "tinyxml.h"

/*
* @Does not work yet, to implement a scene parser, implement everything starting from SimulationState, enter(), we exit the State, delete all Objects, and reinitialize with another system in XML format.
*
*/
template<typename TConfig>
class SceneParserMPI : public SceneParser<TConfig> {
public:

    DEFINE_CONFIG_TYPES_OF(TConfig)

    SceneParserMPI(boost::shared_ptr<DynamicsSystemType> pDynSys,
                   boost::shared_ptr<MPILayer::ProcessCommunicator<DynamicsSystemType> > procComm)
        : SceneParser<TConfig>(pDynSys), m_pProcCommunicator(procComm) {
        m_nGlobalSimBodies = 0;
    }

    bool parseScene( boost::filesystem::path file ) {
        using namespace std;
        m_currentParseFilePath = file;
        m_currentParseFileDir = m_currentParseFilePath.parent_path();

        m_pSimulationLog->logMessage("---> Parsing Scene...");
        LOG( m_pSimulationLog,"---> Scene Input file: "  << file.string() <<std::endl; );


        //Reset all variables
        m_nSimBodies = 0;
        m_nBodies = 0;
        m_nGlobalSimBodies = 0;
        m_globalMaxGroupId = 0;

        m_bodyList.clear();
        m_SimBodyInitStates.clear();


        try {
            m_xmlDoc.LoadFile(m_currentParseFilePath.string());

            m_pSimulationLog->logMessage("---> File successfully loaded ...");

            m_pSimulationLog->logMessage("---> Try to parse the scene ...");

            m_xmlRootNode = m_xmlDoc.FirstChild("DynamicsSystem");
            if(m_xmlRootNode) {
                ticpp::Node *node = NULL;

                node = node = m_xmlRootNode->FirstChild("MPISettings");
                processMPISettings(node);
                m_pSimulationLog->logMessage("---> Parsed MPISettings...");

                node = m_xmlRootNode->FirstChild("SceneSettings");
                this->processSceneSettings(node);
                m_pSimulationLog->logMessage("---> Parsed SceneSettings...");

                node = m_xmlRootNode->FirstChild("SceneObjects");
                this->processSceneObjects(node);
                m_pSimulationLog->logMessage("---> Parsed SceneObjects...");

                /*ticpp::Node * initialConditionAll = m_xmlRootNode->FirstChild("InitialCondition");
                processinitialConditionAll(initialConditionAll);*/

            } else {
                m_pSimulationLog->logMessage("---> No DynamicsSystem Node found in XML ...");
                return false;
            }

        } catch(ticpp::Exception& ex) {
            LOG(m_pSimulationLog,"---> Scene XML error: "  << ex.what() << std::endl;);
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

        LOG(m_pSimulationLog,"---> Process SceneObjects ..."<<std::endl;);

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
                throw ticpp::Exception("---> String conversion in processMPISettings: minPoint failed");
            }
            if(!Utilities::stringToVector3<PREC>(maxPoint,  topo->GetAttribute("maxPoint"))) {
                throw ticpp::Exception("---> String conversion in processMPISettings: maxPoint failed");
            }

            MyMatrix<unsigned int>::Vector3 dim;
            if(!Utilities::stringToVector3<unsigned int>(dim,  topo->GetAttribute("dimension"))) {
                throw ticpp::Exception("---> String conversion in processMPISettings: dimension failed");
            }
            // saftey check
            if(dim(0)*dim(1)*dim(2) != m_pProcCommunicator->getProcInfo()->getNProcesses()) {
                LOG(m_pSimulationLog,"---> Grid and Process Number do not match!: Grid: ("<< dim.transpose() << ")"<< " with: " << m_pProcCommunicator->getProcInfo()->getNProcesses() <<" Processes"<<std::endl; );
                sleep(2);
                throw ticpp::Exception("---> You have launched to many processes for the grid!");
            }

            m_pProcCommunicator->getProcInfo()->createProcTopoGrid(minPoint,maxPoint, dim);

        } else {
            throw ticpp::Exception("---> String conversion in MPISettings:ProcessTopology:type failed: not a valid setting");
        }

    }

    void processRigidBodies( ticpp::Node * rigidbodies ) {

        //Clear current body list;
        ticpp::Element* rigidBodiesEl = rigidbodies->ToElement();
        m_bodyList.clear();
        m_bodyListScales.clear();

         unsigned int instances = rigidbodies->ToElement()->GetAttribute<unsigned int>("instances");

        unsigned int groupId;
        if(rigidBodiesEl->HasAttribute("groupId")){
            groupId = rigidBodiesEl->GetAttribute<unsigned int>("groupId");
            m_globalMaxGroupId = std::max(m_globalMaxGroupId,groupId);
        }else{
            m_globalMaxGroupId++;
            groupId = m_globalMaxGroupId;
        }

        for(int i=0; i<instances; i++) {

            RigidBodyType * temp_ptr = new RigidBodyType(RigidBodyId::makeId(i, groupId));

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

            m_nGlobalSimBodies += m_bodyList.size();

            typename std::vector<RigidBodyType*>::iterator bodyIt;
            //LOG(m_pSimulationLog, "---> SIZE: " << m_bodyList.size() << std::endl)
            for(bodyIt= m_bodyList.begin(); bodyIt!=m_bodyList.end();  ) {
                // Check if Body belongs to the topology! // Check CoG!
                if(m_pProcCommunicator->getProcInfo()->getProcTopo()->belongsPointToProcess((*bodyIt)->m_r_S)) {

                    LOG(m_pSimulationLog, "---> Added Body with ID: " << RigidBodyId::getBodyIdString(*bodyIt)<< std::endl);
                    m_pDynSys->m_SimBodies.addBody((*bodyIt));

                    m_nSimBodies++;
                    m_nBodies++;

                    ++bodyIt;

                }else{
                     LOG(m_pSimulationLog, "---> Rejected Body with ID: " << RigidBodyId::getBodyIdString(*bodyIt)<< std::endl);
                    //Delete this body immediately!
                    delete *bodyIt;
                    bodyIt = m_bodyList.erase(bodyIt);
                }
            }





        } else if(m_eBodiesState == RigidBodyType::NOT_SIMULATED) {

           typename std::vector<RigidBodyType*>::iterator bodyIt;

            for(bodyIt= m_bodyList.begin(); bodyIt!=m_bodyList.end(); bodyIt++) {

                m_pDynSys->m_Bodies.addBody((*bodyIt));

                m_nBodies++;
            }
        } else {
            throw ticpp::Exception("---> Adding only simulated and not simulated objects supported!");
        }


        ticpp::Node * visualizationNode = rigidbodies->FirstChild("Visualization");
        this->processVisualization( visualizationNode);


        //Remove all bodies from the sceneparsers intern list!
        m_bodyList.clear();
        m_bodyListScales.clear();
    }



    boost::shared_ptr< MPILayer::ProcessCommunicator<DynamicsSystemType> > m_pProcCommunicator;

    unsigned int m_nGlobalSimBodies;

    //using declerations
    using SceneParser<TConfig>::m_globalMaxGroupId;
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


};


#endif
