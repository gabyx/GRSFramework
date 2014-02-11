#ifndef SceneParserMPI_hpp
#define SceneParserMPI_hpp

#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/filesystem.hpp>
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>       // Output data structure
#include <assimp/postprocess.h> // Post processing flags

#define TIXML_USE_TICPP
#include "ticpp/ticpp.h"
//#include "tinyxml.h"

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"
#include "LogDefines.hpp"

#include "SceneParser.hpp"
#include "MPICommunication.hpp"



class SceneParserMPI : public SceneParser {
public:

    DEFINE_CONFIG_TYPES

    SceneParserMPI(boost::shared_ptr<DynamicsSystemType> pDynSys,
                   boost::shared_ptr<MPILayer::ProcessCommunicator > procComm)
        : SceneParser(pDynSys), m_pProcCommunicator(procComm) {
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
        m_bodyListScales.clear();
        m_bodyInitStates.clear();

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

                node = m_xmlRootNode->FirstChild("SceneSettings");
                processSceneSettings2(node);
                m_pSimulationLog->logMessage("---> Parsed SceneSettings (second part)...");

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
        m_bodyInitStates.clear();

         unsigned int instances = rigidbodies->ToElement()->GetAttribute<unsigned int>("instances");

        unsigned int groupId, startIdx;
        if(rigidBodiesEl->HasAttribute("groupId")){
            groupId = rigidBodiesEl->GetAttribute<unsigned int>("groupId");
            m_globalMaxGroupId = std::max(m_globalMaxGroupId,groupId);
        }else{
            m_globalMaxGroupId++;
            groupId = m_globalMaxGroupId;
        }


        // Get the startidx for this group
        auto it = this->groupIdToNBodies.find(groupId);
        if( it == this->groupIdToNBodies.end()){
            this->groupIdToNBodies[groupId] = startIdx = 0;
        }else{
            startIdx = this->groupIdToNBodies[groupId];
        }

        // update the number of bodies
        this->groupIdToNBodies[groupId] += instances;


        for(int i=0; i<instances; i++) {

            RigidBodyType * temp_ptr = new RigidBodyType(RigidBodyId::makeId(startIdx+i, groupId));

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

        if(m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {

            m_nGlobalSimBodies += m_bodyList.size();

            typename std::vector<RigidBodyType*>::iterator bodyIt;
            //LOG(m_pSimulationLog, "---> SIZE: " << m_bodyList.size() << std::endl)
            for(bodyIt= m_bodyList.begin(); bodyIt!=m_bodyList.end();  ) {
                // Check if Body belongs to the topology! // Check CoG!
                if(m_pProcCommunicator->getProcInfo()->getProcTopo()->belongsPointToProcess((*bodyIt)->m_r_S)) {


                    if(! m_pDynSys->m_SimBodies.addBody((*bodyIt))){
                        ERRORMSG("Could not add body to m_SimBodies! Id: " << RigidBodyId::getBodyIdString(*bodyIt) << " already in map!");
                    };
                    LOG(m_pSimulationLog, "---> Added Body with ID: " << RigidBodyId::getBodyIdString(*bodyIt)<< std::endl);


                    m_nSimBodies++;
                    m_nBodies++;

                    ++bodyIt;

                }else{
                    LOG(m_pSimulationLog, "---> Rejected Body with ID: " << RigidBodyId::getBodyIdString(*bodyIt)<< std::endl);

                    m_bodyInitStates.erase((*bodyIt)->m_id);
                    //Delete this body immediately!
                    delete *bodyIt;
                    bodyIt = m_bodyList.erase(bodyIt);
                }
            }

            // Copy all init states
            LOG(m_pSimulationLog, "---> Copy init states... ");
//            for(auto it = m_bodyInitStates.begin(); it!=m_bodyInitStates.end();it++){
//                LOG(m_pSimulationLog, RigidBodyId::getBodyIdString(it->first));
//            }
            m_pDynSys->m_simBodiesInitStates.insert( m_bodyInitStates.begin(), m_bodyInitStates.end() );



        } else if(m_eBodiesState == RigidBodyType::BodyState::STATIC) {

           typename std::vector<RigidBodyType*>::iterator bodyIt;

            for(bodyIt= m_bodyList.begin(); bodyIt!=m_bodyList.end(); bodyIt++) {

                if(! m_pDynSys->m_Bodies.addBody((*bodyIt))){
                        ERRORMSG("Could not add body to m_Bodies! Id: " << RigidBodyId::getBodyIdString(*bodyIt) << " already in map!");
                };

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
        m_bodyInitStates.clear();
    }



    boost::shared_ptr< MPILayer::ProcessCommunicator > m_pProcCommunicator;

    unsigned int m_nGlobalSimBodies;

    //using declerations
    using SceneParser::m_globalMaxGroupId;
    using SceneParser::m_nSimBodies;
    using SceneParser::m_nBodies;
    using SceneParser::m_bParseDynamics;
    using SceneParser::m_pDynSys;
    using SceneParser::m_currentParseFilePath;
    using SceneParser::m_currentParseFileDir;
    using SceneParser::m_xmlDoc;
    using SceneParser::m_xmlRootNode;
    using SceneParser::m_pSimulationLog;
    using SceneParser::logstream;
    // Temprary structures
    using SceneParser::m_eBodiesState;
    using SceneParser::m_bodyList;
    using SceneParser::m_bodyListScales;
    using SceneParser::m_bodyInitStates;

};


#endif
