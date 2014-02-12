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
        m_bodyListGroup.clear();
        m_bodyScalesGroup.clear();
        m_initStatesGroup.clear();

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


        // Filter all bodies according to MPI Grid
        m_pSimulationLog->logMessage("---> Filter bodies ...");
        filterBodies();

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

    virtual void setupInitialConditionBodiesFromFile_imp(boost::filesystem::path relpath, short which, double time){

        InitialConditionBodies::setupInitialConditionBodiesFromFile(relpath,m_pDynSys->m_simBodiesInitStates,time,true,true,which);
        LOG(m_pSimulationLog,"---> Found time: "<< time << " in " << relpath << std::endl;);
        m_pDynSys->applyInitStatesToBodies();
    }

    void processRigidBodies( ticpp::Node * rigidbodies ) {

        //Clear current body list;
        ticpp::Element* rigidBodiesEl = rigidbodies->ToElement();
        m_bodyListGroup.clear();
        m_bodyScalesGroup.clear();
        m_initStatesGroup.clear();

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

            m_bodyListGroup.push_back(temp_ptr);

            Vector3 scale;
            scale.setOnes();
            m_bodyScalesGroup.push_back(scale);
        }


        ticpp::Node * geometryNode = rigidbodies->FirstChild("Geometry");
        this->processGeometry(geometryNode);


        ticpp::Node * dynPropNode = rigidbodies->FirstChild("DynamicProperties");
        this->processDynamicProperties(dynPropNode);


        //Copy the pointers!

        if(m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {

            m_nGlobalSimBodies += m_bodyListGroup.size();

            typename std::vector<RigidBodyType*>::iterator bodyIt;
            //LOG(m_pSimulationLog, "---> SIZE: " << m_bodyListGroup.size() << std::endl)
            for(bodyIt= m_bodyListGroup.begin(); bodyIt!=m_bodyListGroup.end();  ) {
                    if(! m_pDynSys->m_SimBodies.addBody((*bodyIt))){
                        ERRORMSG("Could not add body to m_SimBodies! Id: " << RigidBodyId::getBodyIdString(*bodyIt) << " already in map!");
                    };
                    LOG(m_pSimulationLog, "---> Added Body with ID: " << RigidBodyId::getBodyIdString(*bodyIt)<< std::endl);


                    m_nSimBodies++;
                    m_nBodies++;

                    ++bodyIt;
            }

            // Copy all init states
            LOG(m_pSimulationLog, "---> Copy init states... " << std::endl;);
            for(auto it = m_initStatesGroup.begin(); it!=m_initStatesGroup.end();it++){
                LOG(m_pSimulationLog, "\t---> state id: " << RigidBodyId::getBodyIdString(it->first)
                    << std::endl << "\t\t---> q: " << it->second.m_q.transpose()
                    << std::endl << "\t\t---> u: " << it->second.m_u.transpose() << std::endl;
                    );
            }
            m_pDynSys->m_simBodiesInitStates.insert( m_initStatesGroup.begin(), m_initStatesGroup.end() );



        } else if(m_eBodiesState == RigidBodyType::BodyState::STATIC) {

           typename std::vector<RigidBodyType*>::iterator bodyIt;

            for(bodyIt= m_bodyListGroup.begin(); bodyIt!=m_bodyListGroup.end(); bodyIt++) {

                if(! m_pDynSys->m_Bodies.addBody((*bodyIt))){
                        ERRORMSG("Could not add body to m_Bodies! Id: " << RigidBodyId::getBodyIdString(*bodyIt) << " already in map!");
                };

                m_nBodies++;
            }
        } else {
            throw ticpp::Exception("---> Adding only simulated and not simulated objects supported!");
        }


//        ticpp::Node * visualizationNode = rigidbodies->FirstChild("Visualization");
//        this->processVisualization( visualizationNode);


        //Remove all bodies from the sceneparsers intern list!
        m_bodyListGroup.clear();
        m_bodyScalesGroup.clear();
        m_initStatesGroup.clear();
    }

    void filterBodies(){

        auto & simBodies = m_pDynSys->m_SimBodies;
        for(auto bodyIt= simBodies.begin(); bodyIt!=simBodies.end();
        /*No incremente because we delete inside the loop invalidating iterators*/ ) {
        // Check if Body belongs to the topology! // Check CoG!
                if(!m_pProcCommunicator->getProcInfo()->getProcTopo()->belongsPointToProcess((*bodyIt)->m_r_S)) {
                    LOG(m_pSimulationLog, "---> Reject Body with ID: " << RigidBodyId::getBodyIdString(*bodyIt)<< std::endl);

                    // Delete the init state
                    m_pDynSys->m_simBodiesInitStates.erase((*bodyIt)->m_id);
                    // Delete this body immediately!
                    bodyIt = simBodies.deleteBody(bodyIt);

                    m_nSimBodies--;
                }else{
                    bodyIt++;
                }
        }
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
    using SceneParser::m_bodyListGroup;
    using SceneParser::m_bodyScalesGroup;
    using SceneParser::m_initStatesGroup;

};


#endif
