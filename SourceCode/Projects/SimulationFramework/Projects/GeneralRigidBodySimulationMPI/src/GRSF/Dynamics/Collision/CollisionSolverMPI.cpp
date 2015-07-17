
#include "GRSF/Dynamics/Collision/CollisionSolverMPI.hpp"

#include "GRSF/Dynamics/General/PrintGeometryDetails.hpp"

CollisionSolverMPI::CollisionSolverMPI(std::shared_ptr< DynamicsSystemType> pDynSys):
    m_simBodies(pDynSys->m_simBodies), m_staticBodies(pDynSys->m_staticBodies), m_remoteSimBodies(pDynSys->m_remoteSimBodies),
    m_Collider(&m_collisionSet)
{

}


CollisionSolverMPI::~CollisionSolverMPI() {
    clearCollisionSet();
}



void CollisionSolverMPI::initializeLog( Logging::Log* pSolverLog ) {
    m_pSolverLog = pSolverLog;
    ASSERTMSG(m_pSolverLog != nullptr, "Logging::Log: nullptr!");
}



void CollisionSolverMPI::reset() {
    clearCollisionSet();
    removeAllContactDelegates();
    m_maxOverlap = 0;
}

void CollisionSolverMPI::resetTopology() {
    clearCollisionSet();
    m_maxOverlap = 0;
}


void CollisionSolverMPI::clearCollisionSet() {
    m_collisionSet.clear();
}


const typename CollisionSolverMPI::CollisionSetType &
CollisionSolverMPI::getCollisionSetRef()
{
    return m_collisionSet;
}



void CollisionSolverMPI::solveCollision() {


    clearCollisionSet();
    m_maxOverlap = 0;


    LOGSLLEVEL1(m_pSolverLog, "---> solveCollision(): "<<std::endl;)

    // All objects have been updated...

    // Do simple collision detection (SimBodies to SimBodies)
    LOGSLLEVEL2(m_pSolverLog, "\t---> SimBodies to SimBodies "<<std::endl;)
    if(m_simBodies.size()){
        for(auto bodyIti = m_simBodies.begin(); bodyIti != --m_simBodies.end(); bodyIti++) {
            auto bodyItj = bodyIti;
            bodyItj++;
            for(; bodyItj != m_simBodies.end(); bodyItj++ ) {

                //check for a collision
                m_Collider.checkCollision((*bodyIti), (*bodyItj));

            }
        }
    }

    //// Do simple collision detection (SimBodies to RemoteSimBodies)
    LOGSLLEVEL2(m_pSolverLog, "\t---> SimBodies to RemoteBodies "<<std::endl;)
    for(auto bodyIti = m_simBodies.begin(); bodyIti != m_simBodies.end(); bodyIti++) {
        for(auto bodyItj = m_remoteSimBodies.begin(); bodyItj != m_remoteSimBodies.end(); bodyItj++ ) {
            //check for a collision
            m_Collider.checkCollision((*bodyIti), (*bodyItj));

        }
    }


    // Do simple collision detection (RemoteSimBodies to RemoteSimBodies, but only different rank!)
    LOGSLLEVEL2(m_pSolverLog, "\t---> RemoteSimBodies to RemoteSimBodies (different rank) "<<std::endl;)
    if(m_remoteSimBodies.size()){
        for(auto bodyIti = m_remoteSimBodies.begin(); bodyIti != --m_remoteSimBodies.end(); bodyIti++) {
            auto bodyItj = bodyIti;
            bodyItj++;
            for(; bodyItj != m_remoteSimBodies.end(); bodyItj++ ) {

                //check for a collision

//                PrintGeometryDetailsVisitor _2(m_pSolverLog, (*bodyIti)->m_geometry, "--->");
//                PrintGeometryDetailsVisitor _1(m_pSolverLog, (*bodyItj)->m_geometry, "--->");

                if((*bodyIti)->m_pBodyInfo->m_ownerRank !=  (*bodyItj)->m_pBodyInfo->m_ownerRank){

                    m_Collider.checkCollision((*bodyIti), (*bodyItj));
                }
            }
        }
    }


    LOGSLLEVEL2(m_pSolverLog, "\t---> SimBodies to Bodies "<<std::endl;)
    // Do simple collision detection (SimBodies to Bodies)
    for(auto bodyIti = m_simBodies.begin(); bodyIti != m_simBodies.end(); bodyIti++) {
        for(auto bodyItk = m_staticBodies.begin(); bodyItk != m_staticBodies.end(); bodyItk ++) {
                //check for a collision and signal
                m_Collider.checkCollision((*bodyIti), (*bodyItk));
        }
    }

    LOGSLLEVEL2(m_pSolverLog, "\t---> Collision done "<<std::endl;)

    // Signal all found contact
    signalContactAdd();

}


std::string CollisionSolverMPI::getIterationStats() {
    std::stringstream s;
    s << m_maxOverlap;
    return s.str();
}
std::string CollisionSolverMPI::getStatsHeader() {
    std::stringstream s;
    s << "MaxOverlap [m]";
    return s.str();
}



void CollisionSolverMPI::signalContactAdd() {

    if(m_collisionSet.size()!=0){

        for( auto * colDataPtr : m_collisionSet){

            ASSERTMSG( std::abs(colDataPtr->m_cFrame.m_e_x.dot(colDataPtr->m_cFrame.m_e_y)) < 1e-3 &&
                       std::abs(colDataPtr->m_cFrame.m_e_y.dot(colDataPtr->m_cFrame.m_e_z))< 1e-3, "Vectors not orthogonal");

            LOGSLLEVEL3_CONTACT(m_pSolverLog,"---> Contact Frame: n: " << colDataPtr->m_cFrame.m_e_z.transpose() << std::endl;)

            //Set contact frame point
            colDataPtr->m_cFrame.m_p = colDataPtr->m_pBody[0]->m_r_S + colDataPtr->m_r_SC[0];

            // Calculate some Statistics
            m_maxOverlap = std::max(m_maxOverlap,colDataPtr->m_overlap);

            invokeAllContactDelegates(colDataPtr); // Propagate pointers! they will not be deleted!
        }

    }
}
