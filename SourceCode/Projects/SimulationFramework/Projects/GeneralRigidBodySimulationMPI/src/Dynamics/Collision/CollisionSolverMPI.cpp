
#include "CollisionSolverMPI.hpp"

#include "PrintGeometryDetails.hpp"

CollisionSolverMPI::CollisionSolverMPI(std::shared_ptr< DynamicsSystemType> pDynSys):
    m_SimBodies(pDynSys->m_SimBodies), m_Bodies(pDynSys->m_Bodies), m_RemoteSimBodies(pDynSys->m_RemoteSimBodies),
    m_Collider(&m_collisionSet)
{
    m_expectedNContacts = 300;
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

    m_expectedNContacts =  m_SimBodies.size() * 3;


    m_maxOverlap = 0;

}


void CollisionSolverMPI::clearCollisionSet() {
    for( typename CollisionSetType::iterator it = m_collisionSet.begin(); it != m_collisionSet.end(); ++it) {
        delete (*it);
    }
    m_collisionSet.clear();
}


const typename CollisionSolverMPI::CollisionSetType &
CollisionSolverMPI::getCollisionSetRef()
{
    return m_collisionSet;
}



void CollisionSolverMPI::solveCollision() {


    reset();


    LOGSLLEVEL2(m_pSolverLog, "---> solveCollision(): "<<std::endl;)

    // All objects have been updated...

    // Do simple collision detection (SimBodies to SimBodies)
    LOGSLLEVEL2(m_pSolverLog, "\t---> SimBodies to SimBodies "<<std::endl;)
    if(m_SimBodies.size()){
        for(auto bodyIti = m_SimBodies.begin(); bodyIti != --m_SimBodies.end(); bodyIti++) {
            auto bodyItj = bodyIti;
            bodyItj++;
            for(; bodyItj != m_SimBodies.end(); bodyItj++ ) {

                //check for a collision
                m_Collider.checkCollision((*bodyIti), (*bodyItj));

            }
        }
    }

    //// Do simple collision detection (SimBodies to RemoteSimBodies)
    LOGSLLEVEL2(m_pSolverLog, "\t---> SimBodies to RemoteBodies "<<std::endl;)
    for(auto bodyIti = m_SimBodies.begin(); bodyIti != m_SimBodies.end(); bodyIti++) {
        for(auto bodyItj = m_RemoteSimBodies.begin(); bodyItj != m_RemoteSimBodies.end(); bodyItj++ ) {
            //check for a collision
            m_Collider.checkCollision((*bodyIti), (*bodyItj));

        }
    }


    // Do simple collision detection (RemoteSimBodies to RemoteSimBodies, but only different rank!)
    LOGSLLEVEL2(m_pSolverLog, "\t---> RemoteSimBodies to RemoteSimBodies (different rank) "<<std::endl;)
    if(m_RemoteSimBodies.size()){
        for(auto bodyIti = m_RemoteSimBodies.begin(); bodyIti != --m_RemoteSimBodies.end(); bodyIti++) {
            auto bodyItj = bodyIti;
            bodyItj++;
            for(; bodyItj != m_RemoteSimBodies.end(); bodyItj++ ) {

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
    for(auto bodyIti = m_SimBodies.begin(); bodyIti != m_SimBodies.end(); bodyIti++) {
        for(auto bodyItk = m_Bodies.begin(); bodyItk != m_Bodies.end(); bodyItk ++) {
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

        for( auto colDataIt = m_collisionSet.begin(); colDataIt != m_collisionSet.end(); colDataIt++ ){

            ASSERTMSG( std::abs((*colDataIt)->m_cFrame.m_e_x.dot((*colDataIt)->m_cFrame.m_e_y)) < 1e-3 &&
                      std::abs((*colDataIt)->m_cFrame.m_e_y.dot((*colDataIt)->m_cFrame.m_e_z))< 1e-3, "Vectors not orthogonal");

            LOGSLLEVEL3_CONTACT(m_pSolverLog,"---> Contact Frame: n: " << (*colDataIt)->m_cFrame.m_e_z.transpose() << std::endl;)

            //Set contact frame point
            (*colDataIt)->m_cFrame.m_p = (*colDataIt)->m_pBody1->m_r_S + (*colDataIt)->m_r_S1C1;

            // Calculate some Statistics
            m_maxOverlap = std::max(m_maxOverlap,(*colDataIt)->m_overlap);

            invokeAllContactDelegates(*colDataIt); // Propagate pointers! they will not be deleted!

        }

    }
}
