#include "GMSF/Dynamics/Collision/CollisionSolver.hpp"


CollisionSolver::CollisionSolver(std::shared_ptr< DynamicsSystemType> pDynSys):
    m_simBodies(pDynSys->m_simBodies), m_staticBodies(pDynSys->m_staticBodies),
    m_Collider(&m_collisionSet)
{
}


CollisionSolver::~CollisionSolver() {
    clearCollisionSet();
}



void CollisionSolver::initializeLog( Logging::Log* pSolverLog ) {
    m_pSolverLog = pSolverLog;
    ASSERTMSG(m_pSolverLog != nullptr, "Logging::Log: nullptr!");
}



void CollisionSolver::reset() {
    // Do a Debug check if sizes match!
    ASSERTMSG( m_simBodies.size() != 0, "CollisionSolver:: No Bodies added to the system!");

    removeAllContactDelegates();

    clearCollisionSet();
    m_maxOverlap = 0;

}


void CollisionSolver::clearCollisionSet() {
    for( typename CollisionSetType::iterator it = m_collisionSet.begin(); it != m_collisionSet.end(); ++it) {
        delete (*it);
    }
    m_collisionSet.clear();
}


const typename CollisionSolver::CollisionSetType &
CollisionSolver::getCollisionSetRef()
{
    return m_collisionSet;
}



void CollisionSolver::solveCollision() {


    clearCollisionSet();
    m_maxOverlap = 0;

    LOGSLLEVEL2(m_pSolverLog, "---> solveCollision(): "<<std::endl;)


    // All objects have been updated...

    //// Do simple collision detection (SimBodies to SimBodies)

    if(m_simBodies.size()){
        for(auto bodyIti = m_simBodies.begin(); bodyIti != --m_simBodies.end(); bodyIti++) {
            typename DynamicsSystemType::RigidBodySimContainerType::iterator bodyItj = bodyIti;
            bodyItj++;
            for(; bodyItj != m_simBodies.end(); bodyItj++ ) {

                //check for a collision
                m_Collider.checkCollision((*bodyIti), (*bodyItj));

            }
        }
    }


    // Do simple collision detection (SimBodies to Bodies)
    for(auto bodyIti = m_simBodies.begin(); bodyIti != m_simBodies.end(); bodyIti++) {
        for(auto bodyItk = m_staticBodies.begin(); bodyItk != m_staticBodies.end(); bodyItk ++) {

                //check for a collision and signal
                m_Collider.checkCollision((*bodyIti), (*bodyItk));


        }
    }

    // Signal all found contact
    signalContactAdd();

}


std::string CollisionSolver::getIterationStats() {
    std::stringstream s;
    s << m_maxOverlap;
    return s.str();
}

std::string CollisionSolver::getStatsHeader() {
    std::stringstream s;
    s << "MaxOverlap [m]";
    return s.str();
}

void CollisionSolver::signalContactAdd() {

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
