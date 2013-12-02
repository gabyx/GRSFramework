#include "CollisionSolver.hpp"


CollisionSolver::CollisionSolver(boost::shared_ptr< DynamicsSystemType> pDynSys):
    m_SimBodies(pDynSys->m_SimBodies), m_Bodies(pDynSys->m_Bodies),
    m_Collider(&m_collisionSet)
{
    m_expectedNContacts = 300;
}


CollisionSolver::~CollisionSolver() {
    clearCollisionSet();
}



void CollisionSolver::initializeLog( Logging::Log* pSolverLog ) {
    m_pSolverLog = pSolverLog;
    ASSERTMSG(m_pSolverLog != NULL, "Logging::Log: NULL!");
}



void CollisionSolver::reset() {
    // Do a Debug check if sizes match!
    ASSERTMSG( m_SimBodies.size() != 0, "CollisionSolver:: No Bodies added to the system!");


    clearCollisionSet();

    m_expectedNContacts =  m_SimBodies.size() * 3;


    m_maxOverlap = 0;

}


void CollisionSolver::clearCollisionSet() {
    for( typename CollisionSetType::iterator it = m_collisionSet.begin(); it != m_collisionSet.end(); it++) {
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

#if CoutLevelSolver>1
    LOG(m_pSolverLog, " % -> solveCollision(): "<<std::endl;)
#endif



    // All objects have been updated...

    //// Do simple collision detection (SimBodies to SimBodies)
    typename DynamicsSystemType::RigidBodySimContainerType::iterator bodyIti;
    CollisionData * pColData;
//    for(bodyIti = m_SimBodies.begin(); bodyIti != --m_SimBodies.end(); bodyIti++) {
//        typename DynamicsSystemType::RigidBodySimContainerType::iterator bodyItj = bodyIti;
//        bodyItj++;
//        for(; bodyItj != m_SimBodies.end(); bodyItj++ ) {
//
//            //check for a collision
//            m_Collider.checkCollision((*bodyIti), (*bodyItj));
//
//        }
//    }


    // Do simple collision detection (SimBodies to Bodies)
    typename DynamicsSystemType::RigidBodyNotAniContainer::iterator bodyItk;
    for(bodyIti = m_SimBodies.begin(); bodyIti != m_SimBodies.end(); bodyIti++) {
        for(bodyItk = m_Bodies.begin(); bodyItk != m_Bodies.end(); bodyItk ++) {

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

void CollisionSolver::signalContactAdd() {

    if(m_collisionSet.size()!=0){

        for( auto colDataIt = m_collisionSet.begin(); colDataIt != m_collisionSet.end(); colDataIt++ ){

            ASSERTMSG( std::abs((*colDataIt)->m_cFrame.m_e_x.dot((*colDataIt)->m_cFrame.m_e_y)) < 1e-3 &&
                      std::abs((*colDataIt)->m_cFrame.m_e_y.dot((*colDataIt)->m_cFrame.m_e_z))< 1e-3, "Vectors not orthogonal");

            #if CoutLevelSolverWhenContact>2
                LOG(m_pSolverLog,"Contact Frame: n: " << (*colDataIt)->m_cFrame.m_e_z.transpose() << std::endl;)
            #endif

            //Set contact frame point
            (*colDataIt)->m_cFrame.m_p = (*colDataIt)->m_pBody1->m_r_S + (*colDataIt)->m_r_S1C1;

            // Calculate some Statistics
            m_maxOverlap = std::max(m_maxOverlap,(*colDataIt)->m_overlap);

            m_ContactDelegateList.invokeAll(*colDataIt); // Propagate pointers! they will not be deleted!

        }

    }
}
