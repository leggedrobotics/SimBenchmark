//
// Created by kangd on 10.02.18.
//

#include "World.hpp"

namespace bullet_sim {

World::World(SolverOption solverOption) : solverOption_(solverOption) {

  // broadphase
  broadphase_ = new btDbvtBroadphase();

  // collision configuration and dispatcher
  collisionConfiguration_ = new btDefaultCollisionConfiguration();
  collisionConfiguration_->setConvexConvexMultipointIterations();
  collisionConfiguration_->setPlaneConvexMultipointIterations(5);
  collisionDispatcher_ = new btCollisionDispatcher(collisionConfiguration_);

  // physics solver
  switch(solverOption) {
    case SOLVER_SEQUENTIAL_IMPULSE:
      solver_ = new btSequentialImpulseConstraintSolver;
      break;
    case SOLVER_NNCG:
      solver_ = new btNNCGConstraintSolver;
      break;
    case SOLVER_MLCP_PGS:
      mlcpSolver_ = new btSolveProjectedGaussSeidel;
      solver_ = new btMLCPSolver(mlcpSolver_);
      break;
    case SOLVER_MLCP_DANTZIG:
      solver_ = new btMLCPSolver(new btDantzigSolver);
      break;
    case SOLVER_MLCP_LEMKE:
      mlcpSolver_ = new btLemkeSolver;
      solver_ = new btMLCPSolver(mlcpSolver_);
      break;
    default:
      solver_ = new btSequentialImpulseConstraintSolver;
  }

  // world
  dynamicsWorld_ = new btDiscreteDynamicsWorld(collisionDispatcher_,
                                               broadphase_,
                                               solver_,
                                               collisionConfiguration_);
  dynamicsWorld_->setGravity(gravity_);

  // solver properties
//  dynamicsWorld_->getSolverInfo().m_tau = btScalar(0.6);
//  dynamicsWorld_->getSolverInfo().m_damping = btScalar(1.0);
//  dynamicsWorld_->getSolverInfo().m_friction = btScalar(0.8);
//  dynamicsWorld_->getSolverInfo().m_restitution = 0.0;
//  dynamicsWorld_->getSolverInfo().m_maxErrorReduction = btScalar(20.);
//  dynamicsWorld_->getSolverInfo().m_numIterations = 10;                     // TODO
  dynamicsWorld_->getSolverInfo().m_erp = 0;
  dynamicsWorld_->getSolverInfo().m_erp2 = 0;
  dynamicsWorld_->getSolverInfo().m_frictionERP = 0;
//  dynamicsWorld_->getSolverInfo().m_globalCfm = btScalar(0.0);
//  dynamicsWorld_->getSolverInfo().m_frictionCFM = 0;
//  dynamicsWorld_->getSolverInfo().m_sor = btScalar(1.);
//  dynamicsWorld_->getSolverInfo().m_splitImpulse = false;
//  dynamicsWorld_->getSolverInfo().m_splitImpulsePenetrationThreshold = 0;
//  dynamicsWorld_->getSolverInfo().m_splitImpulseTurnErp = 0;
//  dynamicsWorld_->getSolverInfo().m_linearSlop = 0;
  dynamicsWorld_->getSolverInfo().m_warmstartingFactor= 0;
//  dynamicsWorld_->getSolverInfo().m_solverMode = SOLVER_SIMD;// | SOLVER_RANDMIZE_ORDER;
//  dynamicsWorld_->getSolverInfo().m_restingContactRestitutionThreshold = 2;//unused as of 2.81
//  dynamicsWorld_->getSolverInfo().m_minimumSolverBatchSize = 128; //try to combine islands until the amount of constraints reaches this limit
//  dynamicsWorld_->getSolverInfo().m_singleAxisRollingFrictionThreshold = 1e30f;///if the velocity is above this threshold, it will use a single constraint row (axis), otherwise 3 rows.
//  dynamicsWorld_->getSolverInfo().m_leastSquaresResidualThreshold = 0.f;
//  dynamicsWorld_->getSolverInfo().m_restitutionVelocityThreshold = 0.2f;//if the relative velocity is below this threshold, there is zero restitution

}

World::~World() {

  // remove world
  delete dynamicsWorld_;
  
  // remove solver
  if(mlcpSolver_)
    delete mlcpSolver_;
  delete solver_;
  
  delete collisionDispatcher_;
  delete collisionConfiguration_;
  delete broadphase_;

  // remove objects
  for (auto *ob: objectList_)
    delete ob;
}

bullet_sim::object::Sphere *bullet_sim::World::addSphere(double radius,
                                                         double mass,
                                                         CollisionGroupType collisionGroup,
                                                         CollisionGroupType collisionMask) {
  bullet_sim::object::Sphere *sphere = new bullet_sim::object::Sphere(radius, mass);
  dynamicsWorld_->addRigidBody(sphere->getRigidBody(), collisionGroup, collisionMask);
  objectList_.push_back(sphere);
  return sphere;
}

bullet_sim::object::Box *bullet_sim::World::addBox(double xLength,
                                                   double yLength,
                                                   double zLength,
                                                   double mass,
                                                   CollisionGroupType collisionGroup,
                                                   CollisionGroupType collisionMask) {

  bullet_sim::object::Box *box = new bullet_sim::object::Box(xLength, yLength, zLength, mass);
  dynamicsWorld_->addRigidBody(box->getRigidBody(), collisionGroup, collisionMask);
  objectList_.push_back(box);
  return box;
}

object::Capsule *World::addCapsule(double radius,
                                   double height,
                                   double mass,
                                   CollisionGroupType collisionGroup,
                                   CollisionGroupType collisionMask) {
  bullet_sim::object::Capsule *capsule = new bullet_sim::object::Capsule(radius, height, mass);
  dynamicsWorld_->addRigidBody(capsule->getRigidBody(), collisionGroup, collisionMask);
  objectList_.push_back(capsule);
  return capsule;
}

bullet_sim::object::CheckerBoard *bullet_sim::World::addCheckerboard(double gridSize,
                                                                     double xLength,
                                                                     double yLength,
                                                                     double reflectanceI,
                                                                     CollisionGroupType collisionGroup,
                                                                     CollisionGroupType collisionMask) {

  object::CheckerBoard *checkerBoard = new bullet_sim::object::CheckerBoard(xLength, yLength);
  dynamicsWorld_->addRigidBody(checkerBoard->getRigidBody(), collisionGroup, collisionMask);
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}

void bullet_sim::World::integrate(double dt) {
  // TODO substep
  // simulation step
  dynamicsWorld_->stepSimulation(dt, 0);

  // clear collision
  contactProblemList_.clear();
  int contactProblemSize = 0;
  for (int i = 0; i < collisionDispatcher_->getNumManifolds(); i++)
    contactProblemSize += collisionDispatcher_->getManifoldByIndexInternal(i)->getNumContacts();
  contactProblemList_.reserve(contactProblemSize);

  for (int i = 0; i < collisionDispatcher_->getNumManifolds(); i++)
  {
    btPersistentManifold* contactManifold = collisionDispatcher_->getManifoldByIndexInternal(i);
//    const btCollisionObject* obA = contactManifold->getBody0();
//    const btCollisionObject* obB = contactManifold->getBody1();

    for (int j = 0; j < contactManifold->getNumContacts(); j++)
    {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      if (pt.getDistance() <= 0.f)
      {
        const btVector3& ptA = pt.getPositionWorldOnA();
//        const btVector3& ptB = pt.getPositionWorldOnB();
        const btVector3& normalOnB = pt.m_normalWorldOnB;
        contactProblemList_.emplace_back(ptA, normalOnB);
      }
    }
  }
}

const std::vector<Single3DContactProblem> *World::getCollisionProblem() const {
  return &contactProblemList_;
}

void bullet_sim::World::setGravity(const btVector3 &gravity_) {
  World::gravity_ = gravity_;
  dynamicsWorld_->setGravity(gravity_);
}

void World::setERP(double erp, double erp2, double frictionErp) {
  dynamicsWorld_->getSolverInfo().m_erp = erp;
  dynamicsWorld_->getSolverInfo().m_erp2 = erp2;
  dynamicsWorld_->getSolverInfo().m_frictionERP = frictionErp;
}

} // bullet_sim
