#ifndef BT_SUBSTEP_SOLVER_H
#define BT_SUBSTEP_SOLVER_H

#include "Bullet3Common/b3Logging.h"
#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "btBlockSolver.h"

class btSubstepSolver : public btMultiBodyConstraintSolver {
  struct btSubstepSolverInternalData* m_data;

 public:
  btSubstepSolver();

  virtual ~btSubstepSolver();

  // btRigidBody
  virtual btScalar solveGroup(btCollisionObject** bodies, int numBodies,
                              btPersistentManifold** manifoldPtr,
                              int numManifolds, btTypedConstraint** constraints,
                              int numConstraints,
                              const btContactSolverInfo& info,
                              class btIDebugDraw* debugDrawer,
                              btDispatcher* dispatcher);
    
  btScalar solveGroupInternal(btCollisionObject** bodies, int numBodies,
                              btPersistentManifold** manifoldPtr,
                              int numManifolds, btTypedConstraint** constraints,
                              int numConstraints,
                              const btContactSolverInfo& info,
                              btIDebugDraw* debugDrawer,
                              btDispatcher* dispatcher);
 
  // btMultibody
  virtual void solveMultiBodyGroup(
      btCollisionObject** bodies, int numBodies,
      btPersistentManifold** manifold, int numManifolds,
      btTypedConstraint** constraints, int numConstraints,
      btMultiBodyConstraint** multiBodyConstraints, int numMultiBodyConstraints,
      const btContactSolverInfo& info, btIDebugDraw* debugDrawer,
      btDispatcher* dispatcher);

  /// clear internal cached data and reset random seed
  virtual void reset();

  virtual btConstraintSolverType getSolverType() const {
    return BT_SUBSTEP_SOLVER;
  }
  
 void writeBackBodiesInternal(btAlignedObjectArray<btSolverBody>& tmpSolverBodyPool, int iBegin, int iEnd);
  void setupHelper(btSISolverSingleIterationData& siData,
                 btCollisionObject** bodies, int numBodies,
                 const btContactSolverInfo& info,
                 btTypedConstraint** constraintStart, int constrainNums,
                 btPersistentManifold** manifoldPtr, int numManifolds);
};

#endif  // BT_SUBSTEP_SOLVER_H
