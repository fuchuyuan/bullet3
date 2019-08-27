#include "btSubstepSolver.h"

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "LinearMath/btQuickprof.h"

btScalar gResolveSingleConstraintSubstepRowLowerLimit_scalar_reference(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
{
//    printf("calling new solver");
    btScalar vel1Dotn = c.m_contactNormal1.dot(bodyA.m_linearVelocity) + c.m_relpos1CrossNormal.dot(bodyA.m_angularVelocity);
    btScalar vel2Dotn = c.m_contactNormal2.dot(bodyB.m_linearVelocity) + c.m_relpos2CrossNormal.dot(bodyB.m_angularVelocity);
    btScalar normalVel = vel1Dotn + vel2Dotn;
    
    btScalar relLinMotion1 = c.m_contactNormal1.dot(bodyA.m_deltaLinVelDt) + c.m_relpos1CrossNormal.dot(bodyA.m_deltaAngVelDt);
    btScalar relLinMotion2 = c.m_contactNormal2.dot(bodyB.m_deltaLinVelDt) + c.m_relpos2CrossNormal.dot(bodyB.m_deltaAngVelDt);
    btScalar deltaV = relLinMotion1 + relLinMotion2;
    btScalar bias = -(c.m_rhs + deltaV)*.8;
    
//    if(bias<=0) bias = 0;
    if(bias > .1) bias = .1;
    
    btScalar deltaImpulse =(bias - normalVel)*c.m_jacDiagABInv;
    const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
    
    if (sum < c.m_lowerLimit)
    {
        deltaImpulse = c.m_lowerLimit - c.m_appliedImpulse;
        c.m_appliedImpulse = c.m_lowerLimit;
    }
    else
    {
        c.m_appliedImpulse = sum;
    }
    bodyA.internalApplyImpulse(c.m_contactNormal1 * bodyA.internalGetInvMass(), c.m_angularComponentA, deltaImpulse);
    bodyB.internalApplyImpulse(c.m_contactNormal2 * bodyB.internalGetInvMass(), c.m_angularComponentB, deltaImpulse);
    bodyA.writebackVelocity();
    bodyB.writebackVelocity();
    return deltaImpulse * (1. / c.m_jacDiagABInv);
}

void solveVelocity(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
{
    btScalar vel1Dotn = c.m_contactNormal1.dot(bodyA.m_linearVelocity) + c.m_relpos1CrossNormal.dot(bodyA.m_angularVelocity);
    btScalar vel2Dotn = c.m_contactNormal2.dot(bodyB.m_linearVelocity) + c.m_relpos2CrossNormal.dot(bodyB.m_angularVelocity);
    btScalar normalVel = vel1Dotn + vel2Dotn;
    
    btScalar relLinMotion1 = c.m_contactNormal1.dot(bodyA.m_deltaLinVelDt) + c.m_relpos1CrossNormal.dot(bodyA.m_deltaAngVelDt);
    btScalar relLinMotion2 = c.m_contactNormal2.dot(bodyB.m_deltaLinVelDt) + c.m_relpos2CrossNormal.dot(bodyB.m_deltaAngVelDt);
    btScalar deltaV = relLinMotion1 + relLinMotion2;
    btScalar bias = (c.m_rhs + deltaV)*.8;
    
    if(bias<0) bias = 0;
    
    btScalar deltaImpulse =(bias - normalVel)*c.m_jacDiagABInv;
}

void updateRHS(btSISolverSingleIterationData& siData, int numPoolConstraints, btScalar invStepDt){
    for (int j = 0; j < numPoolConstraints; j++)
    {
        btSolverConstraint& c = siData.m_tmpSolverContactConstraintPool[siData.m_orderTmpConstraintPool[j]];
        btManifoldPoint* pt = static_cast<btManifoldPoint*>(c.m_originalContactPoint);
        btScalar penetration = pt->getDistance() /* + 1e-5*/;
//        if(penetration<0)
//          c.m_rhs = -penetration*invStepDt;
//        else
        //negative if penetration
          c.m_rhs= penetration*invStepDt;
    }
}


void updateDeltaVel(btAlignedObjectArray<btSolverBody>& tmpSolverBodyPool, int iBegin, int iEnd){
    for (int i = iBegin; i < iEnd; i++)
    {
        tmpSolverBodyPool[i].m_deltaLinVelDt += tmpSolverBodyPool[i].m_linearVelocity;
        tmpSolverBodyPool[i].m_deltaAngVelDt += tmpSolverBodyPool[i].m_angularVelocity;
    }
}
void writeBack(btSISolverSingleIterationData& siData, btCollisionObject** bodies, int numBodies, const btContactSolverInfo& infoGlobal){
    btSequentialImpulseConstraintSolver::writeBackJointsInternal(siData.m_tmpSolverNonContactConstraintPool, 0, siData.m_tmpSolverNonContactConstraintPool.size(), infoGlobal);
    btSequentialImpulseConstraintSolver::writeBackBodiesInternal(siData.m_tmpSolverBodyPool, 0, siData.m_tmpSolverBodyPool.size(), infoGlobal);
}

void btSubstepSolver::writeBackBodiesInternal(btAlignedObjectArray<btSolverBody>& tmpSolverBodyPool, int iBegin, int iEnd)
{
    for (int i = iBegin; i < iEnd; i++)
    {
        btRigidBody* body = tmpSolverBodyPool[i].m_originalBody;
        if (body)
        {
//            tmpSolverBodyPool[i].writebackVelocity();
            tmpSolverBodyPool[i].m_originalBody->setLinearVelocity(tmpSolverBodyPool[i].m_linearVelocity);
            tmpSolverBodyPool[i].m_originalBody->setAngularVelocity(tmpSolverBodyPool[i].m_angularVelocity);
        }
    }
}

void resetData(btSISolverSingleIterationData& siData){
    siData.m_tmpSolverContactConstraintPool.resizeNoInitialize(0);
    siData.m_tmpSolverNonContactConstraintPool.resizeNoInitialize(0);
    siData.m_tmpSolverContactFrictionConstraintPool.resizeNoInitialize(0);
    siData.m_tmpSolverContactRollingFrictionConstraintPool.resizeNoInitialize(0);
    
    siData.m_tmpSolverBodyPool.resizeNoInitialize(0);
}

void copyDataToSolverBody(btSolverBody * solverBody, btCollisionObject * collisionObject)
{
    btRigidBody* rb = collisionObject ? btRigidBody::upcast(collisionObject) : 0;
    
    solverBody->internalGetDeltaLinearVelocity().setValue(0.f, 0.f, 0.f);
    solverBody->internalGetDeltaAngularVelocity().setValue(0.f, 0.f, 0.f);
    solverBody->internalGetPushVelocity().setValue(0.f, 0.f, 0.f);
    solverBody->internalGetTurnVelocity().setValue(0.f, 0.f, 0.f);
    
    if (rb)
    {
        solverBody->m_worldTransform = rb->getWorldTransform();
        solverBody->internalSetInvMass(btVector3(rb->getInvMass(), rb->getInvMass(), rb->getInvMass()) * rb->getLinearFactor());
        solverBody->m_originalBody = rb;
        solverBody->m_angularFactor = rb->getAngularFactor();
        solverBody->m_linearFactor = rb->getLinearFactor();
        solverBody->m_linearVelocity = rb->getLinearVelocity();
        solverBody->m_angularVelocity = rb->getAngularVelocity();
    }
    else
    {
        solverBody->m_worldTransform.setIdentity();
        solverBody->internalSetInvMass(btVector3(0, 0, 0));
        solverBody->m_originalBody = 0;
        solverBody->m_angularFactor.setValue(1, 1, 1);
        solverBody->m_linearFactor.setValue(1, 1, 1);
        solverBody->m_linearVelocity.setValue(0, 0, 0);
        solverBody->m_angularVelocity.setValue(0, 0, 0);
    }
}


struct btSubstepSolverInternalData
{
        btAlignedObjectArray<btSolverBody> m_tmpSolverBodyPool;
        btConstraintArray m_tmpSolverContactConstraintPool;
        btConstraintArray m_tmpSolverNonContactConstraintPool;
        btConstraintArray m_tmpSolverContactFrictionConstraintPool;
        btConstraintArray m_tmpSolverContactRollingFrictionConstraintPool;

        btAlignedObjectArray<int> m_orderTmpConstraintPool;
        btAlignedObjectArray<int> m_orderNonContactConstraintPool;
        btAlignedObjectArray<int> m_orderFrictionConstraintPool;
        btAlignedObjectArray<btTypedConstraint::btConstraintInfo1>
                m_tmpConstraintSizesPool;

        unsigned long m_btSeed2;
        int m_fixedBodyId;
        int m_maxOverrideNumSolverIterations;
        btAlignedObjectArray<int>
                m_kinematicBodyUniqueIdToSolverBodyTable;  // only used for multithreading

        btSingleConstraintRowSolver m_resolveSingleConstraintRowGeneric;
        btSingleConstraintRowSolver m_resolveSingleConstraintRowLowerLimit;
        btSingleConstraintRowSolver m_resolveSplitPenetrationImpulse;

        btSubstepSolverInternalData()
                : m_btSeed2(0),
                  m_fixedBodyId(-1),
                  m_maxOverrideNumSolverIterations(0),
                  m_resolveSingleConstraintRowGeneric(
                          btSequentialImpulseConstraintSolver::
                                  getScalarConstraintRowSolverGeneric()),
                  m_resolveSingleConstraintRowLowerLimit(
                          gResolveSingleConstraintSubstepRowLowerLimit_scalar_reference),
//                  m_resolveSingleConstraintRowLowerLimit(
//                        btSequentialImpulseConstraintSolver::
//                                 getScalarConstraintRowSolverLowerLimit()),
                  m_resolveSplitPenetrationImpulse(
                          btSequentialImpulseConstraintSolver::
                                  getScalarSplitPenetrationImpulseGeneric()) {}
};

btSubstepSolver::btSubstepSolver() { m_data = new btSubstepSolverInternalData; }

btSubstepSolver::~btSubstepSolver() { delete m_data; }

void btSubstepSolver::setupHelper(btSISolverSingleIterationData& siData,
                                                                  btCollisionObject** bodies, int numBodies,
                                                                  const btContactSolverInfo& info,
                                                                  btTypedConstraint** constraintStart,
                                                                  int constrainNums,
                                                                  btPersistentManifold** manifoldPtr,
                                                                  int numManifolds)
{
        btSequentialImpulseConstraintSolver::convertBodiesInternal(siData, bodies, numBodies, info);
        btSequentialImpulseConstraintSolver::convertJointsInternal(
                siData, constraintStart, constrainNums, info);

        int i;
        btPersistentManifold* manifold = 0;

        //convert contact
        for (i = 0; i < numManifolds; i++)
        {
                manifold = manifoldPtr[i];
                btSequentialImpulseConstraintSolver::convertContactInternal(siData,                                                                                                                                        manifold, info);

                int numNonContactPool = siData.m_tmpSolverNonContactConstraintPool.size();
                int numConstraintPool = siData.m_tmpSolverContactConstraintPool.size();
                int numFrictionPool =
                        siData.m_tmpSolverContactFrictionConstraintPool.size();

                siData.m_orderNonContactConstraintPool.resizeNoInitialize(
                        numNonContactPool);
                if ((info.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
                        siData.m_orderTmpConstraintPool.resizeNoInitialize(numConstraintPool * 2);
                else
                        siData.m_orderTmpConstraintPool.resizeNoInitialize(numConstraintPool);

                siData.m_orderFrictionConstraintPool.resizeNoInitialize(numFrictionPool);
                {
                        int i;
                        for (i = 0; i < numNonContactPool; i++)
                        {
                                siData.m_orderNonContactConstraintPool[i] = i;
                        }
                        for (i = 0; i < numConstraintPool; i++)
                        {
                                siData.m_orderTmpConstraintPool[i] = i;
                        }
                        for (i = 0; i < numFrictionPool; i++)
                        {
                                siData.m_orderFrictionConstraintPool[i] = i;
                        }
                }
        }
}

btScalar btSubstepSolver::solveGroup(btCollisionObject** bodies, int numBodies,
                                                                         btPersistentManifold** manifoldPtr,
                                                                         int numManifolds,
                                                                         btTypedConstraint** constraints,
                                                                         int numConstraints,
                                                                         const btContactSolverInfo& info,
                                                                         btIDebugDraw* debugDrawer,
                                                                         btDispatcher* dispatcher)
{
        return solveGroupInternal(bodies, numBodies, manifoldPtr, numManifolds,
                                                          constraints, numConstraints, info, debugDrawer,
                                                          dispatcher);
//    return btSequentialImpulseConstraintSolver::solveGroup(bodies, numBodies, manifoldPtr, numManifolds,
//                                                           constraints, numConstraints, info, debugDrawer,
//                                                           dispatcher);
}

void addExternalforceToVelcityAndInitializeDelta(btAlignedObjectArray<btSolverBody>& tmpSolverBodyPool, int iBegin, int iEnd)
{
    for (int i = iBegin; i < iEnd; i++)
    {
        btRigidBody* body = tmpSolverBodyPool[i].m_originalBody;
        if (body)
        {
            tmpSolverBodyPool[i].m_originalBody->setLinearVelocity(
                                                                   tmpSolverBodyPool[i].m_linearVelocity +
                                                                   tmpSolverBodyPool[i].m_externalForceImpulse);
            tmpSolverBodyPool[i].m_originalBody->setAngularVelocity(
                                                                    tmpSolverBodyPool[i].m_angularVelocity +
                                                                    tmpSolverBodyPool[i].m_externalTorqueImpulse);
           tmpSolverBodyPool[i].m_deltaLinVelDt.setValue(0, 0, 0);
           tmpSolverBodyPool[i].m_deltaAngVelDt.setValue(0, 0, 0);
        }
    }
}

btScalar btSubstepSolver::solveGroupInternal(
        btCollisionObject** bodies, int numBodies,
        btPersistentManifold** manifoldPtr, int numManifolds,
        btTypedConstraint** constraints, int numConstraints,
        const btContactSolverInfo& info, btIDebugDraw* debugDrawer,
        btDispatcher* dispatcher)
{
        btSISolverSingleIterationData siData(
                m_data->m_tmpSolverBodyPool, m_data->m_tmpSolverContactConstraintPool,
                m_data->m_tmpSolverNonContactConstraintPool,
                m_data->m_tmpSolverContactFrictionConstraintPool,
                m_data->m_tmpSolverContactRollingFrictionConstraintPool,
                m_data->m_orderTmpConstraintPool, m_data->m_orderNonContactConstraintPool,
                m_data->m_orderFrictionConstraintPool, m_data->m_tmpConstraintSizesPool,
                m_data->m_resolveSingleConstraintRowGeneric,
                m_data->m_resolveSingleConstraintRowLowerLimit,
                m_data->m_resolveSplitPenetrationImpulse,
                m_data->m_kinematicBodyUniqueIdToSolverBodyTable, m_data->m_btSeed2,
                m_data->m_fixedBodyId, m_data->m_maxOverrideNumSolverIterations);

        m_data->m_fixedBodyId = -1;
        int maxIterations =
                siData.m_maxOverrideNumSolverIterations > info.m_numIterations
                        ? siData.m_maxOverrideNumSolverIterations
                        : info.m_numIterations;
        // convert contact to constraints
        setupHelper(siData, bodies, numBodies, info, constraints, numConstraints,
                                manifoldPtr, numManifolds);

        // add external force to rigidbody velocity
        addExternalforceToVelcityAndInitializeDelta(siData.m_tmpSolverBodyPool, 0, numBodies);
//        writeBackBodiesInternal(siData.m_tmpSolverBodyPool, 0, siData.m_tmpSolverBodyPool.size());
    
        btScalar stepDt = info.m_timeStep/maxIterations;
        btScalar invStepDt = 1/info.m_timeStep*maxIterations;
        btScalar leastSquaresResidual = 0;
        {
                BT_PROFILE("solveGroupCacheFriendlyIterations");
                /// this is a special step to resolve penetrations (just for contacts)
                //    btSequentialImpulseConstraintSolver::
                //        solveGroupCacheFriendlySplitImpulseIterationsInternal(
                //            siData, bodies, numBodies, manifoldPtr, numManifolds, constraints,
                //            numConstraints, info, debugDrawer);
            
            //hack into updating rhs with penetration
            updateRHS(siData, siData.m_tmpSolverContactConstraintPool.size(), 1/info.m_timeStep);
            //todo: only update solverbody vel in the iterations
            //      avoid write back to velocity
                btTransform predictedTrans;

                for (int iteration = 0; iteration < maxIterations; iteration++)
                {
                        for (int i = 0; i < siData.m_tmpSolverBodyPool.size(); i++)
                        {
                                btRigidBody* body = siData.m_tmpSolverBodyPool[i].m_originalBody;
                            if (body){
                                //write rigidbody data to solverbody
                                //set vel, angVel
                                copyDataToSolverBody(&siData.m_tmpSolverBodyPool[i], body);
                            }
                        }
                        //solve using updated single row solver
                        leastSquaresResidual =
                                btSequentialImpulseConstraintSolver::solveSingleIterationInternal(
                                        siData, iteration, constraints, numConstraints, info);
                        // printf("residual = %f at iteration #%d\n", m_leastSquaresResidual);

                        // update dv to v
                        // write back velocity from solver body to rigidbody
                      writeBackBodiesInternal(siData.m_tmpSolverBodyPool, 0, siData.m_tmpSolverBodyPool.size());
                      updateDeltaVel(siData.m_tmpSolverBodyPool, 0, siData.m_tmpSolverBodyPool.size());
                        // use solved velocity to integrate
                        for (int i = 0; i < numBodies; i++)
                        {
                                btRigidBody* body = btRigidBody::upcast(bodies[i]);
                                if (body)
                                {
                                        if (body->isActive() && (!body->isStaticObject()))
                                        {
                                                body->predictIntegratedTransform(stepDt, predictedTrans);
                                                body->proceedToTransform(predictedTrans);
                                        }
                                }
                        }
                    
                }
            resetData(siData);
            
//                btSequentialImpulseConstraintSolver::solveGroupCacheFriendlyFinishInternal(siData, bodies, numBodies, info);
        }
        return leastSquaresResidual;
}

void btSubstepSolver::solveMultiBodyGroup(
        btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold,
        int numManifolds, btTypedConstraint** constraints, int numConstraints,
        btMultiBodyConstraint** multiBodyConstraints, int numMultiBodyConstraints,
        const btContactSolverInfo& info, btIDebugDraw* debugDrawer,
        btDispatcher* dispatcher)
{
        btMultiBodyConstraintSolver::solveMultiBodyGroup(
                bodies, numBodies, manifold, numManifolds, constraints, numConstraints,
                multiBodyConstraints, numMultiBodyConstraints, info, debugDrawer,
                dispatcher);
}

void btSubstepSolver::reset()
{
        delete m_data;
        m_data = new btSubstepSolverInternalData;
}
