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
    
    btScalar relLinMotion1 = c.m_contactNormal1.dot(bodyA.m_deltaLinMontion) + c.m_relpos1CrossNormal.dot(bodyA.m_deltaAngMotion);
    btScalar relLinMotion2 = c.m_contactNormal2.dot(bodyB.m_deltaLinMontion) + c.m_relpos2CrossNormal.dot(bodyB.m_deltaAngMotion);
    btScalar deltaV = relLinMotion1 + relLinMotion2;
    btScalar bias = (c.m_rhs + deltaV*c.m_jacDiagABInv)*0.025;
//    btScalar bias = 0;
//    printf("\n\nmass A: %f, mass B: %f\n", bodyA.internalGetInvMass()==0?0:1.0/bodyA.internalGetInvMass()[0], bodyB.internalGetInvMass()==0?0:1.0/bodyB.internalGetInvMass()[0]);
//    printf("vel1Dotn: %f, vel2Dotn %f, normalVel %f\n", vel1Dotn, vel2Dotn, normalVel);
    btScalar deltaImpulse = bias - normalVel*c.m_jacDiagABInv;
    const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
    
    if (sum < c.m_lowerLimit)
    {
        deltaImpulse = c.m_lowerLimit - c.m_appliedImpulse;
//         printf("less than lower limit %f\n", deltaImpulse);
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
    bodyA.m_deltaLinearVelocity.setValue(0.f , 0.f, 0.f);
    bodyA.m_deltaAngularVelocity.setValue(0.f, 0.f, 0.f);
    bodyB.m_deltaLinearVelocity.setValue(0.f , 0.f, 0.f);
    bodyB.m_deltaAngularVelocity.setValue(0.f, 0.f, 0.f);
    return deltaImpulse * (1. / c.m_jacDiagABInv);
}

btScalar gResolveSplitPenetrationImpulseSubstep_scalar_reference(
                                                                 btSolverBody& bodyA,
                                                                 btSolverBody& bodyB,
                                                                 const btSolverConstraint& c)
{
    btScalar deltaImpulse = 0.f;
    if (c.m_rhsPenetration)
    {
        deltaImpulse = c.m_rhsPenetration - btScalar(c.m_appliedPushImpulse) * c.m_cfm;
        const btScalar deltaVel1Dotn = c.m_contactNormal1.dot(bodyA.internalGetPushVelocity()) + c.m_relpos1CrossNormal.dot(bodyA.internalGetTurnVelocity());
        const btScalar deltaVel2Dotn = c.m_contactNormal2.dot(bodyB.internalGetPushVelocity()) + c.m_relpos2CrossNormal.dot(bodyB.internalGetTurnVelocity());
        
//        printf("\nmass A: %f, mass B: %f\n", bodyA.internalGetInvMass()==0?0:1.0/bodyA.internalGetInvMass()[0], bodyB.internalGetInvMass()==0?0:1.0/bodyB.internalGetInvMass()[0]);
        deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
        deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;
        const btScalar sum = btScalar(c.m_appliedPushImpulse) + deltaImpulse;
        if (sum < c.m_lowerLimit)
        {
            deltaImpulse = c.m_lowerLimit - c.m_appliedPushImpulse;
            c.m_appliedPushImpulse = c.m_lowerLimit;
        }
        else
        {
            c.m_appliedPushImpulse = sum;
        }
        bodyA.internalApplyPushImpulse(c.m_contactNormal1 * bodyA.internalGetInvMass(), c.m_angularComponentA, deltaImpulse);
        bodyB.internalApplyPushImpulse(c.m_contactNormal2 * bodyB.internalGetInvMass(), c.m_angularComponentB, deltaImpulse);
    }
    return deltaImpulse * (1. / c.m_jacDiagABInv);
}

void updateRHS(btSISolverSingleIterationData& siData, int numPoolConstraints, btScalar invStepDt){
    for (int j = 0; j < numPoolConstraints; j++)
    {
        btSolverConstraint& c = siData.m_tmpSolverContactConstraintPool[siData.m_orderTmpConstraintPool[j]];
        btManifoldPoint* pt = static_cast<btManifoldPoint*>(c.m_originalContactPoint);
        btScalar penetration = pt->getDistance() /*1e-5*/;
        
        c.m_rhs = -penetration*invStepDt*c.m_jacDiagABInv;
        c.m_rhsPenetration = - penetration*invStepDt*c.m_jacDiagABInv;
    }
}


void updateSolverBody(btAlignedObjectArray<btSolverBody>& tmpSolverBodyPool, int iBegin, int iEnd, btScalar dt){
    for (int i = iBegin; i < iEnd; i++)
    {
        btSolverBody& solverBody = tmpSolverBodyPool[i];
        solverBody.m_deltaLinMontion += tmpSolverBodyPool[i].m_linearVelocity;
        solverBody.m_deltaAngMotion += tmpSolverBodyPool[i].m_angularVelocity;
        btTransform newTransform;
        btTransformUtil::integrateTransform(solverBody.m_worldTransform, solverBody.m_linearVelocity, solverBody.m_angularVelocity, dt, newTransform);
        solverBody.m_worldTransform = newTransform;
    }
}

void writeBackToRigidBodies(btAlignedObjectArray<btSolverBody>& tmpSolverBodyPool, int iBegin, int iEnd){
    for (int i = iBegin; i < iEnd; i++)
    {
        btSolverBody& solveBody = tmpSolverBodyPool[i];
        if (solveBody.m_originalBody)
        {
            solveBody.m_originalBody->proceedToTransform(solveBody.m_worldTransform);
            solveBody.m_originalBody->setLinearVelocity(tmpSolverBodyPool[i].m_linearVelocity);
            solveBody.m_originalBody->setAngularVelocity(tmpSolverBodyPool[i].m_angularVelocity);
        }
    }
}

//void writebackSplitImpulseTransform(btAlignedObjectArray<btSolverBody>& tmpSolverBodyPool, int iBegin, int iEnd, btScalar dt, btScalar splitImpulseTurnErp){
//    for (int i = iBegin; i < iEnd; i++)
//    {
//        btSolverBody& solveBody = tmpSolverBodyPool[i];
//        if (solveBody.m_originalBody)
//        {
//            btTransform newTransform;
//            btTransformUtil::integrateTransform(solveBody.m_worldTransform, solveBody.m_pushVelocity, solveBody.m_turnVelocity * splitImpulseTurnErp, dt, newTransform);
////            printf("old trans %f, new trans %f\n", solveBody.m_worldTransform.getOrigin()[2], newTransform.getOrigin()[2]);
//            solveBody.m_worldTransform = newTransform;
////            printf("n\npushed velocity %f %f %f\n", )
//            solveBody.m_originalBody->proceedToTransform(newTransform);
//        }
//    }
//}


void resetData(btSISolverSingleIterationData& siData){
    siData.m_tmpSolverContactConstraintPool.resizeNoInitialize(0);
    siData.m_tmpSolverNonContactConstraintPool.resizeNoInitialize(0);
    siData.m_tmpSolverContactFrictionConstraintPool.resizeNoInitialize(0);
    siData.m_tmpSolverContactRollingFrictionConstraintPool.resizeNoInitialize(0);
    siData.m_tmpSolverBodyPool.resizeNoInitialize(0);
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
                            gResolveSplitPenetrationImpulseSubstep_scalar_reference
//                          btSequentialImpulseConstraintSolver::
//                                  getScalarSplitPenetrationImpulseGeneric()
                                                   ) {}
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
}

void addExternalforceToVelcityAndInitializeDelta(btAlignedObjectArray<btSolverBody>& tmpSolverBodyPool, int iBegin, int iEnd)
{
    for (int i = iBegin; i < iEnd; i++)
    {
        btRigidBody* body = tmpSolverBodyPool[i].m_originalBody;
        if (body)
        {
            // todo: better way to handle gravity?
           tmpSolverBodyPool[i].m_linearVelocity += tmpSolverBodyPool[i].m_externalForceImpulse;
           tmpSolverBodyPool[i].m_deltaLinMontion.setValue(0, 0, 0);
           tmpSolverBodyPool[i].m_deltaAngMotion.setValue(0, 0, 0);
           tmpSolverBodyPool[i].m_deltaLinearVelocity.setValue(0,0,0);
           tmpSolverBodyPool[i].m_deltaAngularVelocity.setValue(0,0,0);
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

        // add external force to solver body velocity
        addExternalforceToVelcityAndInitializeDelta(siData.m_tmpSolverBodyPool, 0, numBodies);
    
        btScalar stepDt = info.m_timeStep/maxIterations;
        btScalar invStepDt = 1/info.m_timeStep*maxIterations;

        btScalar leastSquaresResidual = 0;
        {
                BT_PROFILE("solveGroupCacheFriendlyIterations");
            //hack into updating rhs with penetration
            updateRHS(siData, siData.m_tmpSolverContactConstraintPool.size(), invStepDt);

            btTransform predictedTrans;
//            btSequentialImpulseConstraintSolver::
//            solveGroupCacheFriendlySplitImpulseIterationsInternal(siData, bodies, numBodies,
//                        manifoldPtr, numManifolds, constraints, numConstraints, info, debugDrawer);
//            writebackSplitImpulseTransform(siData.m_tmpSolverBodyPool, 0, siData.m_tmpSolverBodyPool.size(), info.m_timeStep, info.m_splitImpulseTurnErp);

                for (int iteration = 0; iteration < maxIterations; iteration++)
                {
                        leastSquaresResidual =
                                btSequentialImpulseConstraintSolver::solveSingleIterationInternal(
                                        siData, iteration, constraints, numConstraints, info);
                      updateSolverBody(siData.m_tmpSolverBodyPool, 0, siData.m_tmpSolverBodyPool.size(), stepDt);
                }
            
            writeBackToRigidBodies(siData.m_tmpSolverBodyPool, 0, siData.m_tmpSolverBodyPool.size());

            resetData(siData);
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
