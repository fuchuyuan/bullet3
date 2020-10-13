/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */
#include "Stacking.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include <stdio.h>

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"

///The TGSStacking shows contact between deformable objects and rigid objects.
class TGSStacking : public CommonDeformableBodyBase
{
    int internalSteps;
    int iterations;
    int tgsSteps;
    int num_objects;
public:
	TGSStacking(struct GUIHelperInterface* helper)
		: CommonDeformableBodyBase(helper)
	{
        internalSteps = 250;
        iterations = 1;
        tgsSteps = 100;
        num_objects = 2;
	}
	virtual ~TGSStacking()
	{
	}
	void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
		float dist = 1;
		float pitch = 0;
		float yaw = 0;
		float targetPos[3] = {0, 0.2, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	void stepSimulation(float deltaTime)
	{
		//use a smaller internal timestep, there are stability issues
        
//        m_dynamicsWorld->stepSimulation(1.0/60, internalSteps/60.0 + 1 , 1.0/internalSteps);
        m_dynamicsWorld->stepSimulation(1.0/internalSteps, 1 , 1.0/internalSteps);
	}

    void createMultibody(btScalar mass, const btTransform& transform, btCollisionShape* collisionShape, bool floating = false, bool canSleep = false){
        btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
          if (mass)
          {
              collisionShape->calculateLocalInertia(mass, baseInertiaDiag);
          }
        btMultiBody* pMultiBody = new btMultiBody(0, mass, baseInertiaDiag, floating, canSleep);
        pMultiBody->finalizeMultiDof();
        m_dynamicsWorld->addMultiBody(pMultiBody);
        
        pMultiBody->setBasePos(transform.getOrigin());
        pMultiBody->setWorldToBaseRot(transform.getRotation());
        pMultiBody->setCanSleep(canSleep);

        btScalar friction = 0.1;
        {
            btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
            col->setCollisionShape(collisionShape);

            btTransform tr;
            tr.setIdentity();
            tr.setOrigin(transform.getOrigin());
            tr.setRotation(btQuaternion(transform.getRotation()));
            col->setWorldTransform(tr);
            m_dynamicsWorld->addCollisionObject(col, 2, 1 + 2);
            col->setFriction(friction);
            pMultiBody->setBaseCollider(col);
        }
    }
    
	void Ctor_RbUpStack(int count)
	{
		float mass = 1;

		btCompoundShape* cylinderCompound = new btCompoundShape;
		btCollisionShape* cylinderShape = new btCylinderShapeX(btVector3(2, .5, .5));
		btCollisionShape* boxShape = new btBoxShape(btVector3(2, .5, .5));
		btTransform localTransform;
		localTransform.setIdentity();
		cylinderCompound->addChildShape(localTransform, boxShape);
		btQuaternion orn(SIMD_HALF_PI, 0, 0);
		localTransform.setRotation(orn);
		cylinderCompound->addChildShape(localTransform, cylinderShape);

        btScalar radius = .1;
		btCollisionShape* shape[] = {
			new btBoxShape(btVector3(radius, radius, radius)),
			new btSphereShape(radius),
			cylinderCompound};
		btTransform startTransform;
		startTransform.setIdentity();
		for (int i = 0; i < count; i++)
		{
			startTransform.setOrigin(btVector3(0, i*radius*2 /*+ 1 */ + radius, 0));
            mass *=10;
			createMultibody(mass, startTransform, shape[1]);
		}
	}

	virtual const btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld() const
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
	}

	virtual btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld()
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
	}

	virtual void renderScene()
	{
		CommonDeformableBodyBase::renderScene();
	}
};

void TGSStacking::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
	btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
	sol->setDeformableSolver(deformableBodySolver);
	m_solver = sol;

	m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
	btVector3 gravity = btVector3(0, -10, 0);
	m_dynamicsWorld->setGravity(gravity);
	getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
    
    getDeformableDynamicsWorld()->getSolverInfo().m_timeStep = 1.f /internalSteps;
    getDeformableDynamicsWorld()->getSolverInfo().m_numIterations = iterations;
    getDeformableDynamicsWorld()->getSolverInfo().m_TGS_steps = tgsSteps;
    getDeformableDynamicsWorld()->getSolverInfo().m_warmstartingFactor = 0.f;
    getDeformableDynamicsWorld()->getSolverInfo().m_articulatedWarmstartingFactor = 0.f;
    getDeformableDynamicsWorld()->getSolverInfo().m_linearSlop = 0.f;
    getDeformableDynamicsWorld()->getSolverInfo().m_splitImpulse = true;
    getDeformableDynamicsWorld()->getSolverInfo().m_splitImpulsePenetrationThreshold = 0;
    

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	// create a ground
	{
		btVector3 baseHalfExtents(btScalar(25), btScalar(25), btScalar(25));
		float baseMass = .0f;

        btTransform trans;
        trans.setIdentity();
        trans.setOrigin(btVector3(0, -25, 0));
        btCollisionShape* box = new btBoxShape(baseHalfExtents);
        createMultibody(baseMass, trans, box);
    }

	Ctor_RbUpStack(num_objects);
	getDeformableDynamicsWorld()->setImplicit(false);
	getDeformableDynamicsWorld()->setLineSearch(false);
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void TGSStacking::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization
	removePickingConstraint();
	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}
	// delete forces
	for (int j = 0; j < m_forces.size(); j++)
	{
		btDeformableLagrangianForce* force = m_forces[j];
		delete force;
	}
	m_forces.clear();
	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;

	delete m_solver;

	delete m_broadphase;

	delete m_dispatcher;

	delete m_collisionConfiguration;
}

class CommonExampleInterface* TGSStackingCreateFunc(struct CommonExampleOptions& options)
{
	return new TGSStacking(options.m_guiHelper);
}
