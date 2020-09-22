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
#include "FixedJoints.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include <stdio.h>

#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"

///The TGSFixedJoints shows contact between deformable objects and rigid objects.
class TGSFixedJoints : public CommonDeformableBodyBase
{
public:
    int internalSteps = 100;
    int iterations = 10;
    int tgsSteps = 0;
	TGSFixedJoints(struct GUIHelperInterface* helper)
		: CommonDeformableBodyBase(helper)
	{
	}
	virtual ~TGSFixedJoints()
	{
	}
	void initPhysics();

	void exitPhysics();
	void addColliders_testMultiDof(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents);
	btMultiBody* createFeatherstoneMultiBody_testMultiDof(btMultiBodyDynamicsWorld* pWorld, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical, bool floating);

	void resetCamera()
	{
		float dist = 5;
		float pitch = -25;
		float yaw = 30;
		float targetPos[3] = {0, 0.4, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	void stepSimulation(float deltaTime)
	{
		m_dynamicsWorld->stepSimulation(1.0/internalSteps, 0, 1.0/internalSteps);
	}

	void createMultibody(btScalar mass, const btTransform& transform, btCollisionShape* collisionShape, bool floating = false, bool canSleep = false)
	{
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

		btScalar friction = 0;
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
		btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();

		for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
		{
			btSoftBody* psb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
			//if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
			{
				btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
				btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), fDrawFlags::Faces);  // deformableWorld->getDrawFlags());
			}
		}
	}
};

void TGSFixedJoints::initPhysics()
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

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	btVector3 basePos(0, 0, 1);
	btVector3 baseHalfExtents(0.5, 0.5, 0.5);
	btVector3 linkHalfExtents = baseHalfExtents;
	bool floating = false;
	btMultiBody* row = createFeatherstoneMultiBody_testMultiDof(m_dynamicsWorld, 5, basePos, baseHalfExtents, linkHalfExtents, false, floating);

	addColliders_testMultiDof(row, m_dynamicsWorld, baseHalfExtents, linkHalfExtents);

	getDeformableDynamicsWorld()->setImplicit(false);
	getDeformableDynamicsWorld()->setLineSearch(false);

    getDeformableDynamicsWorld()->getSolverInfo().m_timeStep = 1.f /internalSteps;
    getDeformableDynamicsWorld()->getSolverInfo().m_numIterations = iterations;
    getDeformableDynamicsWorld()->getSolverInfo().m_TGS_steps = tgsSteps;

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void TGSFixedJoints::exitPhysics()
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

btMultiBody* TGSFixedJoints::createFeatherstoneMultiBody_testMultiDof(btMultiBodyDynamicsWorld* pWorld, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical, bool floating)
{
	//init the base
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = .1f;

	if (baseMass)
	{
		btCollisionShape* pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}

	bool canSleep = false;

	btMultiBody* pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);

	btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
	pMultiBody->setBasePos(basePosition);
	pMultiBody->setWorldToBaseRot(baseOriQuat);
	pMultiBody->setHasSelfCollision(false);

	//init the links
	float linkMass = 1.f;
	btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

	btCollisionShape* pTempBox = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));
	pTempBox->calculateLocalInertia(linkMass, linkInertiaDiag);
	delete pTempBox;

	//y-axis assumed up
	btVector3 parentComToCurrentCom(linkHalfExtents[1] * 2.f, 0, 0);                       //par body's COM to cur body's COM offset
	btVector3 currentPivotToCurrentCom(linkHalfExtents[1], 0, 0);                          //cur body's COM to cur body's PIV offset
	btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;  //par body's COM to cur body's PIV offset

	//////
	btScalar q0 = 0.f * SIMD_PI / 180.f;
	btQuaternion quat0(btVector3(1, 0, 0).normalized(), q0);
	quat0.normalize();
	/////
	btVector3 hingeJointAxis(0, 0, 1);

	for (int i = 0; i < numLinks; ++i)
	{
//		linkMass *= 2;
		pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, quat0, hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, false);
		btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(pMultiBody, i, 0, 0);
		pWorld->addMultiBodyConstraint(con);
	}

	pMultiBody->finalizeMultiDof();
	pWorld->addMultiBody(pMultiBody);
	///
	return pMultiBody;
}

void TGSFixedJoints::addColliders_testMultiDof(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents)
{
	btAlignedObjectArray<btQuaternion> world_to_local;
	world_to_local.resize(pMultiBody->getNumLinks() + 1);

	btAlignedObjectArray<btVector3> local_origin;
	local_origin.resize(pMultiBody->getNumLinks() + 1);
	world_to_local[0] = pMultiBody->getWorldToBaseRot();
	local_origin[0] = pMultiBody->getBasePos();

	{
		btScalar quat[4] = {-world_to_local[0].x(), -world_to_local[0].y(), -world_to_local[0].z(), world_to_local[0].w()};

		btCollisionShape* box = new btBoxShape(baseHalfExtents);
		box->setMargin(0.01);
		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
		col->setCollisionShape(box);

		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(local_origin[0]);
		tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
		col->setWorldTransform(tr);

		pWorld->addCollisionObject(col, 2, 1 + 2);

		col->setFriction(0);
		pMultiBody->setBaseCollider(col);
	}

	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
	{
		const int parent = pMultiBody->getParent(i);
		world_to_local[i + 1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent + 1];
		local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), pMultiBody->getRVector(i)));
	}

	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
	{
		btVector3 posr = local_origin[i + 1];

		btScalar quat[4] = {-world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w()};

		btCollisionShape* box = new btBoxShape(linkHalfExtents);
		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

		col->setCollisionShape(box);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
		col->setWorldTransform(tr);
		col->setFriction(0);
		pWorld->addCollisionObject(col, 2, 1 + 2);

		pMultiBody->getLink(i).m_collider = col;
	}
}

class CommonExampleInterface* TGSFixedJointsCreateFunc(struct CommonExampleOptions& options)
{
	return new TGSFixedJoints(options.m_guiHelper);
}
