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
#include "HV.h"
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
#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"

static int g_constraintSolverType = 1;

class TGSHV : public CommonDeformableBodyBase
{
    int internalSteps;
    int iterations;
    int tgsSteps;
    int num_objects;
    int steps;
    bool use_multibody;
    btMultiBody* objects[10];
    btRigidBody* objects_rigid[10];
public:
	TGSHV(struct GUIHelperInterface* helper)
		: CommonDeformableBodyBase(helper)
	{
        internalSteps = 250;
        iterations = 1;
        tgsSteps = 10;
        num_objects = 2;
        use_multibody = true;
	}
	virtual ~TGSHV()
	{
	}
	void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
		float dist = 1;
		float pitch = -20;
		float yaw = 0;
		float targetPos[3] = {0, 0.8, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	void stepSimulation(float deltaTime)
	{
        m_dynamicsWorld->stepSimulation(1.0/60, internalSteps/60.0 + 1 , 1.0/internalSteps);
//        m_dynamicsWorld->stepSimulation(1.0/internalSteps, 1 , 1.0/internalSteps);
        steps ++;
        
	}

    btMultiBody* createMultibody(btScalar mass, const btTransform& transform, btCollisionShape* collisionShape, bool floating = false, bool canSleep = false){
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
        return pMultiBody;
    }
  
	virtual const btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld() const
	{
		return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
	}

	virtual btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld()
	{
		return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
	}

	virtual void renderScene()
	{
		CommonDeformableBodyBase::renderScene();
	}
};

void TGSHV::initPhysics()
{
    steps = 0;
    if (g_constraintSolverType == 3)
    {
        g_constraintSolverType = 0;
    }
    switch (g_constraintSolverType++)
    {
        case 0:
            iterations = 10;
            tgsSteps = 0;
            internalSteps = 250;
            b3Printf("Constraint Solver: Sequential Impulse");
            break;
        case 1:
            iterations = 1;
            internalSteps = 250;
            tgsSteps = 10;
            b3Printf("Constraint Solver: TGS");
            break;
        default:
            iterations = 1;
            tgsSteps = 0;
            internalSteps = 2500;
            b3Printf("Constraint Solver: Sequential Impulse, substepping");
            break;
    }
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
    getDeformableDynamicsWorld()->getSolverInfo().m_splitImpulse = false;
    getDeformableDynamicsWorld()->getSolverInfo().m_splitImpulsePenetrationThreshold = 0;
    getDeformableDynamicsWorld()->getDispatchInfo().m_deterministicOverlappingPairs = true;
    

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
        
    //load desk from urdf file
    {
        int flags = 0;
        double globalScaling = 1;
        char m_fileName[1024] = "hv/hv_sim.urdf";
        BulletURDFImporter u2b(m_guiHelper, 0, 0, globalScaling, flags);
        bool loadOk = u2b.loadURDF(m_fileName);
        if(loadOk){
            btTransform trans;
            trans.setIdentity();
            trans.setRotation(btQuaternion(btVector3(1,0,0), -SIMD_PI*0.5));
            //            trans.setOrigin(btVector3(0,-1,0));
            MyMultiBodyCreator creation(m_guiHelper);
            
            bool m_useMultiBody = true;
            ConvertURDF2Bullet(u2b, creation, trans, m_dynamicsWorld, m_useMultiBody, u2b.getPathPrefix());
            for (int i = 0; i < u2b.getNumAllocatedCollisionShapes(); i++)
            {
                m_collisionShapes.push_back(u2b.getAllocatedCollisionShape(i));
            }
            for (int i = 0; i < m_dynamicsWorld->getNumMultiBodyConstraints(); i++)
            {
                m_dynamicsWorld->getMultiBodyConstraint(i)->finalizeMultiDof();
            }
        }
    }
    
    //create ground
    {
        btVector3 baseHalfExtents(btScalar(25), btScalar(25), btScalar(25));
        btCollisionShape* groundShape = new btBoxShape(baseHalfExtents);
        m_collisionShapes.push_back(groundShape);
        
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -25, 0));
        groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
        //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
        btScalar mass(0.);
        
        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        btVector3 localInertia(0, 0, 0);
        
        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        body->setFriction(1);
        
        //add the ground to the dynamics world
        m_dynamicsWorld->addRigidBody(body, 1, 1+2);
    }
    
    
	getDeformableDynamicsWorld()->setImplicit(false);
	getDeformableDynamicsWorld()->setLineSearch(false);
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void TGSHV::exitPhysics()
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

class CommonExampleInterface* TGSHVCreateFunc(struct CommonExampleOptions& options)
{
	return new TGSHV(options.m_guiHelper);
}
