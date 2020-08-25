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

#include "VolumetricDeformable.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"

#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"

///The VolumetricDeformable shows the contact between volumetric deformable objects and rigid objects.
static btScalar E = 25;
static btScalar nu = 0.3;
static btScalar damping = 0.01;

struct TetraCube
{
#include "../SoftDemo/cube.inl"
};

class VolumetricDeformable : public CommonDeformableBodyBase
{
	btDeformableNeoHookeanForce* m_neohookean;

public:
	VolumetricDeformable(struct GUIHelperInterface* helper)
		: CommonDeformableBodyBase(helper)
	{
        m_neohookean = 0;
	}
	virtual ~VolumetricDeformable()
	{
	}
	void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
        float dist = 3;
        float pitch = -15;
        float yaw = 10;
        float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
    
    void stepSimulation(float deltaTime)
    {
//		m_neohookean->setPoissonRatio(nu);
//		m_neohookean->setYoungsModulus(E);
//		m_neohookean->setDamping(damping);
        //use a smaller internal timestep, there are stability issues
        deltaTime = 1. / 60.f;
        float stepTime = 1. / (250*50);
        
        m_dynamicsWorld->stepSimulation(deltaTime, deltaTime/stepTime, stepTime);
    
    }
    
    void createStaticBox(const btVector3& halfEdge, const btVector3& translation)
    {
        btCollisionShape* box = new btBoxShape(halfEdge);
        m_collisionShapes.push_back(box);
        
        btTransform Transform;
        Transform.setIdentity();
        Transform.setOrigin(translation);
        Transform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.0));
        //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
        btScalar mass(0.);
        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);
        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            box->calculateLocalInertia(mass, localInertia);
        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(Transform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, box, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        body->setFriction(0.5);
        
        //add the ground to the dynamics world
        m_dynamicsWorld->addRigidBody(body);
    }
    
    void Ctor_RbUpStack(int count)
    {
        float mass = 0.002;
//
//        btCompoundShape* cylinderCompound = new btCompoundShape;
//        btCollisionShape* cylinderShape = new btCylinderShapeX(btVector3(2, .5, .5));
//        btCollisionShape* boxShape = new btBoxShape(btVector3(2, .5, .5));
//        btTransform localTransform;
//        localTransform.setIdentity();
//        cylinderCompound->addChildShape(localTransform, boxShape);
//        btQuaternion orn(SIMD_HALF_PI, 0, 0);
//        localTransform.setRotation(orn);
//        //    localTransform.setOrigin(btVector3(1,1,1));
//        cylinderCompound->addChildShape(localTransform, cylinderShape);
        
        btCollisionShape* shape[] = {
            new btSphereShape(.1),
//            new btBoxShape(btVector3(1,1,1)),
        };
        static const int nshapes = sizeof(shape) / sizeof(shape[0]);
        for (int i = 0; i < count; ++i)
        {
            btTransform startTransform;
            startTransform.setIdentity();
            startTransform.setOrigin(btVector3(0, 5 * i + .4, 0));
//            shape[0]->setMargin(1);
            createRigidBody(mass, startTransform, shape[i % nshapes]);
        }
    }
    
    virtual void renderScene()
    {
        CommonDeformableBodyBase::renderScene();
        btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btSoftBody* psb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
            {
                btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
            }
        }
    }
};

void VolumetricDeformable::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
    btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();

	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(deformableBodySolver);
	m_solver = sol;

	m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
    btVector3 gravity = btVector3(0, -10, 0);
	m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
	getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.setDefaultVoxelsz(0.25);
	getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.Reset();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);


    getDeformableDynamicsWorld()->getSolverInfo().m_TGS_steps = 0;
    getDeformableDynamicsWorld()->getSolverInfo().m_numIterations = 1;
    getDeformableDynamicsWorld()->getSolverInfo().m_erp2 = 0.08f;
    // add a few rigid bodies
    Ctor_RbUpStack(0);
    
    //load desk from urdf file
    {
        int flags = 0;
        double globalScaling = 1;
        char m_fileName[1024] = "planeMesh.urdf";
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
    // create a rigidbody from urdf file
    if(1)
     {
         int flags = 0;
         double globalScaling = 1;
         char m_fileName[1024] = "cup/coffee_cup.urdf";
         BulletURDFImporter u2b(m_guiHelper, 0, 0, globalScaling, flags);
         bool loadOk = u2b.loadURDF(m_fileName);
         if(loadOk){
             btTransform identityTrans;
             identityTrans.setIdentity();
             identityTrans.setRotation(btQuaternion(btVector3(1,0,0), -0.5*SIMD_PI));
//             identityTrans.setRotation(btQuaternion(btVector3(0,1,0), 1*SIMD_PI));
             identityTrans.setOrigin(btVector3(0,0.2,0));
             MyMultiBodyCreator creation(m_guiHelper);

             bool m_useMultiBody = true;
             ConvertURDF2Bullet(u2b, creation, identityTrans, m_dynamicsWorld, m_useMultiBody, u2b.getPathPrefix());
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
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	
	{
		SliderParams slider("Young's Modulus", &E);
		slider.m_minVal = 0;
		slider.m_maxVal = 50;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		SliderParams slider("Poisson Ratio", &nu);
		slider.m_minVal = 0.05;
		slider.m_maxVal = 0.40;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		SliderParams slider("Damping", &damping);
		slider.m_minVal = 0.01;
		slider.m_maxVal = 0.02;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
}

void VolumetricDeformable::exitPhysics()
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



class CommonExampleInterface* VolumetricDeformableCreateFunc(struct CommonExampleOptions& options)
{
	return new VolumetricDeformable(options.m_guiHelper);
}


