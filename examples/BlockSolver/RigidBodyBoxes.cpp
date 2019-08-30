#include "RigidBodyBoxes.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BlockSolverExample.h"
#include "btBlockSolver.h"
#include "btSubstepSolver.h"

class RigidBodyBoxes : public CommonRigidBodyBase
{
	int m_option;
	int m_numIterations;
	int m_numBoxes;
	btAlignedObjectArray<btRigidBody*> boxes;
	static btScalar numSolverIterations;
    btScalar m_timeElapsed;

public:
	RigidBodyBoxes(GUIHelperInterface* helper, int option);
	virtual ~RigidBodyBoxes();

	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);
	void resetCubePosition();
	virtual void resetCamera()
	{
		float dist = 3;
		float pitch = -35;
		float yaw = 50;
		float targetPos[3] = {0, 0, .1};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1],
								 targetPos[2]);
	}

	void createRigidBodyStack();
};

btScalar RigidBodyBoxes::numSolverIterations = 4;

RigidBodyBoxes::RigidBodyBoxes(GUIHelperInterface* helper, int option)
	: CommonRigidBodyBase(helper),
	  m_option(option),
	  m_numBoxes(3),
      m_numIterations(numSolverIterations),
      m_timeElapsed(0)
{
	m_guiHelper->setUpAxis(2);
}

RigidBodyBoxes::~RigidBodyBoxes()
{
	// Do nothing
}

void RigidBodyBoxes::createRigidBodyStack()
{
	// create ground
	btBoxShape* groundShape =
		createBoxShape(btVector3(btScalar(5.), btScalar(5.), btScalar(5.)));
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, -5));
	btScalar mass(0.);
	btRigidBody* body = createRigidBody(mass, groundTransform, groundShape,
										btVector4(0, 0, 1, 1));

	// create a few boxes
	mass = 1;
	for (int i = 0; i < m_numBoxes; i++)
	{
//        btBoxShape* boxShape = createBoxShape(btVector3(btScalar(.1), btScalar(.1), btScalar(.1)));
        btSphereShape* boxShape = new btSphereShape(btScalar(.1));
		m_collisionShapes.push_back(boxShape);
        if(i==m_numBoxes-1) mass = 20;
		btTransform tr;
		tr.setIdentity();
        btRigidBody* box = createRigidBody(mass, tr, boxShape);
        box->setSleepingThresholds(0, 0);
        box->setFriction(0);
		boxes.push_back(box);
	}
    resetCubePosition();
}

void RigidBodyBoxes::initPhysics()
{
	/// collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	/// use the default collision dispatcher. For parallel processing you can use
	/// a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();

	{
		SliderParams slider("numSolverIterations", &numSolverIterations);
		slider.m_minVal = 1;
		slider.m_maxVal = 30;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	if (m_option & BLOCK_SOLVER_SI)
	{
		m_solver = new btSequentialImpulseConstraintSolver;
		b3Printf("Constraint Solver: Sequential Impulse");
	}
	if (m_option & BLOCK_SOLVER_BLOCK)
	{
		m_solver = new btBlockSolver();
		b3Printf("Constraint Solver: Block solver");
	}
	if (m_option & BLOCK_SUBSTEP_SOLVER)
	{
        m_solver = new btSubstepSolver();
//        m_solver = new btSequentialImpulseConstraintSolver;
		b3Printf("Constraint Solver: small timestep");
	}
	btAssert(m_solver);

	m_dynamicsWorld = new btDiscreteDynamicsWorld(
		m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld->setGravity(btVector3(0, 0, -1));

	createRigidBodyStack();

	m_dynamicsWorld->getSolverInfo().m_numIterations = numSolverIterations;
	m_dynamicsWorld->getSolverInfo().m_globalCfm = btScalar(1e-6);

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void RigidBodyBoxes::resetCubePosition()
{
	for (int i = 0; i < m_numBoxes; i++)
	{
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, .3 + i * 0.2));
		boxes[i]->setWorldTransform(tr);
        btVector3 baseLinVel(0, 0, 0);
        boxes[i]->setLinearVelocity(baseLinVel);
	}
}

void RigidBodyBoxes::stepSimulation(float deltaTime)
{
	if ((int)numSolverIterations != m_numIterations)
	{
		resetCubePosition();
		m_numIterations = (int)numSolverIterations;
		m_dynamicsWorld->getSolverInfo().m_numIterations = m_numIterations;
		b3Printf("New num iterations; %d", m_numIterations);
        m_timeElapsed=0;
	}
    double dt = .1;
    int maxsubsteps = 0;
    double fixed_dt = dt;
//    m_dynamicsWorld->stepSimulation(dt);
    btVector3 pos = boxes[0]->getCenterOfMassPosition();
    if(m_timeElapsed<1.2)
    printf("time: %f, pos: %f %f %f \n", m_timeElapsed, pos[0], pos[1], pos[2]);
    m_dynamicsWorld->stepSimulation(dt, maxsubsteps, fixed_dt);
    m_timeElapsed +=dt;
}

CommonExampleInterface* RigidBodyBoxesCreateFunc(
	CommonExampleOptions& options)
{
	return new RigidBodyBoxes(options.m_guiHelper, options.m_option);
}
