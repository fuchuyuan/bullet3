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
#include "BagRigid.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"
#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"
#include "../src/LinearMath/btScalar.h"
#include "../Utils/b3BulletDefaultFileIO.h"

///The DeformableRigid shows contact between deformable objects and rigid objects.
class BagRigid : public CommonDeformableBodyBase
{
public:
    BagRigid(struct GUIHelperInterface* helper)
    :CommonDeformableBodyBase(helper)
    {
        m_pickingForceElasticStiffness = 10;
        m_pickingForceDampingStiffness = .1;
    }
    virtual ~BagRigid()
    {
    }
    void initPhysics();
    
    void exitPhysics();
    
    void resetCamera()
    {
        float dist = 1;
        float pitch = -10;
        float yaw = 100;
        float targetPos[3] = {0, 0, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
    
    void stepSimulation(float deltaTime)
    {
        //use a smaller internal timestep, there are stability issues
        float internalTimeStep = 1. / 240.f;
        m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
    }
    
    void Ctor_RbUpStack(int count)
    {
        float mass = .02;
        
        btCompoundShape* cylinderCompound = new btCompoundShape;
        btCollisionShape* cylinderShape = new btCylinderShapeX(btVector3(2, .5, .5));
        btCollisionShape* boxShape = new btBoxShape(btVector3(2, .5, .5));
        btTransform localTransform;
        localTransform.setIdentity();
        cylinderCompound->addChildShape(localTransform, boxShape);
        btQuaternion orn(SIMD_HALF_PI, 0, 0);
        localTransform.setRotation(orn);
        //    localTransform.setOrigin(btVector3(1,1,1));
        cylinderCompound->addChildShape(localTransform, cylinderShape);
        
        btCollisionShape* shape[] = {
            new btBoxShape(btVector3(.05, .05, .05)),
            new btSphereShape(0.75),
            cylinderCompound
        };
        //        static const int nshapes = sizeof(shape) / sizeof(shape[0]);
        //        for (int i = 0; i < count; ++i)
        //        {
        //            btTransform startTransform;
        //            startTransform.setIdentity();
        //            startTransform.setOrigin(btVector3(0, 2+ 2 * i, 0));
        //            startTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
        //            createRigidBody(mass, startTransform, shape[i % nshapes]);
        //        }
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0, .5, 0));
        createRigidBody(mass, startTransform, shape[0]);
//        startTransform.setOrigin(btVector3(1, 1.5, -1));
//        createRigidBody(mass, startTransform, shape[0]);
//        startTransform.setOrigin(btVector3(-1, 1.5, 1));
//        createRigidBody(mass, startTransform, shape[0]);
//        startTransform.setOrigin(btVector3(-1, 1.5, -1));
//        createRigidBody(mass, startTransform, shape[0]);
//        startTransform.setOrigin(btVector3(0, 3.5, 0));
//        createRigidBody(mass, startTransform, shape[0]);
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
            {
                btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), fDrawFlags::Faces);// deformableWorld->getDrawFlags());
            }
        }
    }
};

void BagRigid::initPhysics()
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
    //    deformableBodySolver->setWorld(getDeformableDynamicsWorld());
    //	m_dynamicsWorld->getSolverInfo().m_singleAxisDeformableThreshold = 0.f;//faster but lower quality
    btVector3 gravity = btVector3(0, -10, 0);
    m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
    getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.setDefaultVoxelsz(0.25);
    getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.Reset();
    
    //    getDeformableDynamicsWorld()->before_solver_callbacks.push_back(dynamics);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    
    {
        ///create a ground
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150.), btScalar(25.), btScalar(150.)));
        
        m_collisionShapes.push_back(groundShape);
        
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -25, 0));
        groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
        btScalar mass(0.);
        
        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);
        
        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            groundShape->calculateLocalInertia(mass, localInertia);
        
        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        body->setFriction(1);
        
        //add the ground to the dynamics world
        m_dynamicsWorld->addRigidBody(body);
    }
    
    // create a piece of cloth
    if(1)
    {
        btSoftBody* psb = NULL;
        
        std::vector<tinyobj::shape_t> shapes;
        tinyobj::attrib_t attribute;
        char absolute_path[1024];
        b3BulletDefaultFileIO fileio;
        fileio.findResourcePath("dang_sticky_rice_chips_full_sim.obj", absolute_path, 1024);
        std::string err = tinyobj::LoadObj(attribute, shapes, absolute_path, "", &fileio);
        if (!shapes.empty())
        {
            const tinyobj::shape_t& shape = shapes[0];
            btAlignedObjectArray<btScalar> vertices;
            btAlignedObjectArray<int> indices;
            for (int i = 0; i < attribute.vertices.size(); i++)
            {
                vertices.push_back(attribute.vertices[i]);
            }
            for (int i = 0; i < shape.mesh.indices.size(); i++)
            {
                indices.push_back(shape.mesh.indices[i].vertex_index);
            }
            int numTris = shape.mesh.indices.size() / 3;
            if (numTris > 0)
            {
                psb = btSoftBodyHelpers::CreateFromTriMesh(   getDeformableDynamicsWorld()->getWorldInfo(), &vertices[0], &indices[0], numTris);
                
                btAssert(psb!=nullptr);
            }
            btScalar elastic_stiffness = 0.2;
            btScalar damping_stiffness = 0.01;
            bool not_damp_all_directions = true;
            btScalar bending_stiffness = 0.2;
            btDeformableLagrangianForce* springForce =
            new btDeformableMassSpringForce(elastic_stiffness,
                                            damping_stiffness, not_damp_all_directions,bending_stiffness);
            psb->generateBendingConstraints(3);
            getDeformableDynamicsWorld()->addForce(psb, springForce);
            m_forces.push_back(springForce);
            
            btVector3 gravity =getDeformableDynamicsWorld()->getGravity();
            btDeformableLagrangianForce* gravityForce = new btDeformableGravityForce(gravity *0.2);
            getDeformableDynamicsWorld()->addForce(psb, gravityForce);
            m_forces.push_back(gravityForce);
            
            btScalar friction = 1;
            psb->m_cfg.kDF = friction;
            
            // turn on the collision flag for deformable
            // collision between deformable and rigid
            psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
            // turn on face contact for multibodies
            psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_MDF;
            /// turn on face contact for rigid body
            psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
            // collion between deformable and deformable and self-collision
            psb->m_cfg.collisions |= btSoftBody::fCollision::VF_DD;
            psb->setCollisionFlags(0);
            
            psb->setTotalMass(0.02);
            psb->setSelfCollision(false);
            btScalar repulsionStiffness = 0.2;
            psb->setSpringStiffness(repulsionStiffness);
            psb->initializeFaceTree();
        }
//        psb->scale(btVector3(1, 1, 1));
        psb->rotate(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.5));
        psb->translate(btVector3(0,0.1,0));
        
        psb->getCollisionShape()->setMargin(0.005);
        psb->getCollisionShape()->setUserPointer(psb);
        
        psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb->m_cfg.kCHR = 1; // collision hardness with rigid body
        getDeformableDynamicsWorld()->addSoftBody(psb);        
    }
        Ctor_RbUpStack(10);
    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BagRigid::exitPhysics()
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



class CommonExampleInterface* BagRigidCreateFunc(struct CommonExampleOptions& options)
{
    return new BagRigid(options.m_guiHelper);
}


