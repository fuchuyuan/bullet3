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
#include "DeformableOnRigid.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "../Utils/b3BulletDefaultFileIO.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include <stdio.h>  //printf debugging

#include "../../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../../Importers/ImportURDFDemo/MyMultiBodyCreator.h"

#include "BulletCollision/CollisionShapes/btSdfCollisionShape.h"


#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../Utils/b3ResourcePath.h"

///The DeformableOnRigid shows deformable rigidbody collisions
class DeformableOnRigid : public CommonRigidBodyBase
{
    btAlignedObjectArray<btDeformableLagrangianForce*> m_forces;
public:
    DeformableOnRigid(struct GUIHelperInterface* helper)
    : CommonRigidBodyBase(helper)
    {
    }
    virtual ~DeformableOnRigid()
    {
    }
    void initPhysics();
    
    void exitPhysics();
    
    void resetCamera()
    {
        float dist = 1;
        float pitch = 0;
        float yaw = 30;
        float targetPos[3] = {0, 0, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
    
    void stepSimulation(float deltaTime)
    {
        float internalTimeStep = 1. / 240.f;
        m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
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
        CommonRigidBodyBase::renderScene();
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

void DeformableOnRigid::initPhysics()
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
    //    m_dynamicsWorld->getSolverInfo().m_singleAxisDeformableThreshold = 0.f;//faster but lower quality
    btVector3 gravity = btVector3(0, -10, 0);
    m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
    
    //    getDeformableDynamicsWorld()->before_solver_callbacks.push_back(dynamics);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    
    
    {
    //        load cdf file
            int flags = 0;
            double globalScaling = 1;
            BulletURDFImporter u2b(m_guiHelper, 0, 0, globalScaling, flags);
            char m_fileName[1024];
            b3BulletDefaultFileIO fileio;
            fileio.findResourcePath("toys/white_large_cappuccino_mug_cdf.urdf", m_fileName, 1024);
            bool loadOk = u2b.loadURDF(m_fileName);

            if (loadOk)
            {
                btTransform trans;
                trans.setIdentity();
//                btVector3 orig (0,0.1,0);
//                trans.setOrigin(orig);
                trans.setRotation(btQuaternion(btVector3(1, 0, 0), -SIMD_PI * 0.5));
                u2b.setRootTransformInWorld(trans);
                MyMultiBodyCreator creation(m_guiHelper);
                ConvertURDF2Bullet(u2b, creation, trans, getDeformableDynamicsWorld(), false, u2b.getPathPrefix());
                for (int i = 0; i < u2b.getNumAllocatedCollisionShapes(); i++)
                {
                    m_collisionShapes.push_back(u2b.getAllocatedCollisionShape(i));
                    printf("collision shape size %d\n", m_collisionShapes.size());
                }
            }
        }

    
    // create a piece of cloth
    {
        const btScalar s = .1;
        const btScalar h = 0.2;
        
        btSoftBody* psb = btSoftBodyHelpers::CreatePatch(getDeformableDynamicsWorld()->getWorldInfo(), btVector3(-s, h, -s),
                                                         btVector3(+s, h, -s),
                                                         btVector3(-s, h, +s),
                                                         btVector3(+s, h, +s),
                                                         20,20,
                                                         0, true, 0.01);
        
        
        psb->getCollisionShape()->setMargin(0.006);
        psb->generateBendingConstraints(2);
        psb->setTotalMass(.01);
        psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb->m_cfg.kCHR = 1; // collision hardness with rigid body
        psb->m_cfg.kDF = 10;
//        psb->rotate(btQuaternion(0,SIMD_PI / 2, 0));
        psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        psb->m_cfg.collisions |= btSoftBody::fCollision::VF_DD;
        getDeformableDynamicsWorld()->addSoftBody(psb);
        psb->setSelfCollision(true);
        
        btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(0.2,0.01, false);
        getDeformableDynamicsWorld()->addForce(psb, mass_spring);
        m_forces.push_back(mass_spring);
        
        btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(gravity);
        getDeformableDynamicsWorld()->addForce(psb, gravity_force);
        m_forces.push_back(gravity_force);
    }
    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void DeformableOnRigid::exitPhysics()
{
    //cleanup in the reverse order of creation/initialization
    
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



class CommonExampleInterface* DeformableOnRigidCreateFunc(struct CommonExampleOptions& options)
{
    return new DeformableOnRigid(options.m_guiHelper);
}


