import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera(cameraDistance=1,
                             cameraYaw=-70,
                             cameraPitch=-14,
                             cameraTargetPosition=[0, .20, -0])
p.setPhysicsEngineParameter(fixedTimeStep=1.0/240)

# planeId = p.loadURDF("plane.urdf", [0,0,0])
#p.loadURDF("toys/concave_box.urdf")


cloth_dir = "napkin.obj"
cloth = p.loadSoftBody(cloth_dir,  basePosition = [0.02,0.02,.2], mass = 0.01, collisionMargin=0.006, useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness = 0.2, springDampingStiffness = 0.01, springBendingStiffness = 0.2, useSelfCollision = 0, frictionCoeff = 2, useFaceContact = 0)

cup_dir = "toys/white_large_cappuccino_mug_cdf.urdf"
cup = p.loadURDF(cup_dir, [0.02,0.02,0],useMaximalCoordinates = True)

p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.005)
#p.setRealTimeSimulation(1)

while p.isConnected():
  p.stepSimulation()
