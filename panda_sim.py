import time
import numpy as np
import math
from pyquaternion import Quaternion 

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
jointPositions=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
rp = jointPositions


class PandaSim(object):
  def __init__(self, bullet_client, offset, rotate = False):
    self.bullet_client = bullet_client
    self.offset = np.array(offset)
    self.current_j_pos = jointPositions
    self.pandaEndEffectorIndex = pandaEndEffectorIndex
    #print("offset=",offset)
    flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    legos=[]
    # self.bullet_client.loadURDF("table/table.urdf", [0+offset[0], -0.61+offset[1], -0.6+offset[2]], [-0.5, -0.5, -0.5, 0.5], flags=flags)
    # sphereId = self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0, 0.3, -0.6])+self.offset, flags=flags)
    # self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0, 0.3, -0.5])+self.offset, flags=flags)
    # self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0, 0.3, -0.7])+self.offset, flags=flags)
    
    orn=[-0.707107, 0.0, 0.0, 0.707107]#p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
    if(rotate):
      q1 = Quaternion(0.707107, -0.707107, 0.0, 0.0 ) # Rotate 180 about X
      q2 = Quaternion(axis=[0,0, 1], angle=3.14159265 ) 
      q3 = q1 * q2 # Composite rotation of q1 then q2 expressed as standard multiplication
      orn = (q3.x, q3.y, q3.z, q3.w)
    eul = self.bullet_client.getEulerFromQuaternion([-0.5, -0.5, -0.5, 0.5])
    self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([0,0,0])+self.offset, orn, useFixedBase=True, flags=flags)
    index = 0
    for j in range(self.bullet_client.getNumJoints(self.panda)):
      self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
      info = self.bullet_client.getJointInfo(self.panda, j)
  
      jointName = info[1]
      jointType = info[2]
      if (jointType == self.bullet_client.JOINT_PRISMATIC):
        
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
      if (jointType == self.bullet_client.JOINT_REVOLUTE):
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
    self.t = 0.
    self.finger_target = 0
  def reset(self):
    pass

  def accurateInverseKinematics(self, position, treshold, maxIter,  rp=None):
    x,y,z,b, q,w,e,r = position
    target_pos = [x/200,1.11+y/200,-0.6+z/200]

    
   
    #orn = self.bullet_client.getQuaternionFromEuler([-g3,-g2,-g1])
    q1 = Quaternion(r,q,w,e) # Rotate 180 about X
    q2 = Quaternion(axis=[0, 1, 0], angle=3.14159265 ) # Rotate 90 about Y
    q3 = q1 * q2 # Composite rotation of q1 then q2 expressed as standard multiplication

    targetorientation = (q3.x, q3.y, q3.z, q3.w)    
    closeEnough = False
    des_joints = None
    num_calls = 0
    self.current_j_pos = self.bullet_client.getJointStates(self.panda,np.arange(0,7))[:][:1]
    
    c_joints = self.current_j_pos if rp is None else rp
    for i in range(pandaNumDofs):
        self.bullet_client.resetJointState(self.panda, i, c_joints[i])
    for i in [9,10]:
      self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target ,force= 10)
    while not closeEnough and num_calls < maxIter:
      des_joints =  self.bullet_client.calculateInverseKinematics(self.panda,pandaNumDofs, target_pos, targetorientation, ll, ul, jr, rp, maxNumIterations=5)
      for i in range(pandaNumDofs):
        self.bullet_client.resetJointState(self.panda, i, des_joints[i])
      for i in [9,10]:
        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target ,force= 10)
      ls = self.bullet_client.getLinkState(bodyUniqueId=self.panda,linkIndex=pandaEndEffectorIndex,computeForwardKinematics = 1)
      new_c_pos = ls[4]
      diff = [target_pos[0] - new_c_pos[0],target_pos[1] - new_c_pos[1],target_pos[2] - new_c_pos[2]]
      dist2 = (diff[0]* diff[0]+diff[1]* diff[1]+diff[2]* diff[2])
      closeEnough = (dist2< treshold)
      num_calls +=1
    return np.array(des_joints)[:7]




  def step(self, position = (0,0,0,0,0,0,0,0)):
    #print(" Parameters", ll, ul, jr)
    #t = self.t
    #self.t += 1./60.
    #pos = [self.offset[0]+0.2 * math.sin(1.5 * t), self.offset[1]+0.044, self.offset[2]+-0.6 + 0.1 * math.cos(1.5 * t)]
    x,y,z,b, q,w,e,r = position
    if(b):
      self.finger_target = 0.01
    else:
      self.finger_target = 0.04
    pos = [x/200,1.11+y/200,-0.6+z/200]

    
   
    #orn = self.bullet_client.getQuaternionFromEuler([-g3,-g2,-g1])
    q1 = Quaternion(r,q,w,e) # Rotate 180 about X
    q2 = Quaternion(axis=[0, 1, 0], angle=3.14159265 ) # Rotate 90 about Y
    q3 = q1 * q2 # Composite rotation of q1 then q2 expressed as standard multiplication

    orn = (q3.x, q3.y, q3.z, q3.w)
    #orn = self.bullet_client
    #print("orn" , orn)

    #jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, pos, orn, ll, ul,
    jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaNumDofs, pos, orn, ll, ul,
      jr, rp, maxNumIterations=5)
      
    for i in range(pandaNumDofs):
        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240.)
    for i in [9,10]:
      self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target ,force= 10)
    #self.bullet_client.setJointMotorControl2(self.panda, 5, self.bullet_client.POSITION_CONTROL,(g2)*360/400000 ,force= 10)
    #self.bullet_client.setJointMotorControl2(self.panda, 6, self.bullet_client.POSITION_CONTROL,(g1-1800)*360/400000 ,force= 10)   
    pass
  def step2(self ):
    t = self.t
    self.t += 1./60.
    pos = [self.offset[0]+0.2 * math.sin(1.5 * t), self.offset[1]+0.044, self.offset[2]+-0.6 + 0.1 * math.cos(1.5 * t)]
    orn = self.bullet_client.getQuaternionFromEuler([math.pi/2.,0.,0.])
    jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=5)
    #jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, pos, orn, rp, rp, rp, rp, maxNumIterations=5)

    for i in range(pandaNumDofs):
        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240.)
        
    pass
  def step3(self, jointPoses, pos = (0,0,0,0,0,0,0,1)):
      x,y,z,b, q,w,e,r = pos
      if(b == 1):
        self.finger_target = 0.01
      else:
        self.finger_target = 0.04
        
      for i in range(pandaNumDofs):
          self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240.)
      for i in [9,10]:
          self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target ,force= 10)
      #for i in [9,10]:
      #  self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target ,force= 10)
      #self.bullet_client.setJointMotorControl2(self.panda, 5, self.bullet_client.POSITION_CONTROL,(g2)*360/400000 ,force= 10)
      #self.bullet_client.setJointMotorControl2(self.panda, 6, self.bullet_client.POSITION_CONTROL,(g1-1800)*360/400000 ,force= 10)   
      pass
  def moveToCartAndQuat(self, cart, quat):
      pass