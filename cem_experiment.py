import pybullet as p
import pybullet_data as pd
import gym
import numpy as np
import math
import time
import random
import socket
import sys
import os
import time
import threading
import keyboard

from scipy.spatial import distance
from gym import spaces 
from ast import literal_eval as make_tuple
from stable_baselines3.sac.policies import MlpPolicy
from stable_baselines3 import SAC
from stable_baselines3.common.env_checker import check_env
import panda_sim

finalForce = np.array([0,0,0])
location = [0,0,0,0,1,0,0,0]
force = np.array([0,0,0])

#Network communication thread function for Touch X Haptics
def netCom():
    s = socket.socket()
     # Define the port on which you want to connect 
    port = 12341                
  
    # connect to the server on local computer 
    s.connect(('127.0.0.1', port))
    time.sleep(3)
    j = 0
    while True:
        j+= 1
        try:
            reading = s.recv(1024).decode()
            #print(reading)
            reading = reading.rstrip('\x00')
            if(reading == ""):
                reading=("(0.0,0.0,2.0,0,1,0,0,0)")
            #Calculate sum of external forces to hand model 
            #TODO Replace for real robot 
            x= -force[0]
            y= -force[1]
            z= force[2]
            forceVec = np.array([x,y,z])

            
            # try:
			#     print(reading)

            #     # env.scene.moveTarget(make_tuple(reading))
            #     # env.scene.moveWithC([0.5+j/1000, -0.2, 1.5])
            #     # contacts1 = env.scene.getContact(1)
            
            #     # for v in contacts1:
            #     #     forceVec = np.add(forceVec,  np.array(list(v[7]))*v[9])
            #     # contacts0 = env.scene.getContact(0)
            #     # for v in contacts0:
            #     #     forceVec = np.add(forceVec,  np.array(list(v[7]))*v[9])

            #     # contacts3 = env.scene.getContact(3)
            #     # for v in contacts3:
            #     #     forceVec = np.add(forceVec,  np.array(list(v[7]))*v[9])
            #     # contacts4 = env.scene.getContact(4)
            #     # for v in contacts4:
            #     #     forceVec = np.add(forceVec,  np.array(list(v[7]))*v[9])
            # except Exception as e:
            #     print(e)
            #     pass
            
            #Scale down magnitude of the force 
            forceVec = forceVec * (1.0/1000.0)
            if (np.linalg.norm(forceVec)>=0.1):
                forceVec = forceVec * (0.1/np.linalg.norm(forceVec))
            
            finalForce = forceVec
            pos = finalForce
            msg = "" + str(pos[0]) + " " + str(pos[1]) + " " + str(pos[2]) + " "
            #print("msg", msg)
            s.sendall(msg.encode())
            global location
            location = make_tuple(reading)
        except Exception as e:
            print(e)  
            s.close()
            break

class CustomEnv(gym.Env):



    def __init__(self):

        #Action space for end effector 3 dimensional
        self.action_space = gym.spaces.box.Box(
            low = np.full(8,-0.06,dtype=np.float32),
            high = np.full(8,0.06,dtype=np.float32)
        )
        #Observation space comprising of target position
        self.observation_space = gym.spaces.box.Box(
            low = np.full(3, -10,dtype=np.float32),
            high = np.full(3,10,dtype=np.float32)
        )
        self.max_ep = 250
        self.ep = 0
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP,1)
        p.setAdditionalSearchPath(pd.getDataPath())

        self.timeStep=1./60.
        p.setTimeStep(self.timeStep)
        p.setGravity(0,-9.8,0)

        processThread = threading.Thread(target=netCom)
        processThread.start()
        self.target_pos = [3,2,1]
        flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        legos=[]
        p.loadURDF("table/table.urdf", [0.25, 0.2, -0.65], [-0.5, -0.5, -0.5, 0.5], flags=flags)
        
        self.panda = panda_sim.PandaSim(p,[0,0.81,0])
        self.panda2 = panda_sim.PandaSim(p,[3,0.81,0])
        self.panda3 = panda_sim.PandaSim(p,[0.4,0.81,-1.5], rotate = True)
        self.panda4 = panda_sim.PandaSim(p,[3.4,0.81,-1.5], rotate = True)
        p.enableJointForceTorqueSensor(self.panda.panda, 7,1)
        self.ep = 0
        self.max_ep = 2500


    def step(self, pos):
        print("Ep :", self.ep)
        stime = time.time()
        if(self.ep <=1):
            link_pos,link_orn,_,_,_,_ = p.getLinkState(self.panda3.panda,11)

            #self.last_pos = np.concatenate((np.array([0,1.41,-0.3,0]),p.getQuaternionFromEuler([math.pi/2.,0.,0.])))
            self.last_pos = np.concatenate((np.array(link_pos),np.array([0]),np.array(link_orn)))
        #Calculate Jacobian
	# qdq_matrix = np.array([np.array(p.getJointState(bodyUniqueId=panda.panda, jointIndex=jointIndex, physicsClientId=0)[:2])for jointIndex in np.arange(1, 8)])
	# q = qdq_matrix[:, 0]
	# dq = qdq_matrix[:, 1]
	# jac_t, jac_r = p.calculateJacobian(panda.panda, 7,[0., 0., 0.0], list(q) + [0.] * 2, [0.] * 9, [0.] * 9,physicsClientId=0)
	# J = np.concatenate((np.array(jac_t)[:, :7], np.array(jac_r)[:, :7]), axis=0)
	# print(J)

        loc = location
        #TODO Proper values for next 3
        self.target_pos = [0.4,1.41,-1.4]
        reward = -1
        done = False
        self.ep +=1 
        if(self.ep >= self.max_ep):
            done = True
            self.ep = 0
        pos = np.array(pos) 

        pos = pos + self.last_pos
        self.last_pos = pos
        pos = tuple(pos)

        first_dist = distance.euclidean(self.target_pos, p.getLinkState(self.panda3.panda,self.panda3.pandaEndEffectorIndex)[0])



        self.panda.current_j_pos =  np.array(p.getJointStates(self.panda.panda,np.arange(0,7)))[:,0]
        global force
        force = np.array(p.getJointState(self.panda.panda,7)[2][:3])
        #print(force)
        #self.panda2.step2()
        
        des= self.panda2.accurateInverseKinematics(tuple(np.array(loc) + [600,0,0,0,0,0,0,0]), 0.1,50,rp=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02] )
        #self.panda.step(position = loc)
        self.panda.step3(des,loc)
        #self.panda2.step(position = tuple(np.array(loc) + [1,0,0,0,0,0,0,0]))

        #autonomous robot control
        des2 = self.panda4.accurateInverseKinematics(tuple(np.array(pos) + [600,0,0,0,0,0,0,0]), 0.1,50,rp=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02] )
        self.panda3.step3(des2)


        p.stepSimulation()
        dist = distance.euclidean(self.target_pos, p.getLinkState(self.panda3.panda,self.panda3.pandaEndEffectorIndex)[0])
        reward = (first_dist - dist)*4000
        if(reward < 0):
            reward *= 5
        print("Distance ", dist)
        print("Reward ", reward)
        t = time.time() - stime
        t_sleep = t- self.timeStep
        if(t_sleep) >0:
            
            time.sleep(self.timeStep)
        return np.array(self.target_pos), reward, done, dict()
    def reset(self):
        p.resetSimulation()
        p.setTimeStep(self.timeStep)
        p.setGravity(0,-9.8,0)
        self.panda = panda_sim.PandaSim(p,[0,0.81,0])
        flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        legos=[]
        p.loadURDF("table/table.urdf", [0, 0.2, -0.65], [-0.5, -0.5, -0.5, 0.5], flags=flags)
        self.panda2 = panda_sim.PandaSim(p,[3,0.81,0])
        self.panda3 = panda_sim.PandaSim(p,[0.2,0.81,-1.3], rotate = True)
        self.panda4 = panda_sim.PandaSim(p,[3.2,0.81,-1.3], rotate = True)
        
        p.enableJointForceTorqueSensor(self.panda.panda, 7,1)
        return np.array(self.target_pos)
        


env = CustomEnv()
model = SAC(MlpPolicy, env, verbose=1)
model.learn(total_timesteps=50000, log_interval=10)
model.save("sac_approach_duck")
#check_env(env,True, True)
i = 0
while (1):
    if keyboard.is_pressed('r'):  
        env.reset()        #Action space for end effector 3 dimensional
    #env.step(location)
    # env.step([0,0+i/1000000,0,0,0,0,0,0])
    # i+=1
    # if(i>30000):
    #     i=0
	#Calculate Jacobian
	# qdq_matrix = np.array([np.array(p.getJointState(bodyUniqueId=panda.panda, jointIndex=jointIndex, physicsClientId=0)[:2])for jointIndex in np.arange(1, 8)])
	# q = qdq_matrix[:, 0]
	# dq = qdq_matrix[:, 1]
	# jac_t, jac_r = p.calculateJacobian(panda.panda, 7,[0., 0., 0.0], list(q) + [0.] * 2, [0.] * 9, [0.] * 9,physicsClientId=0)
	# J = np.concatenate((np.array(jac_t)[:, :7], np.array(jac_r)[:, :7]), axis=0)
	# print(J)
    # panda.current_j_pos =  np.array(p.getJointStates(panda.panda,np.arange(0,7)))[:,0]

    # force= np.array(p.getJointState(panda.panda,7)[2][:3])
	# #print(force)
    # #panda2.step2()
    # panda.step(position = location)
    # panda2.step(position = np.array(location) + [1,0,0,0,0,0,0,0])
    # p.stepSimulation()
    # time.sleep(timeStep)
	
