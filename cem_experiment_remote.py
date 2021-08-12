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
import panda_sim_grasp as panda_sim_grasp
import panda_sim
from pyquaternion import Quaternion

finalForce = np.array([0,0,0])
location = [0,0,0,0,-0.707107, 0.0, 0.0, 0.707107]
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
            if (np.linalg.norm(forceVec)>=0.5):
                forceVec = forceVec * (0.5/np.linalg.norm(forceVec))
            
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
            exit()

class CustomEnv(gym.Env):



    def __init__(self):

        #Action space for end effector 3 dimensional
        self.action_space = gym.spaces.box.Box(
            low = np.full(8,-0.008,dtype=np.float32),
            high = np.full(8,0.008,dtype=np.float32)
        )
        #Observation space comprising of target position
        self.observation_space = gym.spaces.box.Box(
            low = np.full(9, -1000,dtype=np.float32),
            high = np.full(9,1000,dtype=np.float32)
        )
        self.max_ep = 250
        self.ep = 0
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP,1)
        p.setAdditionalSearchPath(pd.getDataPath())

        self.timeStep=1./120.
        p.setTimeStep(self.timeStep)
        p.setGravity(0,-9.8,0)

        processThread = threading.Thread(target=netCom)
        processThread.start()
        self.target_pos = [0,2,-0.7]
        self.setupscene()
        self.ep = 0
        self.max_ep = 2500
        self.f = open("userinput2.txt", "a")
        self.f.write("Data Collection\n")
        


    def step(self, pos):
        print("Ep :", self.ep)
        stime = time.time()
        loc = location
        posi, o = p.getBasePositionAndOrientation(self.legos[0])
        x,y,z,b,q,w,e,r = loc
        if(self.ep <1):
            link_pos,link_orn,_,_,_,_ = p.getLinkState(self.panda3.panda,7)
            link_pos2,link_orn2,_,_,_,_ = p.getLinkState(self.panda.panda,7)
            self.init_pos = np.array((x,y,z))
            #self.last_pos = np.concatenate((np.array([0,1.41,-0.3,0]),p.getQuaternionFromEuler([math.pi/2.,0.,0.])))
            self.last_pos = np.concatenate((np.array(link_pos),np.array([1]),np.array(link_orn)))
            self.last_pos2 = np.concatenate((np.array(link_pos2),np.array([1]),np.array(link_orn2)))
        #Calculate Jacobian
	# qdq_matrix = np.array([np.array(p.getJointState(bodyUniqueId=panda.panda, jointIndex=jointIndex, physicsClientId=0)[:2])for jointIndex in np.arange(1, 8)])
	# q = qdq_matrix[:, 0]
	# dq = qdq_matrix[:, 1]
	# jac_t, jac_r = p.calculateJacobian(panda.panda, 7,[0., 0., 0.0], list(q) + [0.] * 2, [0.] * 9, [0.] * 9,physicsClientId=0)
	# J = np.concatenate((np.array(jac_t)[:, :7], np.array(jac_r)[:, :7]), axis=0)
	# print(J)
        
        
        #TODO Proper values for next 3
        self.target_pos = [0.4,1.41,-1.4]
        reward = -1
        done = False
        self.ep +=1 
        if(self.ep >= self.max_ep):
            done = True
            self.ep = 0
            
        #if remote control comment out
        pos = np.array(pos) 
        pos2 = np.array(self.last_pos2)
        pos2[0] = pos2[0] + (x-self.init_pos[0])/200
        pos2[1] = pos2[1] + (y-self.init_pos[1])/200
        pos2[2] = pos2[2] + (z-self.init_pos[2])/200

        #for mirroring autonomous robot
        # pos2 = pos
        # pos2[0] = -pos2[0]
        # pos2[2] = -pos2[0]


        pos = pos + self.last_pos
        #pos2 = pos2 +self.last_pos2
        self.last_pos = pos
        #self.last_pos2 = pos2
        

        pos = tuple(pos)
        pos2 = tuple(pos2)

        self.f.write(str(pos2))
        self.f.write(" , ")
        self.f.write(str(self.ep))
        self.f.write("\n")




        #print("Pos ", pos)
        first_dist = distance.euclidean(self.target_pos, posi)



        self.panda.current_j_pos =  np.array(p.getJointStates(self.panda.panda,np.arange(0,7)))[:,0]
        global force
        force = np.array(p.getJointState(self.panda.panda,7)[2][:3])
        force_2 = np.array(p.getJointState(self.panda3.panda,7)[2][:3])
        #print(force)
        #self.panda2.step2()
        
        des= self.panda2.accurateInverseKinematics(tuple(np.array(pos2) + [600,0,0,0,0,0,0,0]), 0.1,50,rp=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02] )
        #self.panda.step(position = loc)
        #self.panda.step3(des,loc)
        #self.panda2.step(position = tuple(np.array(loc) + [1,0,0,0,0,0,0,0]))
        self.panda.stepCQ(pos2)

        #autonomous robot control
        des2 = self.panda4.accurateInverseKinematics(tuple(np.array(pos) + [3,0,0,0,0,0,0,0]), 0.1,50,rp=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02], mode = True )
        #self.panda3.step3(np.array(p.getJointStates(self.panda3.panda,np.arange(0,7)))[:,0])
        self.panda3.stepCQ(pos)

        #self.panda5.step3(des2)

        p.stepSimulation()
        posi, o = p.getBasePositionAndOrientation(self.legos[0])
        dist = distance.euclidean(self.target_pos, posi)
        reward = (first_dist - dist)*4000
        if(reward < 0):
            reward *= 5
        print("Distance ", dist)
        
        t = time.time() - stime
        t_sleep = self.timeStep -t
        if(t_sleep) >0:
            
            time.sleep(self.timeStep)
        reward -= np.linalg.norm(force_2)/1000
        if(dist <= 0.1):
            reward+=10
            print("REACHED")


        contactsByRobot = p.getContactPoints(self.legos[0],self.panda3.panda)
        contactsByRobot2 = p.getContactPoints(self.legos[0],self.panda.panda)
        forcesByRobot = np.array([0,0,0])
        for v in contactsByRobot:
            forcesByRobot = forcesByRobot + np.array(v[7])*np.array(v[9])

        contactsOnJenga = p.getContactPoints(self.legos[0])
        forcesOnJenga = np.array([0,0,0])
        for v in contactsOnJenga:
            forcesOnJenga = forcesOnJenga + np.array(v[7])*np.array(v[9])

        dot_product = np.dot((forcesOnJenga/np.linalg.norm(forcesOnJenga)),(forcesByRobot/np.linalg.norm(forcesByRobot)))
        projectedVectorNorm = np.linalg.norm(forcesByRobot)*dot_product
        #this is basically THE dot product
        individualEfficiency = projectedVectorNorm/np.linalg.norm(forcesByRobot) 
        print("Individual efficiency: ", individualEfficiency)

        #print("Forces : ", forcesByRobot, " , ", forcesOnJenga)

        if(not np.isnan(individualEfficiency)):
            #reward += individualEfficiency *4
        
            reward = (-abs(0.5-individualEfficiency))*4


        else:
            reward = -5

        keys = p.getKeyboardEvents()
        if len(keys)>0:
            for k,v in keys.items():
                if v&p.KEY_WAS_TRIGGERED:
                    if (k==ord('1')):
                        self.reset()

        if(len(contactsByRobot) == 0 or len(contactsByRobot2) == 0):
            reward = -10
            done = True
            self.ep=0

        print("Reward ", reward)
        state = np.concatenate((np.array(self.target_pos), np.array(force_2),np.array(p.getLinkState(self.panda3.panda,self.panda3.pandaEndEffectorIndex)[0])))
        return state, reward, done, dict()
    def reset(self):
        p.resetSimulation()
        p.setTimeStep(self.timeStep)
        p.setGravity(0,-9.8,0)
        self.setupscene()
        force_2 = np.array(p.getJointState(self.panda3.panda,7)[2][:3])
        state = np.concatenate((np.array(self.target_pos), np.array(force_2),np.array(p.getLinkState(self.panda3.panda,self.panda3.pandaEndEffectorIndex)[0])))
        self.f.write("Reset")
        self.f.write("\n")
        return state


        
        
        
    def setupscene(self):
        flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        self.legos=[]
        p.loadURDF("table/table.urdf", [0.25, 0.2, -0.65], [-0.5, -0.5, -0.5, 0.5], flags=flags)
        
        orn=[-0.707107, 0.0, 0.0, 0.707107]#p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
        q1 = Quaternion(0.707107, -0.707107, 0.0, 0.0 ) # Rotate 180 about X
        q2 = Quaternion(axis=[0,0, 1], angle=3.14159265 ) 
        q3 = q1 * q2 # Composite rotation of q1 then q2 expressed as standard multiplication
        orn = (q3.x, q3.y, q3.z, q3.w)
        q4 = Quaternion(axis= [0,0, 1], angle=math.pi*3/2)
        q4 = q3*q4
        orn2 = (q4.x, q4.y, q4.z, q4.w)
        self.legos.append(p.loadURDF("jenga/jenga.urdf",np.array([0, 1.01, -0.7]),orn2, globalScaling = 1.5))
        obstacle = p.loadURDF("table/table.urdf",np.array([0.2, 0.91, -0.7]),orn2, globalScaling = 0.13)
        p.changeVisualShape(self.legos[0],-1,rgbaColor=[1,0,0,1])
        p.changeVisualShape(obstacle,-1,rgbaColor=[0,1,0,1])

        #p.loadURDF("jenga/jenga.urdf", [0, 0.91, -0.65], [-0.5, -0.5, -0.5, 0.5], globalScaling = 1)
        self.panda = panda_sim_grasp.PandaSimAuto(p,[0,0.81,0],lego=self.legos[0])
        self.panda2 = panda_sim.PandaSim(p,[3,0.81,0])
        #self.panda3 = panda_sim.PandaSim(p,[0.4,0.81,-1.5], rotate = True)
        self.panda4 = panda_sim.PandaSim(p,[3.0,0.81,-1.4], rotate = True)
        self.panda3 = panda_sim_grasp.PandaSimAuto(p,[0.0,0.81,-1.4], rotate = True, lego=self.legos[0])
        p.enableJointForceTorqueSensor(self.panda.panda, 7,1)
        p.enableJointForceTorqueSensor(self.panda3.panda, 7,1)

        

        start = time.time()
        while(time.time() < start+7.0):
            self.panda3.step_g()
            self.panda.step_g()
            loc = location
            global force
            force = np.array(p.getJointState(self.panda.panda,7)[2][:3])
            force_2 = np.array(p.getJointState(self.panda3.panda,7)[2][:3])


            p.stepSimulation()
            time.sleep(self.timeStep)






env = CustomEnv()
check_env(env)
model = SAC(MlpPolicy, env, verbose=1)
model.learn(total_timesteps=5000, log_interval=10)
model.save("sac_approach_duck3")
#model = SAC.load("sac_approach_duck")
#check_env(env,True, True)
i = 0
env = CustomEnv()
obs = env.reset()
while (1):
    # if keyboard.is_pressed('r'):  
    #     env.reset()        #Action space for end effector 3 dimensional
    print("Evaluation")
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
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
	
