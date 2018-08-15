#!/usr/bin/env python


import os
import numpy as np
import build.custom_main as custom_main


# set env variable to lgazebo model dirs
os.environ['GAZEBO_MODEL_PATH'] = os.path.abspath("models")







sim = custom_main.Gazebo_Sim([""])

sim.loadWorld(os.path.abspath("models/room_closed_10m_10m_single_obstacle/room_closed_10m_10m_single_obstacle.world"))




sim.loadRobot("model://simple_robot")


sim.start()


print("Start simulation...")    
i = 0
total_steps = 0
while(i < 50):
    
    print("Start episode "+str(i)+":")
    #sim.setRobotPose(x=1, y=5, z=0.2, roll=0, pitch=0, yaw=0)
    #sim.reset()
    sim.setRobotPose(x=1, y=5, z=0.2, roll=0, pitch=0, yaw=0)
    sim.setGoalPose(x=7, y=5, z=0.2, roll=0, pitch=0, yaw=0)
    
    sim.reset()
    
    
    j = 0
    while(j < 100000):
        a = (0.7, 0.0, 0.0)
        
        sim.executeAction(*a)
        if sim.makeSteps(100) is True:
            sim.stop()
            print("    Collision !!!")
            total_steps += j
            break
            
        b = np.zeros(3)
        sim.getRobotVelocity(b)
        #print(b)
            
        j += 1
        
    
    print("    total steps: "+str(total_steps))
        
        
    i += 1


sim.stop()

print("Exit python")    
