from pyrobot.brain import Brain

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyrobot.tools.followLineTools import findLineDeviation
from sklearn import preprocessing
import math
import numpy as np


class BrainFollowLine(Brain):

    NO_FORWARD = 0
    SLOW_FORWARD = 0.1
    NORM_FORWARD = 0.25
    MED_FORWARD = 0.6 #0.5
    FULL_FORWARD = 1.0

    NO_TURN = 0
    MED_LEFT = 0.5
    HARD_LEFT = 1.0
    MED_RIGHT = -0.5
    HARD_RIGHT = -1.0

    NO_ERROR = 0

    goalFound = False
    obstacle = False
    xyGoal = (4.0, 0.0) #4.0
    #xyGoal = (3.0, 0.0)


    def destroy(self):
        cv2.destroyAllWindows()

    # ------------- SONARS-----------

    def frontSonar(self):
        #front = min([s.distance() for s in self.robot.range['front']])
        return min([s.distance() for s in self.robot.range['front']])

    def frontRightSonar(self):
        #front = min([s.distance() for s in self.robot.range['front']])
        return min([s.distance() for s in self.robot.range['front-right']])

    def frontLeftSonar(self):
        #front = min([s.distance() for s in self.robot.range['front']])
        return min([s.distance() for s in self.robot.range['front-left']])

    def leftSonar(self):
        #front = min([s.distance() for s in self.robot.range['front']])
        return min([s.distance() for s in self.robot.range['left']])

    def rightSonar(self):
        #front = min([s.distance() for s in self.robot.range['front']])
        return min([s.distance() for s in self.robot.range['right']])

# ------------- SONARS-----------

# ------------- X Y ROBOT -----------
    def xy(self):  # self.robot.x
        return (self.robot.x, self.robot.y)

# ------------- X Y ROBOT -----------

    def step(self):
        # take the last image received from the camera and convert it into
        # opencv format
        

        # print("findLineDeviation returned ",foundLine,error)

        if not self.goalFound:
            (rx, ry) = self.xy()
            (gx, gy) = self.xyGoal 
            
            mrx = gx - 0.05 #0.15
            Mrx = gx + 0.05
            mry = gy - 0.05
            Mry = gy + 0.05
            
            th = self.robot.th
            
            print(rx, ry, th)
            
            if (rx> mrx) and (rx< Mrx) and (ry> mry) and (ry< Mry):     
                self.goalFound = True
                self.obstacle = False
                print("GOAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAL")
                return self.move(self.NO_FORWARD, self.NO_TURN)
            else:              
                                              
                if self.leftSonar() < 0.8:
                    print("Left: ", self.leftSonar())
                    self.obstacle = True
                    return self.move(self.MED_FORWARD, -0.3)#-0.2
                elif self.rightSonar() < 0.8:              
                    print("Right: ",self.rightSonar())
                    self.obstacle = True
                    return self.move(self.MED_FORWARD, 0.3)#0.2
                    
                elif self.frontSonar() < 0.75:  # 1.0
                    print("Front: ", self.frontSonar())
                    self.obstacle = True
                    return self.move(self.SLOW_FORWARD, self.HARD_LEFT) # NO_FORWARD
                elif self.frontLeftSonar() < 1.1:
                    print("Front LEFT: ", self.frontLeftSonar())
                    self.obstacle = True
                    return self.move(self.MED_FORWARD, self.MED_RIGHT)
                elif self.frontRightSonar() < 1.1:    
                    print("Front RIGHT: ", self.frontRightSonar())
                    self.obstacle = True
                    return self.move(self.MED_FORWARD, self.MED_LEFT)
                     
                else:
                    self.obstacle = False
                    print("NO OBSTACLE")
                    
                    if th>180:
                    	th = th-360
                    
                    goalAngle = math.degrees(math.atan2(gy - ry, gx -rx)) -th
                    """if math.degrees(math.atan2(gy - ry, gx -rx))<0:
                    	print("converted:", math.degrees(math.atan2(gy - ry, gx -rx))+360, "deg atan2:", math.degrees(math.atan2(gy - ry, gx -rx)), "rad atan2:", math.atan2(gy - ry, gx -rx),"th:", th )
                    else:
                    	print(math.degrees(math.atan2(gy - ry, gx -rx)),"deg atan2:", math.degrees(math.atan2(gy - ry, gx -rx)), "rad atan2:", math.atan2(gy - ry, gx -rx),"th:", th)"""
                    print(goalAngle)
                    if goalAngle<=3 and goalAngle>=-3:#goalAngle==0
                    	return self.move(self.FULL_FORWARD, self.NO_TURN)
                    elif goalAngle <-3 and goalAngle > -90:
                    	return self.move(0.2, self.MED_RIGHT)	#self.NO_FORWARD
                    elif goalAngle < -90:
                    	return self.move(0.2, self.HARD_RIGHT)	#self.NO_FORWARD
                    else:
                    	return self.move(self.NO_FORWARD, self.NO_TURN)
                    
                 
                    	
                    """print(math.degrees(math.atan2(gy - ry, gx -rx)) )
                    if th > 180:
                    	th = -(360-th)
                    goalAngle = math.degrees(math.atan2(gy - ry, gx -rx)) -(th)
                    print("ANGULO:")    
                    #print(th)   
                    print(goalAngle)        
                    
                    if goalAngle <0:
                        if goalAngle> -10:
                            print("POCO ANGLE")                        	
                            return self.move(self.SLOW_FORWARD, -0.10)
                        elif goalAngle< -180:
                            print("MUCHO ANGLE")                                                                              
                            return self.move(self.MED_FORWARD, -0.25)
                        else:
                            print("NORMAL ANGLE") 
                            return self.move(self.MED_FORWARD, self.MED_RIGHT)                                            
                        print("GIRA A LA DERECHA")                    
                    elif goalAngle == 0:
                        return self.move(self.MED_FORWARD, self.NO_TURN)                    	    
                    elif goalAngle < 5:
                        print("NO GIRA")
                        return self.move(self.MED_FORWARD, self.NO_TURN)                    	
                    elif (goalAngle> 5) and (goalAngle< 360) :
                        print("GIRA A LA IZQUIERDA")
                        return self.move(self.NO_FORWARD, self.MED_LEFT) 
                    	
                    #return self.move(self.MED_FORWARD, self.NO_TURN)"""
        else:
            
            return self.move(self.NO_FORWARD, self.NO_TURN)




def INIT(engine):
    assert (engine.robot.requires("range-sensor") and
            engine.robot.requires("continuous-movement"))

    return BrainFollowLine('BrainFollowLine', engine)


