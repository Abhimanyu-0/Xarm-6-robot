import numpy as np
import math as m 
import matplotlib.pyplot as plt
import time
import scip

# Implementing class/library for kinematics
class Robot:
    
    def __init__(self, theta1,theta2,theta3,theta4,theta5,theta6):
        # Initialize the robot with joint angles
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.theta5 = theta5
        self.theta6 = theta6
        
    # This is transformation from frame 1 to 0     

    def _0T1(self):
        x_in = np.radians(self.theta1)
        d1 = 1
        a1 = 0
        
        _T1 = np.array([[m.cos(-m.pi/2 + x_in ), -m.cos(m.pi/2)*m.sin(-m.pi/2+ x_in), m.sin(m.pi/2)*m.sin(-m.pi/2+x_in), a1*m.cos(-m.pi/2)  ],[ m.sin(-m.pi/2+x_in) , m.cos(m.pi/2)*m.cos(-m.pi/2+x_in), -m.sin(m.pi/2)*m.cos(-m.pi/2+x_in), a1*m.sin(-m.pi/2) ],[ 0 , m.sin(m.pi/2), m.cos(m.pi/2), d1 ],[ 0 , 0, 0, 1]])
       
        return _T1
    
    # This is transformation from frame 2 to 1    
    def _1T2(self):
        x_in = np.radians(self.theta2)
        a2 = 1
        
        _T2 = np.array([[m.cos(m.pi/2 + x_in ), -m.cos(0)*m.sin(m.pi/2+ x_in), m.sin(0)*m.sin(m.pi/2+x_in), a2*m.cos(m.pi/2) ],[ m.sin(m.pi/2+x_in) , m.cos(0)*m.cos(m.pi/2+x_in), -m.sin(0)*m.cos(m.pi/2+x_in), a2*m.sin(m.pi/2) ],[ 0 , m.sin(0), m.cos(0), 0 ],[ 0 , 0, 0, 1]]) 
        return _T2 
    
    # This is transformation from frame 3 to 2
    def _2T3(self):
        x_in = np.radians(self.theta3)
        a3= 0
        d3 = 2
        
        _T3 = np.array([[m.cos(-m.pi/2 + x_in ), -m.cos(-m.pi/2)*m.sin(m.pi/2+ x_in), m.sin(-m.pi/2)*m.sin(m.pi/2+x_in), a3*m.cos(-m.pi/2) ],[ m.sin(-m.pi/2+x_in) , m.cos(-m.pi/2)*m.cos(-m.pi/2+x_in), -m.sin(-m.pi/2)*m.cos(-m.pi/2+x_in), a3*m.sin(-m.pi/2) ],[ 0 , m.sin(-m.pi/2), m.cos(-m.pi/2), d3 ],[ 0 , 0, 0, 1]])
        return _T3
    
    # This is transformation from frame 4 to 3
    def _3T4(self):
        x_in = np.radians(self.theta4)
        d4 = 1.1
        a4 = 0
        _T4 = np.array([[m.cos( x_in ), -m.cos(m.pi/2)*m.sin( x_in), m.sin(m.pi/2)*m.sin(x_in), a4*m.cos(0) ],[ m.sin(x_in) , m.cos(m.pi/2)*m.cos(x_in), -m.sin(m.pi/2)*m.cos(x_in), a4*m.sin(0) ],[ 0 , m.sin(m.pi/2), m.cos(m.pi/2), d4 ],[ 0 , 0, 0, 1]])
        return _T4
    
    # This is transformation from frame 5 to 4
    def _4T5(self):
        x_in = np.radians(self.theta5)
        a5 = 2
        d5 = 0.8
        _T5 = np.array([[m.cos(m.pi + x_in ), -m.cos(-m.pi/2)*m.sin(m.pi+ x_in), m.sin(-m.pi/2)*m.sin(m.pi +x_in), a5*m.cos(m.pi/2) ],[ m.sin(m.pi+x_in) , m.cos(-m.pi/2)*m.cos(m.pi+x_in), -m.sin(-m.pi/2)*m.cos(m.pi+x_in), a5*m.sin(m.pi/2) ],[ 0 , m.sin(-m.pi/2), m.cos(-m.pi/2), d5 ],[ 0 , 0, 0, 1]])
        return _T5
    
    def _5T6(self):
        x_in = self.theta6
        a6 = 10
        d6 = 0
        _T6 = np.array([[m.cos(0 + x_in ), -m.cos(0)*m.sin(0+ x_in), m.sin(0)*m.sin(+x_in), a6*m.cos(x_in) ],[ m.sin( x_in) , m.cos(0)*m.cos(x_in), -m.sin(0)*m.cos( x_in), a6*m.sin(x_in) ],[ 0 , m.sin(0), m.cos(0), d6 ],[ 0 , 0, 0, 1]])
        return _T6

    
        
    def _jointpos(self):
        # Compute joint positions in frames 0, 1, 2, 3, 4
        T1 = self._0T1
        T2 = self._1T2
        T3 = self._2T3
        T4 = self._3T4
        T5 = self._4T5
        T6 = self._5T6
        op1 = np.matmul(T1(),T2()) # frame 2 to 0
        op2 = np.matmul(op1, T3()) # frame 3 to 0
        op3 = np.matmul(op2, T4()) # frame 4 to 0  
        op4 = np.matmul(op3, T5()) # frame 5 to 0
        op5 = np.matmul(op4, T6()) # frame 6 to 0
        return T1()[:,3] ,op1[:,3], op2[:,3], op3[:,3], op4[:,3]
    
    def _jointsvec(self):
        # Compute joint vectors for frames 0, 1, 2, 3, 4, 5
        T1 = self._0T1
        T2 = self._1T2
        T3 = self._2T3
        T4 = self._3T4
        T5 = self._4T5
        T6 = self._5T6
        op1 = np.matmul(T1(),T2()) # frame 2 to 0
        op2 = np.matmul(op1, T3()) # frame 3 to 0
        op3 = np.matmul(op2, T4()) # frame 4 to 0  
        op4 = np.matmul(op3, T5()) # frame 5 to 0
        op5 = np.matmul(op4, T6()) # frame 6 to 0 
        return T1()[:,2], op1[:,2], op2[:,2], op3[:,2], op4[:,2], op5[:,2]
        
    def _endeff(self):
         # Compute end effector transformation matrix in frame 0
        T1 = self._0T1
        T2 = self._1T2
        T3 = self._2T3
        T4 = self._3T4
        T5 = self._4T5
        T6 = self._5T6
        op1 = np.matmul(T1(),T2()) # frame 2 to 0
        op2 = np.matmul(op1, T3()) # frame 3 to 0
        op3 = np.matmul(op2, T4()) # frame 4 to 0  
        op4 = np.matmul(op3, T5()) # frame 5 to 0
        op5 = np.matmul(op4, T6())
       
        return op5
    
    def _ori(self):
        # Compute orientation angles for end-effector angles
        T1 = self._0T1
        T2 = self._1T2
        T3 = self._2T3
        T4 = self._3T4
        T5 = self._4T5
        T6 = self._5T6
        op1 = np.matmul(T1(),T2()) # frame 2 to 0
        op2 = np.matmul(op1, T3()) # frame 3 to 0
        op3 = np.matmul(op2, T4()) # frame 4 to 0  
        op4 = np.matmul(op3, T5()) # frame 5 to 0
        op5 = np.matmul(op4, T6())
        theta_x = np.degrees(np.arctan2(op5[2,1],op5[2,2]))
        theta_y = np.degrees(np.arctan2(op5[2,0], m.sqrt((op5[2,1])**2+ (op5[2,2])**2)))
        theta_z = np.degrees(np.arctan2(op5[1,0],op5[0,0]))
        
        return np.array([theta_x, theta_y, theta_z]) 
    
    def _Jacobian(self):
        # Compute the Jacobian matrix for the robot
        zs =  Robot._jointsvec(self)
        end_effector_position = Robot._endeff(self)
        p6 = end_effector_position[:,3][:3] #p6-p0
        pos2 = p6 - zs[0][:3] #p6 - p1
        pos3 = p6- zs[1][:3] #p6 - p2
        pos4 = p6 - zs[2][:3]#p6 - p3
        pos5 = p6 - zs[3][:3] #p6 - p4
        pos6 = p6 - zs[4][:3] #  p6 -p5
        
        z0 = np.array([0,0,1]).transpose()
        entry1 = np.cross(z0,p6)
        entry2 = np.cross(zs[0][:3],pos2)
        entry3 = np.cross(zs[1][:3],pos3)
        entry4 = np.cross(zs[2][:3],pos4)
        entry5 = np.cross(zs[3][:3],pos5)
        entry6 = np.cross(zs[4][:3],pos6)
        
        e1 = np.expand_dims(entry1,axis=0).transpose()
        E1 = np.append(e1,z0)

        e2 = np.expand_dims(entry2, axis=0).transpose()
        E2 = np.append(e2,zs[0][:3])

        e3 = np.expand_dims(entry3, axis=0).transpose()
        E3 = np.append(e3,zs[1][:3])

        e4 = np.expand_dims(entry4, axis=0).transpose()
        E4 = np.append(e4,zs[2][:3])

        e5 = np.expand_dims(entry5, axis=0).transpose()
        E5 = np.append(e5,zs[3][:3])

        e6 = np.expand_dims(entry6, axis=0).transpose()
        E6 = np.append(e6,zs[4][:3])

        J = np.column_stack((E1,E2,E3,E4,E5,E6))
        
        return J 
        
    """    
    def _Firstori(self):
        T1 = self._0T1
        T2 = self._1T2
        T3 = self._2T3
        T4 = self._3T4
        T5 = self._4T5
        T6 = self._5T6
        op1 = np.matmul(T1(),T2()) # frame 2 to 0
        op2 = np.matmul(op1, T3()) # frame 3 to 0
        op3 = np.matmul(op2, T4()) # frame 4 to 0  
        op4 = np.matmul(op3, T5()) # frame 5 to 0
        op5 = np.matmul(op4, T6())
        theta_x = np.degrees(np.arctan2(T1()[2,1],T1()[2,2]))
        theta_y = np.degrees(np.arctan2(T1()[2,0], m.sqrt((T1()[2,1])**2+ (T1()[2,2])**2)))
        theta_z = np.degrees(np.arctan2(T1()[1,0],T1()[0,0]))
        
        return theta_x, theta_y, theta_z , op2
    y
"""
