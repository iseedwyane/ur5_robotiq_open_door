# -*- coding: utf-8 -*-

import time

import numpy as np
from scipy.optimize import *
import robot


from math import cos as cos
from math import sin as sin
from math import pi as pi

#import rospy
from tf.transformations import euler_from_matrix

mat = np.matrix

global grid_density
global d, a, alph
global joint_angle
global theta_6

grid_density = 7 #越大精确度越高,默认7，rranges范围在pi/2
joint_angle = [-1.0111926237689417, -0.6428774038897913, -0.8211081663714808, 4.438221454620361, 5.097452640533447, -5.2426186243640345]
theta_6 = 0.0 #手腕3关节角度，固定

d = mat([0.089159, 0, 0, 0.10915, 0.09465, 0.0823]) #ur5
a =mat([0 ,-0.425 ,-0.39225 ,0 ,0 ,0]) #ur5
alph = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ])  #ur5

# 正运动学
def AH( n,th  ):


  T_a = mat(np.identity(4), copy=False)
  T_a[0,3] = a[0,n-1]
  T_d = mat(np.identity(4), copy=False)
  T_d[2,3] = d[0,n-1]

  Rzt = mat([[cos(th[n-1]), -sin(th[n-1]), 0 ,0],
	         [sin(th[n-1]),  cos(th[n-1]), 0, 0],
	         [0,               0,              1, 0],
	         [0,               0,              0, 1]],copy=False)


  Rxa = mat([[1, 0,                 0,                  0],
			 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
			 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
			 [0, 0,                 0,                  1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa


  return A_i

def HTrans(th ):
  A_1=AH( 1,th  )
  A_2=AH( 2,th  )
  A_3=AH( 3,th  )
  A_4=AH( 4,th  )
  A_5=AH( 5,th  )
  A_6=AH( 6,th  )

  T_06=A_1*A_2*A_3*A_4*A_5*A_6

  return T_06


# Given coordinates x,y,z,u,v,w;
# Gives the six angles theta 1 - 6 required to 
def find_angles(coordinates):


    def f(p, *parms):
        theta_1, theta_2, theta_3, theta_4, theta_5 = p
        p = np.append(p,theta_6)# theta_6

        p = joint_angle #TEST
    
        #机械臂正运动学，计算机械臂末端位置
        T = HTrans(p) #齐次变换矩阵
        print T
        cartesian = T[0:3,3]
    
        #机械臂正运动学，计算机械臂末端姿态（欧拉角）
        R = T[0:3,0:3]
        eulers = euler_from_matrix(R, 'rxyz') #！！！与matlab转换有差异，待检验
        eulers2 = robot.euler_from_matrix(R) #！！！与matlab转换有差异，待检验
        print eulers
        print eulers2

        #计算当前迭代的位姿与实际需要位姿的差值
        #x, y, z, r, p, y
        diff = np.append(np.array(cartesian), np.array(eulers)) - coordinates
        result = np.dot(diff, diff)

        return result

    rranges = tuple([(-pi/2,0),(-pi/2,0),(-pi/2,0),(pi,3*pi/2),(3*pi/2,2*pi)])
    resbrute = brute(f, rranges, Ns = grid_density, full_output=True)
    print resbrute[1] #error
    return resbrute

if __name__ == "__main__":
    time_start=time.time()
    # (x,y,z,r,p,y)
    coordinates = [-0.35304266144142693, 0.30070653205989784, 0.8412032088936865, -1.6180391224583535, 0.02626281984124196, 1.5541880308473595]
    theta_6 = joint_angle[5]
    print theta_6
    result = np.append(find_angles(coordinates)[0], theta_6)
    result = result.tolist()
    time_end=time.time()
    print result
    print 'totally cost', time_end-time_start, 's'
    
