#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import numpy as np
import math3d as m3d
from tf.transformations import *
from scipy.optimize import *
from math import radians
from math import cos
from math import sin
from math import pi

mat = np.matrix

global grid_density
grid_density = 7 # 越大精确度越高,计算时间越长,默认7

global d, a, alph
# 连杆偏距
d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
# 连杆长度
a = [0, -0.425, -0.39225, 0, 0, 0]
# 连杆扭转角
alph = [pi/2, 0, 0, pi/2, -pi/2, 0]

global origin, xaxis, yaxis, zaxis 
origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

global T_a, T_d, T
T_a = identity_matrix()
T_d = identity_matrix()
T = [identity_matrix()]*7


### 正运动学
# D-H坐标变换
# 输入： n   从(n-1)到n坐标系的齐次变换矩阵
#       th  关节角度
# 输出： T_i 从(n-1)到n坐标系的齐次变换矩阵
def AH(n, th):
  T_a[0,3] = a[n]
  T_d[2,3] = d[n]
  # Z轴旋转变换
  Rzt = rotation_matrix(th[n], zaxis)
  # x轴旋转变换
  Rxa = rotation_matrix(alph[n], xaxis)
  T_i = concatenate_matrices(T_d, Rzt, T_a, Rxa)
  return T_i

def set_MyTCP(mytcp_diff):
  T[6][2,3] = mytcp_diff

def get_MyTCP(th, mytcp_diff):
  TCP = get_TCP(th)
  # MyTCP
  set_MyTCP(mytcp_diff)
  # 基坐标到工具坐标的齐次变换矩阵
  MyTCP = np.dot(TCP,T[6])
  return m3d.Transform(MyTCP).pose_vector

# 计算基坐标到工具端坐标变换
# 输入： th            关节角度
#       mytcp_diff 夹爪中心相对于link6的z方向误差
# 输出： MyTCP       基坐标到工具端的齐次变换矩阵
def get_TCP(th):
  # T1-T6
  for i in range(0, 6):
    T[i] = AH(i, th)
  
  TCP = identity_matrix()
  for i in range(len(T)):
    TCP=np.dot(TCP,T[i])

  return TCP

### 逆运动学
# 输入： (x,y,z,r,p,y)  机械臂工具端位姿
#        theta_6       手腕3关节固定角度
#        mytcp_diff 夹爪中心相对于link6的z方向误差（m）
# 注意： (r,p,y)使用ROS自带的旋转矩阵到欧拉角函数，但计算结果与urx中的rob.getl()获取的值不同
#       为保证计算结果统一，统一采用自带函数将计算所得的旋转矩阵换算为欧拉角
# 输出： 逆运动学数值解
#       resbrute[0]  前5个关节角度（除去手腕3的关节角度）
#       resbrute[1]  数值解与期望位姿的误差
def find_angles(coordinates,theta_6,mytcp_diff,run=1):
  def f(p, *parms):
      theta_1, theta_2, theta_3, theta_4, theta_5 = p 
      p = np.append(p,theta_6)# theta_6

      #机械臂正运动学，计算机械臂位姿
      pose = get_MyTCP(p,mytcp_diff) #齐次变换矩阵

      #计算当前迭代的位姿与实际需要位姿的差值
      #x, y, z, rx, ry, rz
      diff = pose - coordinates
      result = np.dot(diff, diff)

      return result
  # 角度范围限制（开门前后的角度变化范围，限制唯一解）
  rranges = rranges = tuple([(radians(-30),  radians(0)),
                             (radians(-150), radians(-100)),
                             (radians(90),   radians(100)),
                             (radians(200),  radians(220)),
                             (radians(300),  radians(360))])
  #rranges = tuple([(-2*pi,2*pi)]*5)
  # if run == 0: # 开门前
  #   rranges = tuple([(-2*pi,-pi),(-pi,0),(0,pi),(pi,2*pi),(pi,2*pi)])
  # else: # 开门后
  #   rranges = tuple([(-2*pi,2*pi)]*5)
  time_start = time.time()
  resbrute = brute(f, rranges, Ns = grid_density, full_output=True)
  time_end = time.time()
  print "error：", resbrute[1] #error
  print "total cost：", time_end-time_start, "s"
  return resbrute
