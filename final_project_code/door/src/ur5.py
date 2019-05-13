#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import urx
import joint_angle_calc as jac
import numpy as np
import math3d as m3d
from math import cos
from math import sin
from math import pi

global rob
rob = urx.robot.Robot("192.168.131.40")

global W2Q, W2B, C2Q, C2L6
# 二维码相对于世界坐标系的位姿W_T_Q
# UR5基座相对于世界坐标系的位姿W_T_B
# 二维码相对于摄像头坐标系的位姿C_T_Q
# UR5 link6相对于摄像头坐标系的位姿C_T_6
W2Q,W2B,C2Q,C2L6 = m3d.Transform(),m3d.Transform(),m3d.Transform(),m3d.Transform()

# 0_T_6
def FK(th):
    return m3d.Transform(jac.get_TCP(th))

def actual_angles(coords,theta_6,mytcp_diff):
    ret = jac.find_angles(coords,theta_6,mytcp_diff)
    result = ret[0].tolist()
    result.append(theta_6)
    return result

# 获取当前tcp位姿和关节角度
def get_coords_Q():
    return rob.getl(),rob.getj()

def set_mytcp(mytcp_diff):
    mytcp = m3d.Transform() # 创建工具端相对于link6的齐次变换矩阵
    mytcp.pos.z = mytcp_diff # 沿link6的z轴正方向移动handle_z_diff
    rob.set_tcp(mytcp)

def set_W2B():
    th = rob.getj()
    W2B = W2Q * C2Q.inverse * C2L6 * FK(th).inverse

# 将工具端的UR5的基座坐标系B_T_T转换为世界坐标系W_T_T
def get_world_coords(coords):
    return (m3d.Transform(W2B)*m3d.Transform(coords)).pose_vector

# 将工具端的世界坐标系W_T_T转换为UR5的基座坐标系B_T_T
def get_base_coords(coords):
    return (m3d.Transform(W2B).inverse*m3d.Transform(coords)).pose_vector

# 计算开门后工具端的位姿
# coords 门把手中心相对UR5基座的位姿
def get_end_pose(coords,angle):
    r = 0.535 # 门半径（m）
    print "door radius (m): ", r
    T1 = m3d.Transform(get_world_coords(coords)) # 门把手中心相对世界坐标系的位姿
    # RZ
    trans = m3d.Transform()
    trans.set_orient(m3d.Orientation.new_euler((0, 0, angle), encoding='xyz'))
    T2 = trans * T1
    T2.set_pos(m3d.Vector(coords[0]-r+r*cos(angle),coords[1]+r*sin(angle),coords[2]))
    return get_base_coords(T2.pose_vector)
    

def set_joint_ang(Q):
    print "angles：", Q
    rob.movej(Q)

def get_goal_angles(QR_pose):
    C2Q = m3d.Transform(QR_pose)
    mytcp_z_diff = 0.118
    #set_mytcp(mytcp_z_diff)
    coords,Q1 = get_coords_Q()
    coords2 = get_end_pose(coords,pi/6)
    # 计算逆运动学对应的6个实际关节角度
    theta_6 = Q1[5] # 手腕3关节角度，固定
    Q2 = actual_angles(coords2,theta_6,mytcp_z_diff)
    # 比较两组关节角
    print "start joint angles:", Q1
    print "goal joint angles:", Q2
    #set_joint_ang(Q2)
    #print "执行完毕"
    return Q1,Q2

# if __name__ == '__main__':
#     get_goal_angles(QR_pose)