#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import time
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import ur5
import math3d as m3d


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                'elbow_joint', 'wrist_1_joint', 
                'wrist_2_joint', 'wrist_3_joint']

client = None

def get_QR_pose(pose):
    global QR_pose
    pos = [pose.pose.position.x,pose.pose.position.y,pose.pose.position.z]
    orient = m3d.UnitQuaternion(pose.pose.orientation.w,
                                m3d.Vector(pose.pose.orientation.x,
                                pose.pose.orientation.y,
                                pose.pose.orientation.z))
    QR_pose = m3d.Transform(orient,pos).pose_vector

def move2(Q1, Q2):
    # global d
    # d = 0.0
    g = FollowJointTrajectoryGoal() # 关节规划
    g.trajectory = JointTrajectory() # 关节信息
    g.trajectory.joint_names = JOINT_NAMES # 关节名称
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState) # 获取当前机械臂关节角度
        joints_pos = joint_states.position
        # 给出起止路点
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(5.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(10.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def average(nums, default=float('nan')):
    return sum(nums) / float(len(nums)) if nums else default


def open_door():
    Q0, Q2 = ur5.get_goal_angles(QR_pose)
    new_list = [Q0, Q2] # Q0、Q1和Q2都是6个关节角度（弧度）
    Q1 = [average(n) for n in zip(*new_list)] # 直线插补（均值）?
    inp = raw_input("确认开门？ y/n: ")
    if (inp.strip() and inp[0] == 'y'):
        print "开始开门..."
        move2(Q1, Q2)
        print "开门完成！"
    else:
        print "已取消开门操作！"

    
def main():
    rospy.init_node('move_planner', anonymous=True, disable_signals=True)
    
    # QR_Code Pose
    rospy.Subscriber('/visp_auto_tracker/object_position',PoseStamped,get_QR_pose,queue_size = 1)

    global client
    try:
        #client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        while 1:
            open_door()
        
      
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    main()