#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
import copy
import math
import numpy as np
import tf
class MoveItIkDemo:
    def __init__(self):
 
        # 初始化move_group的API，出现roscpp是因为底层是通过C++进行实现的
        moveit_commander.roscpp_initialize(sys.argv)
 
        # 初始化ROS节点，节点名为'moveit_fk_demo'
        rospy.init_node('moveit_ik_demo', anonymous=True)       

        # 初始化需要使用move group控制的机械臂中的arm group
        arm1 = moveit_commander.MoveGroupCommander('left_arm')
        
        # 设置机械臂运动的允许误差值，单位弧度
        arm1.set_goal_position_tolerance(0.01)
        arm1.set_goal_orientation_tolerance(0.01)

        # 设置允许的最大速度和加速度，范围0~1
        arm1.set_max_acceleration_scaling_factor(0.02)
        arm1.set_max_velocity_scaling_factor(0.02)
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'desk_base_link'
        arm1.set_pose_reference_frame(reference_frame)
        # 当运动规划失败后，允许重新规划
        arm1.allow_replanning(True)  
        arm1.set_num_planning_attempts(10)
        # 控制机械臂先回到初始化位置，home是setup assistant中设置的
        # arm1.set_named_target('home')
        # arm1.go(wait=True)  #让机械臂先规划，再运动，阻塞指令，直到机械臂到达home后再向下执行
        # rospy.sleep(1)
        (r, p, y) = tf.transformations.euler_from_quaternion((-0.0030, 0.8739, -0.4852, 0.0280))
        # print(r*180/np.pi,p*180/np.pi,y*180/np.pi)
        # (r, p, y) = tf.transformations.euler_from_quaternion((-0.146692, 0.774175, -0.574882, 0.220554))
        # print(r*180/np.pi,p*180/np.pi,y*180/np.pi)
        # q = tf.transformations.quaternion_from_euler(r, p, y)
        pose_goal = Pose()
        pose_goal.orientation.x = -0.0030 # 0.79840
        pose_goal.orientation.y = 0.8739
        pose_goal.orientation.z = -0.4852
        pose_goal.orientation.w = 0.0280 # 0.60212
        pose_goal.position.x = 0.2 # 0.8
        pose_goal.position.y = 0.0 # 0.5
        pose_goal.position.z = 0.2 #-0.3
        waypoints = []
        arm1.set_pose_target(pose_goal)
        plan = arm1.go(wait=True)
        arm1.stop()
        arm1.clear_pose_targets()

        arm1.set_max_acceleration_scaling_factor(0.02)
        arm1.set_max_velocity_scaling_factor(0.02)
        # pose_goal.position.y = 0.3 # 0.5
        # pose_goal.orientation.x = 0.70692
        # pose_goal.orientation.w = 0.70729
        # arm1.set_pose_target(pose_goal)
        # plan = arm1.go(wait=True)
        # arm1.stop()
        # arm1.clear_pose_targets()
        # end_effector_link = arm1.get_end_effector_link()
        # start_pose = arm1.get_current_pose(end_effector_link).pose
        # 画直线
        # scale = 2
        waypoints_line = []
        for th in np.arange(0.0, 0.4, 0.01):
            pose_goal.position.y = th
            # pose_goal.orientation.x = 0.79840
            # pose_goal.orientation.w = 0.60212
            waypoints_line.append(copy.deepcopy(pose_goal))
        (plan, fraction) = arm1.compute_cartesian_path(
                                waypoints_line,   # waypoints to follow
                                0.5,        # eef_step
                                0.0)         # jump_threshold    
        # new_traj = scale_trajectory_speed(plan, 0.2)     
        arm1.execute(plan, wait=True)
        arm1.stop()
        arm1.clear_pose_targets()

        waypoints_line = []
        for th in np.arange(-0.2, 0.2, 0.01):
            pose_goal.position.x = -th
            roll = r + th + 0.2
            yaw = y + th + 0.2
            pitch = p
            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            pose_goal.orientation.x = q[0]
            pose_goal.orientation.y = q[1]
            pose_goal.orientation.z = q[2]
            pose_goal.orientation.w = q[3]
            waypoints_line.append(copy.deepcopy(pose_goal))
        (plan, fraction) = arm1.compute_cartesian_path(
                                waypoints_line,   # waypoints to follow
                                0.5,        # eef_step
                                0.0)         # jump_threshold    
        # new_traj = scale_trajectory_speed(plan, 0.2)     
        arm1.execute(plan, wait=True)
        arm1.stop()
        arm1.clear_pose_targets()
        # 画圆
        # waypoints.append(pose_goal)
        # centerA = pose_goal.position.x
        # centerB = pose_goal.position.y
        # arm1.set_max_acceleration_scaling_factor(0.02)
        # arm1.set_max_velocity_scaling_factor(0.02)
        # radius = 0.5
        # for th in np.arange(0.6435, 1.5707, 0.01):
        #     pose_goal.position.x = radius * math.cos(th)
        #     pose_goal.position.y = radius * math.sin(th)
        #     # pose_goal.orientation.x = 0.79840
        #     # pose_goal.orientation.w = 0.60212
        #     waypoints.append(copy.deepcopy(pose_goal))
        # reference_frame = 'desk_base_link'
        # arm1.set_pose_reference_frame(reference_frame)
        # (plan, fraction) = arm1.compute_cartesian_path(
        #                                 waypoints,   # waypoints to follow
        #                                 0.5,        # eef_step
        #                                 0.0)         # jump_threshold         
        # arm1.execute(plan,wait=True)
        # arm1.stop()
        # arm1.clear_pose_targets()
        # rospy.sleep(1)  #执行完成后休息1s
 
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    try:
        MoveItIkDemo()
    except rospy.ROSInterruptException:
        pass
