#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy, sys
import moveit_commander
 
class MoveItFkDemo:
    def __init__(self):
 
        # 初始化move_group的API，出现roscpp是因为底层是通过C++进行实现的
        moveit_commander.roscpp_initialize(sys.argv)
 
        # 初始化ROS节点，节点名为'moveit_fk_demo'
        rospy.init_node('moveit_fk_demo', anonymous=True)       

        # 初始化需要使用move group控制的机械臂中的arm group
        arm1 = moveit_commander.MoveGroupCommander('left_arm')
        
        # 设置机械臂运动的允许误差值，单位弧度
        arm1.set_goal_joint_tolerance(0.01)

 
        # 设置允许的最大速度和加速度，范围0~1
        arm1.set_max_acceleration_scaling_factor(0.3)
        arm1.set_max_velocity_scaling_factor(0.3)
   
        reference_frame = 'base_link'
        arm1.set_pose_reference_frame(reference_frame)
        arm1.allow_replanning(True)  
        # 控制机械臂先回到初始化位置，home是setup assistant中设置的
        arm1.set_named_target('home')
        arm1.go()  #让机械臂先规划，再运动，阻塞指令，直到机械臂到达home后再向下执行
        rospy.sleep(1)

        arm1.set_named_target('pose')
        arm1.go()  #让机械臂先规划，再运动，阻塞指令，直到机械臂到达home后再向下执行
        rospy.sleep(1)
         
        # # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions_left = [-1.8674, -2.618, 0.1745, 0.5934, 0.5, -1.8674]        
        joint_positions_right = [1.3788, -0.5934, -0.8028, 0.5934, 0.5, -1.8]
        arm1.set_joint_value_target(joint_positions_left)  #设置关节值作为目标值
        # arm2.set_joint_value_target(joint_positions_right)  #设置关节值作为目标值
       # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125]
        arm1.set_joint_value_target(joint_positions)  #设置关节值作为目标值

        # # 控制机械臂完成运动
        arm1.go()   # 规划+执行        
        rospy.sleep(1)
 
        # 控制机械臂先回到初始化位置
        arm1.set_named_target('home')
        arm1.go()
        rospy.sleep(1)
        
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
