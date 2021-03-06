#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Path, OccupancyGrid,Odometry
from geometry_msgs.msg import PoseStamped, Quaternion,Twist,Pose
import tf
import math
import nav_msgs

# 起始运动状态

class GlobalPath:
    """decision module Class
    """
    def __init__(self):
        self.x = 0 
        self.y = 0, 
        self.theta = 0
        self.globalMap = nav_msgs.OccupancyGrid()  # global map for global planner
        self.globalMapInitialized = False

        self.globalPath = Path()  # global path record
        self.globalPathInitialized = False

        self.odometry = Odometry()
        self.odometryInitialized = False
        self.startPose = Pose()  # actual pose :  Pose
        self.act_vel = Twist()  # actual velocity :   Twist
        self.goalPose =  Pose() # goal pose : Pose
        self.startPoseInitialized = False
        self.goalPosenitialized = False

    def LineMotion(self,startPosition,endPosition):
        


    def DataUpdating(self,path_pub, path_record):
        """
        数据更新函数
        """
        # 发布tf
        tf_br = tf.TransformBroadcaster()
        tf_br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(), "map", "map")
        # 配置运动
        dt = 0.1  # 1 / 50
        vx = 1  # 0.25
        vy = 1
        vth = 0.2
        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_th = vth * dt
        x += delta_x
        y += delta_y
        th += delta_th
        # 四元素转换
        quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # 时间戳
        current_time = rospy.Time.now()
        # 配置姿态
        pose = PoseStamped()
        pose.header.stamp = current_time
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        # 配置路径
        path_record.header.stamp = current_time
        path_record.header.frame_id = 'map'
        path_record.poses.append(pose)

        # 路径数量限制
        if len(path_record.poses) > 2000:
            path_record.poses.pop(0)

    # 发布路径
        path_pub.publish(path_record)


    def node():
        """
        节点启动函数
        """
        try:
            # 初始化节点path
            rospy.init_node('globalPath_publish_node')
            # 定义发布器 path_pub 发布 trajectory
            self.path_pub = rospy.Publisher('/ow/global_path', Path, queue_size=50)
            # 初始化循环频率
            rate = rospy.Rate(10)
            # 定义路径记录
            rospy.loginfo("The program of path_publish_node  is running ...")
            # 在程序没退出的情况下
            while not rospy.is_shutdown():
                # 数据更新函数
                DataUpdating()
                # 休眠
                rate.sleep()
        except Exception as e:
            rospy.logfatal("path_publish_node has a Exception : %s", e)  # [FATAL]
            # rospy.logerr("程序异常终止！%s", e)  # [ERROR] [error code] RED
            # # rospy.loginfo("程序异常终止！%s", e)  # [INFO]


if __name__ == '__main__':
    globalPath = GlobalPath()
