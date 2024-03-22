#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
import random
from visualization_msgs.msg import Marker
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PointStamped  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from std_msgs.msg import Int8

STATUS_EXPLORING    = 0
STATUS_CLOSE_TARGET = 1
STATUS_GO_HOME      = 2

# 探索
target_pose = [0, 0]

# 视觉
goal_pose = [0, 0]

time_start = 0

class ExploringMaze():
    global target_pose 
    global goal_pose
    global time_start

    def __init__(self):  
        rospy.init_node('exploring_maze', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  # shutdown是用来在CTRL+C取消后，给机器人发布一个0速度的指令

        # 在每个目标位置暂停的时间 (单位：s)
        self.rest_time = rospy.get_param("~rest_time", 0.5)  

        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5) 
 
        # 创建一个Subscriber，订阅/exploring_cmd
        # rospy.Subscriber("/exploring_cmd", Int8, self.cmdCallback)

        # rospy.Subscriber("/assigned_centroid", PointArray, self.callback)
        rospy.Subscriber('/goal_visual', Marker, self.callback)

        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  # 订阅move_base服务器，相当于这是个客户端用来给导航发布Goal指令的

        rospy.loginfo("Waiting for move_base action server...")  
        
        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
 
        # 保存运行时间的变量   
        start_time = rospy.Time.now()  
        running_time = 0  

        rospy.loginfo("Starting exploring maze")  
        
        # 初始位置
        start_location = Pose(Point(0, 0, 0), Quaternion(0.000, 0.000, 0.709016873598, 0.705191515089))  	# 设定初始位置实际上就是零位置HOME
 
        # 命令初值
        self.exploring_cmd = 0
        rospy.Subscriber('/object_detect_pose', PointStamped, self.poseCallback) 

        # 开始主循环，随机导航  
        while not rospy.is_shutdown():
          # 设定下一个随机目标点
          self.goal = MoveBaseGoal()
          self.goal.target_pose.pose = start_location	# 先回到初始点
          self.goal.target_pose.header.frame_id = 'map'
          self.goal.target_pose.header.stamp = rospy.Time.now()	# 把当前时间发布
          
          if self.exploring_cmd is STATUS_EXPLORING:	# 探索，随机点导航
              if target_pose[0] != 0 and target_pose[1] != 0:
                  self.goal.target_pose.pose.position.x = target_pose[0]
                  self.goal.target_pose.pose.position.y = target_pose[1]
          elif self.exploring_cmd is STATUS_CLOSE_TARGET:	# 视觉伺服
              if goal_pose[0] != 0 and goal_pose[1] != 0 and target_pose[0] != 0 and target_pose[1] != 0:
                  self.goal.target_pose.pose.position.x = goal_pose[0]
                  self.goal.target_pose.pose.position.y = goal_pose[1]
          elif self.exploring_cmd is STATUS_GO_HOME:	# 返航
               self.goal.target_pose.pose.position.x = 0
               self.goal.target_pose.pose.position.y = 0
		
            # 让用户知道下一个位置
          rospy.loginfo("Going to:" + str(self.goal.target_pose.pose))
          rospy.loginfo("Exploring cmd:" + str(self.exploring_cmd))

            # 向下一个位置进发
          if self.goal.target_pose.pose.position.x != 0 and self.goal.target_pose.pose.position.y != 0 or self.exploring_cmd is STATUS_GO_HOME:
            self.move_base.send_goal(self.goal)
			
          # 五分钟时间限制
          if self.exploring_cmd is STATUS_CLOSE_TARGET:
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(5))
          else:
            rospy.sleep(1)

          # 查看是否成功到达
          # if not finished_within_time:
          #   self.move_base.cancel_goal()
          #   # rospy.loginfo("Timed out achieving goal")
          # else:
          #   state = self.move_base.get_state()
          #   if state == GoalStatus.SUCCEEDED:
          #     # rospy.loginfo("Goal succeeded!")
          #     pass
          #   else:
          #     pass
          #     # rospy.loginfo("Goal failed!")
          # 运行所用时间
          running_time = rospy.Time.now() - start_time
          running_time = running_time.secs / 60.0

          # 输出本次导航的所有信息
          # rospy.loginfo("current time:" + str(trunc(running_time,1))+"min")
          self.shutdown()

    # def cmdCallback(self, msg):
    #     rospy.loginfo("Receive exploring cmd : %d", msg.data)
    #     self.exploring_cmd = msg.data

    #     if self.exploring_cmd is STATUS_CLOSE_TARGET:
    #         rospy.loginfo("Stopping the robot...")
    #         self.move_base.cancel_goal()

    def poseCallback(self, msg):
        global goal_pose
        global time_start
        goal_pose[0] = msg.point.x
        goal_pose[1] = msg.point.y
        self.exploring_cmd = STATUS_CLOSE_TARGET
        time_start = msg.header.stamp.secs

    def callback(self, msg):
        global target_pose
        global time_start
        for point in msg.points:
            print("x: {}, y: {}, z: {}".format(point.x, point.y, point.z))
            target_pose[0] = point.x
            target_pose[1] = point.y
            time = msg.header.stamp.secs
            if time_start != 0 and time - time_start > 1:
                self.exploring_cmd = STATUS_EXPLORING

        # rospy.loginfo("Receive exploring cmd : %d", msg.points[0])
        # self.exploring_cmd = msg.data
        pass

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        # rospy.sleep(2)  
        # self.cmd_vel_pub.publish(Twist())
        # rospy.sleep(1)  

def trunc(f, n):  
    slen = len('%.*f' % (n, f))  
    return float(str(f)[:slen])  

if __name__ == '__main__':  
    try:  
        ExploringMaze()  
        rospy.spin()  
        # spin一直在订阅exploring_cmd如果flag有变化，那么机器人也在导航和视觉伺服直接跳换

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring maze finished.")