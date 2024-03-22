#!/usr/bin/env python3

import rospy
import tf
from numpy import array
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from std_msgs.msg import Header
from numpy import floor
from numpy.linalg import norm
from visualization_msgs.msg import Marker
from numpy import inf
import numpy as np
from copy import copy
from gazebo_msgs.srv import SpawnModel, DeleteModel


# ________________________________________________________________________________


class robot:
    def __init__(self, name):
        #################################################################
        self.goal = MoveBaseGoal()
        self.start = PoseStamped()
        self.end = PoseStamped()
        #################################################################     
        self.assigned_point = []
        self.name = name
        self.global_frame = rospy.get_param('~global_frame', '/map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_footprint')
        self.plan_service = rospy.get_param(
            '~plan_service', '/move_base/NavfnROS/make_plan')
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            self.global_frame, self.name + '/' + self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name + '/' + self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        self.position = array([trans[0], trans[1]])
        self.assigned_point = self.position
        self.client = actionlib.SimpleActionClient(
            self.name + '/move_base', MoveBaseAction)
        self.client.wait_for_server()
        ###################################################################
        self.goal.target_pose.header.frame_id = self.global_frame
        self.goal.target_pose.header.stamp = rospy.Time.now()
        ###################################################################
        self.total_distance = 0
        self.first_run = True
        self.movebase_status = 0
        self.sub = rospy.Subscriber(name + "/odom", Odometry, self.odom_callback)
        #####################################################################
        self.position = array([trans[0], trans[1]])
        self.previous_x = 0
        self.previous_y = 0
        self.assigned_point = self.position
        ######################################################################
        rospy.wait_for_service(self.name + self.plan_service)
        self.make_plan = rospy.ServiceProxy(
            self.name + self.plan_service, GetPlan)
        self.start.header.frame_id = self.global_frame
        self.end.header.frame_id = self.global_frame
        self.del_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)


    def odom_callback(self, data):
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, data.header.frame_id, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0

        if self.first_run == True:
            self.previous_x = trans[0]
            self.previous_y = trans[1]
        x = trans[0]
        y = trans[1]
        d_increment = np.sqrt(
            ((x - self.previous_x) * (x - self.previous_x)) + ((y - self.previous_y) * (y - self.previous_y)))
        self.total_distance = self.total_distance + d_increment
        # print("Total distance traveled is %.2f" %(self.total_distance))
        self.first_run = False
        self.previous_x = x
        self.previous_y = y

    def getPosition(self):
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name + '/' + self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        self.position = array([trans[0], trans[1]])
        return self.position

    def pub_position(self):
        # 创建一个 PointStamped 消息对象
        point_stamped_msg = PointStamped()

        # 设置头部信息（时间戳等）
        point_stamped_msg.header = Header()
        point_stamped_msg.header.stamp = rospy.Time.now()
        point_stamped_msg.header.frame_id = 'base_footprint'  # 设置坐标系

        # 设置二维点坐标
        trans = self.getPosition()

        point_stamped_msg.point.x = trans[0]
        point_stamped_msg.point.y = trans[1]
        point_stamped_msg.point.z = 0.0  # 二维点的z坐标通常设置为0
        return point_stamped_msg

    def sendGoal(self, point):
        self.goal.target_pose.pose.position.x = point[0]
        self.goal.target_pose.pose.position.y = point[1]
        self.goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(self.goal)
        self.assigned_point = array(point)
        print('goal position at: %f %f' % (point[0], point[1]))

    def cancelGoal(self):
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()

    def getState(self):
        return self.client.get_state()

    def makePlan(self, start, end):
        self.start.pose.position.x = start[0]
        self.start.pose.position.y = start[1]
        self.end.pose.position.x = end[0]
        self.end.pose.position.y = end[1]
        start = self.listener.transformPose(self.name + '/map', self.start)
        end = self.listener.transformPose(self.name + '/map', self.end)
        plan = self.make_plan(start=start, goal=end, tolerance=0.1)
        return plan.plan.poses


# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = int((floor((Xp[1] - Xstarty) / resolution) *
                 width) + (floor((Xp[0] - Xstartx) / resolution)))
    return index


def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + \
        (i / mapData.info.width) * mapData.info.resolution
    x = mapData.info.origin.position.x + \
        (float(i - (int(i / mapData.info.width) * (mapData.info.width))) * mapData.info.resolution)
    return array([x, y])


# ________________________________________________________________________________


def informationGain(mapData, point, r):
    infoGain = 0.0
    index = index_of_point(mapData, point)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if (mapData.data[i] == -1 and norm(array(point) - point_of_index(mapData, i)) <= r):
                    infoGain += 1.0
    return infoGain * (mapData.info.resolution ** 2)


# ________________________________________________________________________________


def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if (mapData.data[i] == -1 and norm(point_of_index(mapData, i) - current_pt) <= r and norm(
                            point_of_index(mapData, i) - assigned_pt) <= r):
                        # this should be modified, subtract the area of a cell, not 1
                        infoGain[j] -= 1.0
    return infoGain


# ________________________________________________________________________________


def pathCost(path):
    if (len(path) > 0):
        i = len(path) / 2
        p1 = array([path[i - 1].pose.position.x, path[i - 1].pose.position.y])
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1 - p2) * (len(path) - 1)
    else:
        return inf


# ________________________________________________________________________________


def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    r_region = 5
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if (mapData.data[i] == 1):
                    return True
    return False


# ________________________________________________________________________________


def Nearest(V, x):
    n = inf
    i = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :] - x)
        if (n1 < n):
            n = n1
            result = i
    return result


# ________________________________________________________________________________


def Nearest2(V, x):
    n = inf
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i] - x)

        if (n1 < n):
            n = n1
    return i


# ________________________________________________________________________________


def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = (floor((Xp[1] - Xstarty) / resolution) * width) + \
            (floor((Xp[0] - Xstartx) / resolution))

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100


def goalVisual(point):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = 0
    marker.ns = "marker1"
    marker.type = Marker.POINTS
    marker.action = Marker.ADD

    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.3
    marker.scale.y = 0.3

    marker.color.r = 0.0 / 255.0
    marker.color.g = 125.0 / 255.0
    marker.color.b = 255.0 / 255.0
    marker.color.a = 1

    p = Point()
    p.x = point[0]
    p.y = point[1]
    p.z = 0
    pp = [copy(p)]
    marker.points = pp
    return marker


# --------Include modules---------------
import sys
import os
# sys.path.append('../scripts')
current_dir = os.path.dirname(__file__)
sys.path.append(current_dir)
from copy import copy
import rospy
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
import tf
from mbot_explore.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
# from functions import robot, informationGain, discount, goalVisual
from numpy.linalg import norm
import math

# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
globalmaps = []


def callBack(data):
    global frontiers
    frontiers = []
    for point in data.points:
        frontiers.append(array([point.x, point.y]))


def mapCallBack(data):
    global mapData
    mapData = data


# Node----------------------------------------------

def node():
    global frontiers, mapData, globalmaps
    rospy.init_node('assigner', anonymous=False)

    # fetching all parameters
    map_topic = rospy.get_param('~map_topic', '/map')
    info_radius = rospy.get_param('~info_radius',
                                  1.0)  # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_multiplier = rospy.get_param('~info_multiplier', 3.0)
    hysteresis_radius = rospy.get_param('~hysteresis_radius', 3.0)  # at least as much as the laser scanner range
    hysteresis_gain = rospy.get_param('~hysteresis_gain',
                                      2.0)  # bigger than 1 (biase robot to continue exploring current region
    frontiers_topic = rospy.get_param('~frontiers_topic', '/filtered_points')
    delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.5)
    rateHz = rospy.get_param('~rate', 100)
    robot_namelist = rospy.get_param('~robot_namelist', "")

    rate = rospy.Rate(rateHz)
    # -------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, callBack)
    # ---------------------------------------------------------------------------------------------------------------
    # Publish the goal for visual
    goal_publisher = rospy.Publisher('goal_visual', Marker, queue_size=10)
    position_publisher = rospy.Publisher('robot_position', PointStamped, queue_size=10)
    # ---------------------------------------------------------------------------------------------------------------

    # perform name splitting for the robot
    robot_namelist = robot_namelist.split(',')
    # wait if no frontier is received yet
    while len(frontiers) < 1:
        pass
    centroids = copy(frontiers)
    # wait if map is not received yet
    while (len(mapData.data) < 1):
        pass

    robots = []
    for i in range(0, len(robot_namelist)):
        robots.append(robot(name=robot_namelist[i]))

    for i in range(0, len(robot_namelist)):
        robots[i].sendGoal(robots[i].getPosition())
    # -------------------------------------------------------------------------
    # ---------------------     Main   Loop     -------------------------------
    # -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        centroids = copy(frontiers)
        # -------------------------------------------------------------------------
        # Get information gain for each frontier point
        infoGain = []
        for ip in range(0, len(centroids)):
            infoGain.append(informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius))
        # -------------------------------------------------------------------------
        # get number of available/busy robots
        na = []  # available robots
        nb = []  # busy robots
        for i in range(0, len(robot_namelist)):
            if (robots[i].getState() == 1):
                nb.append(i)
            else:
                na.append(i)
        # rospy.loginfo("available robots: " + str(na))
        # -------------------------------------------------------------------------
        # get dicount and update informationGain
        for i in nb + na:
            infoGain = discount(mapData, robots[i].assigned_point, centroids, infoGain, info_radius)
        # -------------------------------------------------------------------------
        revenue_record = []
        centroid_record = []
        id_record = []

        for ir in na:
            for ip in range(0, len(centroids)):
                cost = norm(robots[ir].getPosition() - centroids[ip])
                threshold = 1
                information_gain = infoGain[ip]
                if (norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius):
                    information_gain *= hysteresis_gain
                revenue = information_gain * info_multiplier - cost
                revenue_record.append(revenue)
                centroid_record.append(centroids[ip])
                id_record.append(ir)

        if len(na) < 1:
            revenue_record = []
            centroid_record = []
            id_record = []
            for ir in nb:
                for ip in range(0, len(centroids)):
                    cost = norm(robots[ir].getPosition() - centroids[ip])
                    threshold = 1
                    information_gain = infoGain[ip]
                    if (norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius):
                        information_gain *= hysteresis_gain

                    if ((norm(centroids[ip] - robots[ir].assigned_point)) < hysteresis_radius):
                        information_gain = informationGain(mapData, [centroids[ip][0], centroids[ip][1]],
                                                           info_radius) * hysteresis_gain

                    revenue = information_gain * info_multiplier - cost
                    revenue_record.append(revenue)
                    centroid_record.append(centroids[ip])
                    id_record.append(ir)

        # rospy.loginfo("revenue record: " + str(revenue_record))
        # rospy.loginfo("centroid record: " + str(centroid_record))
        # rospy.loginfo("robot IDs record: "+str(id_record))

        # -------------------------------------------------------------------------
        if (len(id_record) > 0):
            winner_id = revenue_record.index(max(revenue_record))

            goal = goalVisual(centroid_record[winner_id])
            goal.header.stamp = rospy.Time.now()
            goal.lifetime = rospy.Duration()
            goal_publisher.publish(goal)

            # robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
            # print(centroid_record[winner_id])
            rospy.loginfo( robot_namelist[id_record[winner_id]] + "  assigned to  " + str(centroid_record[winner_id]))
            rospy.sleep(delay_after_assignement)

        # -------------------------------------------------------------------------
        rate.sleep()


# -------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass