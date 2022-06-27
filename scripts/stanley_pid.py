#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from sklearn.utils import check_X_y
import rospy
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from carla_msgs.msg import CarlaEgoVehicleControl
import math


class State():
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.WB = 2.9

class Stanley():
    def __init__(self):
        self.loadParameters()
        self.waypoints_list = []
        self.k = 0.015
        self.Kp = 1.0
        self.L = 2.8
        self.max_steer = 0.523599
        # self.state = State()
        self.last_used_idx = 0
        self.i = 0
        self.totalWaypoints = 0
        self.state = None

        self.show_animation = True

        self.x = []
        self.y = []
        self.yaw = []
        self.v = []

    def pid_control(self, target, current):
        """
        Proportional control for the speed.

        :param target: (float)
        :param current: (float)
        :return: (float)
        """
        return self.Kp * (target - current)

    def stanley_control(self, state, cx, cy, cyaw, last_target_idx):
        """
        Stanley steering control.

        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle, d_theta = self.calc_target_index(state, cx, cy)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = self.normalize_angle(d_theta - state.yaw)
        # theta_d corrects the cross track error
        # print(state.v,"CUrrent vel")
        theta_d = np.arctan2(self.k * error_front_axle, state.v)
        print(theta_e, theta_d)
        print("Heading error, crosstrack error")
        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].

        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def calc_target_index(self, state, cx, cy):
        """
        Compute index in the trajectory list of the target.

        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = state.x + self.L * np.cos(state.yaw)
        fy = state.y + self.L * np.sin(state.yaw)

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        # dx2 = list(map(lambda n: n**2, dx))
        # dy2 = list(map(lambda n: n**2, dy))


        d = np.hypot(dx, dy)
        
        target_idx = np.argmin(d)
        # i = target_idx
        # while d[i] < 7:
        #     i+=1
            
    
        x1 = state.x
        x2 = cx[target_idx + 5]
        y1 = state.y
        y2 = cy[target_idx + 5]

        a = y1 -y2
        b = x2 - x1
        c = x1*(y2 - y1) - y1*(x2-x1)
        # c = 0.5
        if a == 0 or b ==0:
            d_theta = 0
        else:
            d_theta = np.arctan(-a/b)
        error_front_axle = a*fx + b*fy + c / math.sqrt(a**2 + b**2)

        # print("Target IDX: ", target_idx)
        # print(np.min(d))


        # Project RMS error onto front axle vector
        # front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
        #                 -np.sin(state.yaw + np.pi / 2)]
        # print(front_axle_vec, "Front axle vel")                
        # error_front_axle = np.dot([abs(dx[target_idx]), abs(dy[target_idx])], front_axle_vec)
        # print(error_front_axle, "Error front")
        return target_idx + 5, error_front_axle, d_theta

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber("/carla/ego_vehicle/waypoints", Path, buff_size=2**24, queue_size=1, callback=self.wayPointsCallback)
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, callback=self.odometryCallback)
        rospy.Subscriber("/carla/ego_vehicle/speedometer", Float32, callback=self.velocityCallback)

    def loadParameters(self):
        '''
        add data
        '''
        # add something

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.controlcommandPub = rospy.Publisher(
            "/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=1)

    def wayPointsCallback(self, wps):
        self.waypoints_list = wps.poses
        self.cx = []
        self.cy = []
        self.cyaw = []
        for obj in self.waypoints_list:
            self.cx.append(obj.pose.position.x)
            self.cy.append(obj.pose.position.y)
            self.cyaw.append(obj.pose.orientation.z)
        self.totalWaypoints = len(self.cx)

    def odometryCallback(self, data):
        self.state = State(x=data.pose.pose.position.x, y=data.pose.pose.position.y, yaw=euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2])

    def velocityCallback(self, data):
        if self.state != None:
            self.state.v = data.data

    def callController(self):
        target_speed = 30.0 / 3.6  # [m/s]
        last_idx = len(self.cx) -1
        target_idx, _, _ = self.calc_target_index(self.state, self.cx, self.cy)
        if last_idx > target_idx:
            acc = self.pid_control(target_speed, self.state.v)
            steer, target_idx = self.stanley_control(self.state, self.cx, self.cy, self.cyaw, target_idx)
            print("initial steer", steer)
            steer = np.clip(steer, -self.max_steer, self.max_steer)
            steer = ((steer - (-self.max_steer)) / (self.max_steer - (-self.max_steer))) * (1 - (-1)) + (-1)
            self.x.append(self.state.x)
            self.y.append(self.state.y)
            self.yaw.append(self.state.yaw)
            self.v.append(self.state.v)

            control_command = CarlaEgoVehicleControl()
            # control_command.header.stamp = rospy.get_time()
            control_command.steer = steer+-1
            if (acc < 0):
                control_command.throttle = 0
                control_command.brake = -acc
            elif (acc >= 1):
                acc = 1
                control_command.throttle = acc
                control_command.brake = 0

            print(acc, steer)
            print("Throttle, steer")

            control_command.hand_brake = False
            control_command.reverse = False
            control_command.manual_gear_shift = False

            self.controlcommandPub.publish(control_command)

            if self.show_animation:
                plt.cla()
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(self.cx, self.cy, ".r", label="course")
                plt.plot(self.x, self.y, "-g", label="trajectory")
                plt.plot(self.cx[target_idx], self.cy[target_idx], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(self.state.v * 3.6)[:4])
                plt.pause(0.001)




        