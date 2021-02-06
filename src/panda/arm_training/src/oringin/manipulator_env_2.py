#!/usr/bin/env python
import gym
import rospy
import time
import numpy as np
#import tf

from gym import utils, spaces
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

#register the training environment in the gym as an available one
reg = register(
    id='ManipulatorRLControl-v1',
    entry_point='manipulator_env_2:ManipulatorEnv',
    max_episode_steps=30,
    )

class ManipulatorEnv(gym.Env):

    def __init__(self):
        self.joint_number = 7
        for i in range(self.joint_number):
            self.position_pub = rospy.Publisher('/panda/endeffector_command', Float64MultiArray, queue_size=100)

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()
        
        self.action_space = spaces.Discrete(7) #Forward,Backward,Left,Right,Up,Down,Still
        self.reward_range = (-np.inf, np.inf)
        self.desired_pose = [0.25, 0.45, 0.71]
        self.current_pose = [0, 0, 0]
        self.current_command = [0.15, 0.05, 0.75]
        self.blacklist = []

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]


    def reset(self):
        
        # 1st: resets the simulation to initial values
        #self.gazebo.resetSim()
        #rospy.loginfo ("get here lol")
        # 2nd: Unpauses simulation
        self.gazebo.unpauseSim()

        # 3rd: resets the robot to initial conditions
        self.check_topic_publishers_connection()
        self.init_desired_pose()
        
        self.current_command = [0.15, 0.05, 0.75]
        # 4th: takes an observation of the initial condition of the robot
        ee_pose = self.take_observation()
        observation = [self.calculate_state_number(ee_pose[0], ee_pose[1], ee_pose[2])]
        
        # 5th: pauses simulation
        self.gazebo.pauseSim()

        return observation


    def step(self, action):

        endeffector_cmd = Float64MultiArray()
        endeffector_cmd.data = [0, 0 ,0]

        if action == 0:
            endeffector_cmd.data[0] = self.current_command[0] + 0.1
            endeffector_cmd.data[1] = self.current_command[1]
            endeffector_cmd.data[2] = self.current_command[2]
        if action == 1:
            endeffector_cmd.data[0] = self.current_command[0] - 0.1
            endeffector_cmd.data[1] = self.current_command[1]
            endeffector_cmd.data[2] = self.current_command[2]
        if action == 2:
            endeffector_cmd.data[0] = self.current_command[0]
            endeffector_cmd.data[1] = self.current_command[1] + 0.1
            endeffector_cmd.data[2] = self.current_command[2]
        if action == 3:
            endeffector_cmd.data[0] = self.current_command[0]
            endeffector_cmd.data[1] = self.current_command[1] - 0.1
            endeffector_cmd.data[2] = self.current_command[2]
        if action == 4:
            endeffector_cmd.data[0] = self.current_command[0]
            endeffector_cmd.data[1] = self.current_command[1]
            endeffector_cmd.data[2] = self.current_command[2] + 0.1
        if action == 5:
            endeffector_cmd.data[0] = self.current_command[0]
            endeffector_cmd.data[1] = self.current_command[1]
            endeffector_cmd.data[2] = self.current_command[2] - 0.1
        if action == 6:
            endeffector_cmd.data[0] = self.current_command[0]
            endeffector_cmd.data[1] = self.current_command[1]
            endeffector_cmd.data[2] = self.current_command[2]
        #feasible = self.is_command_feasible(endeffector_cmd)
        command_state_raw = self.calculate_state_number(endeffector_cmd.data[0], endeffector_cmd.data[1], endeffector_cmd.data[2])
        # Check blacklist
        if command_state_raw in self.blacklist:
            endeffector_cmd.data[0] = self.current_command[0]
            endeffector_cmd.data[1] = self.current_command[1]
            endeffector_cmd.data[2] = self.current_command[2]

        self.gazebo.unpauseSim()
        # The first time to publishe command in this step
        self.position_pub.publish(endeffector_cmd)
        # Check the is_feasible_flag
        is_feasible_flag = self.take_feasible_flag()
        

        if is_feasible_flag == False:
            # Add the state number to blacklist
            bad_state = self.calculate_state_number(endeffector_cmd.data[0], endeffector_cmd.data[1], endeffector_cmd.data[2])
            self.blacklist.append(bad_state)
            #self.gazebo.pauseSim()
            #state = [bad_state]
            #reward = -10
            #done = True

            endeffector_cmd.data[0] = self.current_command[0];
            endeffector_cmd.data[1] = self.current_command[1];
            endeffector_cmd.data[2] = self.current_command[2];
            self.position_pub.publish(endeffector_cmd)
        #else:
        ee_pose = self.take_observation()
        self.current_pose = ee_pose
        self.current_command = endeffector_cmd.data

        # Wait for the robot to actually go to the commanded position 
        rate = rospy.Rate(100)
        i = 0
        while not self.calculate_state_number(self.current_pose[0], self.current_pose[1], self.current_pose[2]) == self.calculate_state_number(self.current_command[0], self.current_command[1], self.current_command[2]):
            i = i+1
            self.position_pub.publish(endeffector_cmd)
            ee_pose = self.take_observation()
            self.current_pose = ee_pose

            """current_state_number = self.calculate_state_number(self.current_pose[0], self.current_pose[1], self.current_pose[2])
            command_state_number = self.calculate_state_number(self.current_command[0], self.current_command[1], self.current_command[2])
            desired_state_number = self.calculate_state_number(self.desired_pose[0], self.desired_pose[1], self.desired_pose[2])
            rospy.loginfo ("current_state_number is " + str(current_state_number))
            rospy.loginfo ("command_state_number is " + str(command_state_number))
            rospy.loginfo ("desired_state_number is " + str(desired_state_number))"""
            if i >= 1000:
                rospy.loginfo("Go-to loop timeout")
                break
            rate.sleep()

        
        self.gazebo.pauseSim()
        reward, done = self.process_data(self.current_pose, self.desired_pose)
        state = [self.calculate_state_number(ee_pose[0], ee_pose[1], ee_pose[2])]

        return state, reward, done, {}


    



############# Functions do dirty work ####################
    def check_topic_publishers_connection(self):
        
        rate = rospy.Rate(100) # 100hz

        while(self.position_pub.get_num_connections() == 0):
            rospy.loginfo("No susbribers to Cmd_position yet so we wait and try again")
            rate.sleep();
        rospy.loginfo("Cmd_position Publisher Connected")


    def take_observation (self):
        data_pose = None
        while data_pose is None:
            try:
                data_pose_raw = Float64MultiArray()
                data_pose_raw = rospy.wait_for_message('/panda/endeffector_state', Float64MultiArray, timeout=100)
                data_pose = data_pose_raw.data
            except:
                rospy.loginfo("Current arm pose not ready yet, retrying for getting robot pose")
        return data_pose

    def take_feasible_flag (self):
        is_feasible_flag = None
        while is_feasible_flag is None:
            try:
                # rospy.loginfo ("stuck here")
                is_feasible_flag_raw = Bool()
                is_feasible_flag_raw = rospy.wait_for_message('/panda/is_feasible', Bool, timeout=100)
                is_feasible_flag = is_feasible_flag_raw.data
            except:
                rospy.loginfo("Current is_feasible_flag is not false")
        return is_feasible_flag

    def init_desired_pose(self):
        current_init_pose = self.take_observation()
        self.best_dist = self.calculate_dist_between_two_Points(current_init_pose, self.desired_pose)

    # Calculate rewards
    def process_data(self, current_pose_, desired_pose_):
        done = False
        current_state = self.calculate_state_number(current_pose_[0], current_pose_[1], current_pose_[2])
        desired_state = self.calculate_state_number(desired_pose_[0], desired_pose_[1], desired_pose_[2])
        current_dist = self.calculate_dist_between_two_Points(current_pose_, desired_pose_)
        if current_dist < self.best_dist:
            reward = 1
            self.best_dist = current_dist
        elif current_dist == self.best_dist:
            reward = -0.5
        else:
            reward = -1

        if current_state == desired_state:
            reward = 50
            done = True
            rospy.loginfo("ARRIVED AHA!")
        
        return reward, done

    def calculate_dist_between_two_Points(self,p_init,p_end):
        a = np.array((p_init[0] ,p_init[1], p_init[2]))
        b = np.array((p_end[0] ,p_end[1], p_end[2]))
        
        dist = np.linalg.norm(a-b)
        
        return dist

    def calculate_state_number (self, x, y, z):
        if x < -0.5:
            x = -0.499
        if x > 0.5:
            x = 0.499
        if y < -0.5:
            y = -0.499
        if y > 0.5:
            y = 0.499
        if z < 0:#0.1824 when drone stay ground
            z = 0
        if z > 1:
            z = 0.999
        for i in range(10):
            if x >= -0.5+0.1*i and x < -0.5+0.1*i+0.1:
                numberx = 10 * i
        for j in range(10):
            if y >= -0.5+0.1*j and y < -0.5+0.1*j+0.1:
                numbery = j
        for k in range(10):
            if z >= 0.1*k and z < 0.1*k+0.1:
                numberz = 100 * k
        state_number = numberx + numbery + numberz
        state = state_number
        return state

    def is_command_feasible (self, endeffector_cmd_):
        feasible = False
        x = endeffector_cmd_.data[0]
        y = endeffector_cmd_.data[1]
        z = endeffector_cmd_.data[2]
        if x >= 0 and x <= 0.8 and y >= -0.8 and y <= 0.8 and z >= 0.1 and z <= 1.1 and x**2 + y**2 + (z-0.335)**2 <= 0.64:
            feasible = True
        return feasible












