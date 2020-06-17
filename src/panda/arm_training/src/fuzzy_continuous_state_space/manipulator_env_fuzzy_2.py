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
    id='ManipulatorRLControl-v3',
    entry_point='manipulator_env_fuzzy_2:ManipulatorEnv',
    max_episode_steps=50,
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
        
        
        self.current_command = [0.15, 0.05, 0.75]
        self.init_desired_pose()
        # 4th: takes an observation of the initial condition of the robot
        ee_pose = self.take_observation()
        observation = ee_pose       

        # 5th: pauses simulation
        self.gazebo.pauseSim()

        return observation


    def step(self, action):

        endeffector_cmd = Float64MultiArray()
        endeffector_cmd.data = [0, 0 ,0]

        if action == 0:
            endeffector_cmd.data[0] = self.current_command[0] + 0.05;
            endeffector_cmd.data[1] = self.current_command[1];
            endeffector_cmd.data[2] = self.current_command[2];
        if action == 1:
            endeffector_cmd.data[0] = self.current_command[0] - 0.05;
            endeffector_cmd.data[1] = self.current_command[1];
            endeffector_cmd.data[2] = self.current_command[2];
        if action == 2:
            endeffector_cmd.data[0] = self.current_command[0];
            endeffector_cmd.data[1] = self.current_command[1] + 0.05;
            endeffector_cmd.data[2] = self.current_command[2];
        if action == 3:
            endeffector_cmd.data[0] = self.current_command[0];
            endeffector_cmd.data[1] = self.current_command[1] - 0.05;
            endeffector_cmd.data[2] = self.current_command[2];
        if action == 4:
            endeffector_cmd.data[0] = self.current_command[0];
            endeffector_cmd.data[1] = self.current_command[1];
            endeffector_cmd.data[2] = self.current_command[2] + 0.05;
        if action == 5:
            endeffector_cmd.data[0] = self.current_command[0];
            endeffector_cmd.data[1] = self.current_command[1];
            endeffector_cmd.data[2] = self.current_command[2] - 0.05;
        if action == 6:
            endeffector_cmd.data[0] = self.current_command[0];
            endeffector_cmd.data[1] = self.current_command[1];
            endeffector_cmd.data[2] = self.current_command[2];

        self.gazebo.unpauseSim()
        # The first time to publishe command in this step
        self.position_pub.publish(endeffector_cmd)

        # Check the is_feasible_flag
        is_feasible_flag = self.take_feasible_flag()
        if is_feasible_flag == False:
            endeffector_cmd.data[0] = self.current_command[0];
            endeffector_cmd.data[1] = self.current_command[1];
            endeffector_cmd.data[2] = self.current_command[2];
            self.position_pub.publish(endeffector_cmd)
        
        ee_pose = self.take_observation()
        self.current_pose = ee_pose
        self.current_command = endeffector_cmd.data

        # Wait for the robot to actually go to the commanded position 
        #cannot_go = False
        rate = rospy.Rate(100)
        i = 0
        # Go-to loop
        while self.calculate_dist_between_two_Points(self.current_pose, self.current_command) > 0.05:
            i = i+1
            self.position_pub.publish(endeffector_cmd)
            ee_pose = self.take_observation()
            self.current_pose = ee_pose

            if i >= 1000:
                rospy.loginfo("Go-to loop timeout")
                #cannot_go = True
                break
            rate.sleep()

        
        self.gazebo.pauseSim()
        reward, done = self.process_data(self.current_pose, self.desired_pose)
        state = ee_pose

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
        #current_init_pose = self.take_observation()
        self.best_dist = self.calculate_dist_between_two_Points(self.current_command, self.desired_pose)

    # Calculate rewards
    def process_data(self, current_pose_, desired_pose_):
        done = False
        current_dist = self.calculate_dist_between_two_Points(current_pose_, desired_pose_)
        if current_dist < self.best_dist:
            reward = 1
            self.best_dist = current_dist
        elif current_dist == self.best_dist:
            reward = -0.5
        else:
            reward = -1

        if current_dist < 0.1:
            reward = 50
            done = True
            rospy.loginfo("ARRIVED AHA!")
        
        return reward, done

    def calculate_dist_between_two_Points(self,p_init,p_end):
        a = np.array((p_init[0] ,p_init[1], p_init[2]))
        b = np.array((p_end[0] ,p_end[1], p_end[2]))
        
        dist = np.linalg.norm(a-b)
        
        return dist

    def is_command_feasible(self, endeffector_cmd_):
        feasible = False
        x = endeffector_cmd_.data[0]
        y = endeffector_cmd_.data[1]
        z = endeffector_cmd_.data[2]
        if x >= 0 and x <= 0.8 and y >= -0.8 and y <= 0.8 and z >= 0.1 and z <= 1.1 and x**2 + y**2 + (z-0.335)**2 <= 0.64:
            feasible = True
        return feasible


    def near_enough(self, position1, position2):
        calculate_dist_between_two_Points





