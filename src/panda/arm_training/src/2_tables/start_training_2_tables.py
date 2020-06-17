#!/usr/bin/env python

'''
    Training code made by Ricardo Tellez <rtellez@theconstructsim.com>
    Based on many other examples around Internet
    Visit our website at www.theconstruct.ai
'''
import gym
import time
import numpy
import random
import time
import qlearn_2_tables
import pickle
from gym import wrappers

# ROS packages required
import rospy
import rospkg

# import our training environment
import manipulator_env_2_tables

if __name__ == '__main__':
    
    rospy.init_node('arm_gym', anonymous=True)

    # Create the Gym environment
    env = gym.make('ManipulatorRLControl-v4')
    rospy.loginfo ( "Gym environment done")
    

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('arm_training')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True) 
    rospy.loginfo ( "Monitor Wrapper started")

    last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    Alpha = rospy.get_param("/alpha")
    Epsilon = rospy.get_param("/epsilon")
    Gamma = rospy.get_param("/gamma")
    epsilon_discount = rospy.get_param("/epsilon_discount")
    nepisodes = rospy.get_param("/nepisodes")
    nsteps = rospy.get_param("/nsteps")
    Temperature = rospy.get_param("/temperature")

    # Initialises the algorithm that we are going to use for learning
    qlearn = qlearn_2_tables.QLearn(actions=range(env.action_space.n),
                    alpha=Alpha, gamma=Gamma, epsilon=Epsilon, temperature=Temperature)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0

    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):
        rospy.loginfo ("STARTING Episode #"+str(x))
        
        cumulated_reward = 0  
        done = False

        #if qlearn.epsilon > 0.05:
        #    qlearn.epsilon *= epsilon_discount
        qlearn.temperature *= 0.9
        
        # Initialize the environment and get first state of the robot
        
        observation = env.reset()

        state = ''.join(map(str, observation))

        actual_state = env.get_actual_state()
        # Show on screen the actual situation of the robot
        #env.render()
        
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):

            # Pick an action based on the current state
            #rospy.loginfo ("get here")
            #action = qlearn.chooseAction(state)

            actual_state = env.get_actual_state()
            action = qlearn.choosefuzzyAction(actual_state)

            # Execute the action in the environment and get feedback
            observation, reward, done, info = env.step(action)

            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation)) # Make state a string
            next_actual_state = observation
            # Make the algorithm learn based on the results
            qlearn.learn(state, action, reward, nextState)
            qlearn.learn_fuzzy(state, actual_state, action, reward, nextState, next_actual_state)

            rospy.loginfo ("step number is " + str(i))
            rospy.loginfo ("step reward is " + str(reward))
            #rospy.loginfo ("choose action " + str(action))
            #rospy.loginfo ("current state is " + str(observation))
            if not(done):
                state = nextState
                actual_state = next_actual_state
            else:
                rospy.loginfo ("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        rospy.loginfo ( ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))

    
    rospy.loginfo ( ("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    #print("Parameters: a="+str)
    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    #rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))    
    # Save the final qtable
    #with open('/home/liu/CCR/ros_ws/src/arm_training/training_results/final_Q_table.pkl', 'wb') as f:
        #pickle.dump(qlearn.qfuzzytable, f, pickle.HIGHEST_PROTOCOL)
    
    #rospy.loginfo("type of q is"+str(type(qlearn.qtable)))
    env.close()
