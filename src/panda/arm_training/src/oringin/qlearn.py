'''
Q-learning approach for different RL problems
as part of the basic series on reinforcement learning @
https://github.com/vmayoral/basic_reinforcement_learning
 
Inspired by https://gym.openai.com/evaluations/eval_kWknKOkPQ7izrixdhriurA
 
        @author: Victor Mayoral Vilches <victor@erlerobotics.com>
'''

import random
import math
import rospy

class QLearn:
    def __init__(self, actions, epsilon, alpha, gamma, temperature):
        self.q = {}
        self.epsilon = epsilon  # exploration constant
        self.alpha = alpha      # discount constant
        self.gamma = gamma      # discount factor
        self.actions = actions
        self.qtable = {}
        self.temperature = temperature

    def getQ(self, state, action):
        return self.q.get((state, action), 0.0) #0.0 is default value

    def learnQ(self, state, action, reward, value):
        '''
        Q-learning:
            Q(s, a) += alpha * (reward(s,a) + max(Q(s') - Q(s,a))            
        '''
        oldv = self.q.get((state, action), None)
        if oldv is None:
            self.q[(state, action)] = reward
        else:
            self.q[(state, action)] = oldv + self.alpha * (value - oldv)
        
        self.qtable[(state, action)] = self.q[(state, action)]

    def chooseAction(self, state, return_q=False):
        q = [self.getQ(state, a) for a in self.actions]
        maxQ = max(q)
        ####################################
        # Boltzmann Exploration: 

        if self.temperature > 0.5:
            action_probs_numes = []
            denom = 0 # The denominator is a scalar
            for m in q:
                val = math.exp(m / self.temperature)
                action_probs_numes.append(val)
                denom += val
            action_probs = [x / denom for x in action_probs_numes]
            # Pick move according to action_probs
            rand_val = random.uniform(0, 1)
            prob_sum = 0
            for i, prob in enumerate(action_probs):
                prob_sum += prob
                if rand_val <= prob_sum:
                    action = self.actions[i]
                    break
        else: 
            rospy.loginfo("Greedy exploration now.")      
            count = q.count(maxQ)
            # In case there're several state-action max values 
            # we select a random one among them
            if count > 1:
                best = [j for j in range(len(self.actions)) if q[j] == maxQ]
                j = random.choice(best)
            else:
                j = q.index(maxQ)
            action = self.actions[j] 


      
        if return_q: # if they want it, give it!
            return action, q
        return action

        #####################################


    def learn(self, state1, action1, reward, state2):
        maxqnew = max([self.getQ(state2, a) for a in self.actions])
        self.learnQ(state1, action1, reward, reward + self.gamma*maxqnew)
