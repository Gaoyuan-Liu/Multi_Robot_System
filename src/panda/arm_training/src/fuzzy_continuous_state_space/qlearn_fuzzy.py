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
import fuzzy_sets
from operator import add

class QLearn:
    def __init__(self, actions, epsilon, alpha, gamma, temperature):
        self.q = {}
        self.q_fuzzy = {}
        self.epsilon = epsilon  # exploration constant
        self.alpha = alpha      # discount constant
        self.gamma = gamma      # discount factor
        self.actions = actions
        self.qfuzzytable = {}
        self.temperature = temperature
        self.fuzzy_center = [0.25, 0.45, 0.71]
        self.fuzzy_sets_size = 343


    def getQfuzzy(self, fuzzy_set_number, action):
        return self.q_fuzzy.get((fuzzy_set_number, action), 0.0)


############################################################################
    
    # Learn single Q value
    def learnQfuzzy(self, fuzzy_set_number, actual_state, action, reward, value):
        fs = fuzzy_sets.fuzzy_sets_class()
        fs_value = fs.membership_functions_3D(actual_state, self.fuzzy_center)
        oldvfuzzy = self.q_fuzzy.get((fuzzy_set_number, action), None)
        if oldvfuzzy is None:
            self.q_fuzzy[(fuzzy_set_number, action)] = fs_value[fuzzy_set_number]*reward
        else:
            #self.q_fuzzy[(fuzzy_set_number, action)] = fs_value[fuzzy_set_number]*(oldvfuzzy + self.alpha * (value - oldvfuzzy))
            self.q_fuzzy[(fuzzy_set_number, action)] = oldvfuzzy + fs_value[fuzzy_set_number] * self.alpha * (value - oldvfuzzy)
        self.qfuzzytable[(fuzzy_set_number, action)] = self.q_fuzzy[(fuzzy_set_number, action)]



##################################################################################

    def learn_fuzzy(self, actual_state1, action1, reward, actual_state2):
        fs = fuzzy_sets.fuzzy_sets_class()
        fs_value = fs.membership_functions_3D(actual_state2, self.fuzzy_center)
        q_iu_sum = [0]*len(self.actions) #7
        #maxqnew = max([self.getQfuzzy(i, a) for a in self.actions])
        for i in range(self.fuzzy_sets_size):
            q_iu = [self.getQfuzzy(i, a) for a in self.actions]
            q_iu_sum_raw = [fs_value[i]*j for j in q_iu]
            q_iu_sum = list(map(add, q_iu_sum, q_iu_sum_raw))
        maxqnew = max(q_iu_sum)
        for h in range(self.fuzzy_sets_size):
            self.learnQfuzzy(h, actual_state1, action1, reward, reward + self.gamma*maxqnew)
            

###################################################################################

    def choosefuzzyAction(self, actual_state, return_q=False):
        fs = fuzzy_sets.fuzzy_sets_class()
        fs_value = fs.membership_functions_3D(actual_state, self.fuzzy_center)
        #q_fuzzy = [self.getQfuzzy(fuzzy_set_number, a) for a in self.actions]
        
        weighted_q_fuzzy = [0]*len(self.actions)
        for i in range(self.fuzzy_sets_size): #343
            q_fuzzy = [self.getQfuzzy(i, a) for a in self.actions]
            weighted_q_fuzzy_raw = [fs_value[i]*j for j in q_fuzzy]
            weighted_q_fuzzy = list(map(add, weighted_q_fuzzy, weighted_q_fuzzy_raw))
        maxQ = max(weighted_q_fuzzy)
        #rospy.loginfo ("length of weighted_q_fuzzy is " + str(len(weighted_q_fuzzy)))
        ####################################
        # Epsilon Greedy
        """if random.random() < self.epsilon and self.temperature > 0.12:
            action = random.randrange(0,7,1)
        else:
            count = weighted_q_fuzzy.count(maxQ)
            if count > 1:
                best = [i_best for i_best in range(len(self.actions)) if weighted_q_fuzzy[i_best] == maxQ]
                i_best = random.choice(best)
            else:
                i_best = weighted_q_fuzzy.index(maxQ)
        #rospy.loginfo ("index i is " + str(i))
            action = self.actions[i_best]"""

        ####################################
        # Boltzmann Exploration: 

        if self.temperature > 0.2:
            action_probs_numes = []
            denom = 0 # The denominator is a scalar
            #for m in q:
            for m in weighted_q_fuzzy:
                val = math.exp(m / self.temperature)
                action_probs_numes.append(val)
                denom += val
            action_probs = [x / denom for x in action_probs_numes]
            # Pick move according to action_probs
            rand_val = random.uniform(0, 1)
            prob_sum = 0
            #rospy.loginfo ("length of action_probs is " + str(len(action_probs)))

            for i, prob in enumerate(action_probs):
                prob_sum += prob
                if rand_val <= prob_sum:
                    action = self.actions[i]
                    break
        else: 
            # rospy.loginfo("Greedy exploration now.")      
            #count = q.count(maxQ)
            count = weighted_q_fuzzy.count(maxQ)
            # In case there're several state-action max values 
            # we select a random one among them
            if count > 1:
                best = [j for j in range(len(self.actions)) if weighted_q_fuzzy[j] == maxQ]
                j = random.choice(best)
            else:
                j = weighted_q_fuzzy.index(maxQ)
            action = self.actions[j]

        if return_q: # if they want it, give it!
            return action, weighted_q_fuzzy
        return action




