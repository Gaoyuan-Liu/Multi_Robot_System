'''
Fuzzy sets
'''
import numpy as np



class fuzzy_sets_class():

    def __init__(self):
        self.lambdas = [0.05, 0.1, 0.2]
    
    def single_variable_membership_functions(self, state, center):
        M = np.arange(7, dtype=np.float64)
        lambda_1 = self.lambdas[0]
        lambda_2 = self.lambdas[1]
        lambda_3 = self.lambdas[2]
        # M0
        if state < center-lambda_1-lambda_2-lambda_3:
            M[0] = 1
        elif state < center-lambda_1-lambda_2:
            M[0] = -(1/lambda_3)*state + (1/lambda_3)*(center-lambda_1-lambda_2)
        else:
            M[0] = 0
        
        # M1
        if state > center-lambda_1-lambda_2-lambda_3 and state < center-lambda_1-lambda_2:
            M[1] = (1/lambda_3)*state - (1/lambda_3)*(center-lambda_1-lambda_2-lambda_3)
        elif state >= center-lambda_1-lambda_2 and state < center-lambda_1:
            M[1] = -(1/lambda_2)*state + (1/lambda_2)*(center-lambda_1)
        else:
            M[1] = 0

        # M2
        if state > center-lambda_1-lambda_2 and state < center-lambda_1:
            M[2] = (1/lambda_2)*state - (1/lambda_2)*(center-lambda_1-lambda_2)
        elif state >= center-lambda_1 and state < center:
            M[2] = -(1/lambda_1)*state + (1/lambda_1)*(center)
        else:
            M[2] = 0

        # M3
        if state > center-lambda_1 and state < center:
            M[3] = (1/lambda_1)*state - (1/lambda_1)*(center-lambda_1)
        elif state >= center and state < center+lambda_1:
            M[3] = -(1/lambda_1)*state + (1/lambda_1)*(center+lambda_1)
        else:
            M[3] = 0

        # M4
        if state > center and state < center+lambda_1:
            M[4] = (1/lambda_1)*state - (1/lambda_1)*center
        elif state >= center+lambda_1 and state < center + lambda_1 + lambda_2:
            M[4] = -(1/lambda_2)*state + (1/lambda_2)*(center + lambda_1 + lambda_2)
        else:
            M[4] = 0

        # M5
        if state > center+lambda_1 and state < center + lambda_1 + lambda_2:
            M[5] = (1/lambda_2)*state - (1/lambda_2)*(center+lambda_1)
        elif state >= center + lambda_1 + lambda_2 and state < center + lambda_1 + lambda_2 + lambda_3:
            M[5] = -(1/lambda_3)*state + (1/lambda_3)*(center + lambda_1 + lambda_2 + lambda_3)
        else:
            M[5] = 0

        # M6
        if state > center + lambda_1 + lambda_2 and state < center + lambda_1 + lambda_2 + lambda_3:
            M[6] = (1/lambda_3)*state - (1/lambda_3)*(center + lambda_1 + lambda_2)
        elif state >= center + lambda_1 + lambda_2 + lambda_3:
            M[6] = 1
        else:
            M[6] = 0

        return M

    def membership_functions_3D(self, state_point, center_point):
        #MF_3D = np.zeros(343, dtype=np.float64)
        
        MF_3D = [0]*343
        MF_x = self.single_variable_membership_functions(state_point[0], center_point[0])
        MF_y = self.single_variable_membership_functions(state_point[1], center_point[1])
        MF_z = self.single_variable_membership_functions(state_point[2], center_point[2])
        for i in range(7):
            for j in range(7):
                 for k in range(7):
                     MF_3D[49*i+7*j+k] = MF_x[j] * MF_y[k] * MF_z[i]
  
        return MF_3D
        

