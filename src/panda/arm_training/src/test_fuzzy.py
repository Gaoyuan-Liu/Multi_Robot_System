#!/usr/bin/env python


import fuzzy_sets

if __name__ =='__main__':

    state_point = [0.03, 0.21, 0.35]
    center_point = [0.25, 0.25, 0.25]
    fs = fuzzy_sets.fuzzy_sets_class()
    M = fs.membership_functions_3D(state_point, center_point)
    print(M[256])
