#!/usr/bin/env python3
import rospy
import math
import numpy as np
    
    
def DE(func, target, angle, link, n_params, lb, ub,Cr=0.5, F=0.5, NP=10, max_gen=300):
    
    #inisial populasi secara random
    target_vectors = np.random.uniform(low=lb, high=ub, size=(NP, n_params))
    
    donor_vector = np.zeros(n_params)
    trial_vector = np.zeros(n_params)
    ea = []
    best_fitness = np.inf
    list_best_fitness = []
    for gen in range(max_gen):
       # print("Generation :", gen)
        for pop in range(NP):
            #mutasi
            index_choice = [i for i in range(NP) if i != pop]
            a, b, c = np.random.choice(index_choice, 3)
            while a == b or a == c or b == c:
                a, b, c = np.random.choice(index_choice, 3)
            
            #mutasi    
            donor_vector = target_vectors[a] + F * (target_vectors[b]-target_vectors[c])
            donor_vector = np.clip(donor_vector, lb,ub)
            donor_vector = donor_vector.flatten()
            #print("donor",donor_vector)
            #crossover
            cross_points = np.random.rand(n_params) < Cr            
            trial_vector = np.where(cross_points, donor_vector, target_vectors[pop])
            #obj_func
            target_fitness, d = func(target,target_vectors[pop],link)
            trial_fitness, e = func(target,trial_vector,link)
            
            #seleksi
            if trial_fitness < target_fitness:
                angle = e
                target_vectors[pop] = trial_vector.copy()
                best_fitness = trial_fitness
                #angle = angle.tolist()
            else:
                best_fitness = target_fitness
                angle = d
                #angle = angle.tolist()
    
    #    print("Best fitness :", best_fitness)
        #print(angle)
        
    
       
    return best_fitness, angle

