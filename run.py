import SET 
from uav import *
import numpy           as   np
import multiprocessing as   mp
from thread import *
from obstacle_corridor import *
from inter_avoid import *
import cvxpy    as   cp
from plot import *
from geometry import * 
from others import *
import time
from plot import *
import copy
from scipy import linalg as lg
from cvxopt import matrix, solvers
from cvxopt.solvers import options


# this is the main loop
def run_one_step(agent_list,obstacle_list):

    start=time.time()

    ######## communicate data ########

    share_data = get_share_data(agent_list)


    ######## the main caculation #######
    start=time.time()
    
    # the calculation for each agent

    items=[]
    for i in range(SET.Num):
        items.append( [copy.deepcopy(agent_list[i]),copy.deepcopy(share_data),copy.deepcopy(obstacle_list)] )

    if SET.compute_model=='thread':

        pool = []
        for i in range(SET.Num):
            thread = MyThread(run_one_agent,args=[items[i]])
            pool.append(thread)
            thread.start()


        agent_list=[]
        for thread in pool:
            thread.join() 
            agent_list.append(thread.get_result())

    elif SET.compute_model=='norm':

        agent_list=[run_one_agent(item) for item in items]
        

    elif SET.compute_model=='process':
        
        pool = mp.Pool(SET.core_num)
        agent_list=pool.map(run_one_agent, items)
        pool.close()
        pool.join()

    else:

        raise ValueError('Please choose a compute model')


    return agent_list

    

def run_one_agent(item):

    agent=item[0]
    share_data=item[1]
    obstacle_list=item[2]


    # get inter robot avoidance constraints
    agent.inter_cons_A,agent.inter_cons_B,agent.inter_cons_C,agent.Rho_ij = Get_inter_cons(agent,share_data)
    start=time.time()

    # get obstacle collision avoidance constraints
    agent.ob_cons_A, agent.ob_cons_B,agent.ob_corridor_list,agent.segment_list = Get_ob_cons(agent,obstacle_list)
    # print(str(agent.index)+" obstacle constraint time: "+str(time.time()-start))


    start=time.time()
    # running convex program
    agent.cache=run_convex_program(agent)
    # print(str(agent.index)+" convex time: "+str(time.time()-start))

    # post processing
    agent.post_processing()


    return agent


# run convex program of each agents
def run_convex_program(agent):

    type=agent.type
    
    ####### functional constraints related #######
    inter_cons_A = agent.inter_cons_A
    inter_cons_B = agent.inter_cons_B
    inter_cons_C = agent.inter_cons_C
    ob_cons_A = agent.ob_cons_A
    ob_cons_B = agent.ob_cons_B
    Rho_ij = agent.Rho_ij
    epsilon = agent.epsilon
    buffer=agent.buffer


    ####### dynamic related #######

    state = agent.state

    K = agent.K 
    D = agent.D

    VA=agent.VA
    VB = agent.VB
    VC=agent.VC

    Theta_u = agent.Theta_u
    Theta_v=agent.Theta_v
    Theta_p=agent.Theta_p
    
    Umax = agent.Umax
    Vmax = agent.Vmax

    Xi = agent.Xi
    Phi = agent.Phi
    Xi_K = agent.Xi_K

    G_p = agent.G_p
    W = agent.W.copy()

    ######## objective function related #######

    if np.linalg.norm(agent.terminal_p-agent.target) > 2.0:
        Q_tar = 10.0*2.0/np.linalg.norm(agent.terminal_p-agent.target)
    else:
        Q_tar = 10.0

    if type == "Free-transitor" or type == "Obstacle-transitor":

        cost_index=agent.cost_index

        # get the needed weight coefficient matrix
        Sigma=np.zeros([ D* K, D* K])
        
        for i in range(max([cost_index-1,0]), K):
            for j in range( D):
                Sigma[ D*i+j][ D*i+j]=Q_tar

        Delta_P = 1.0*agent.Delta_P

        Q= VB.T @  Phi.T @ Sigma @  Phi @  VB  +  VB.T @  Phi.T @ Delta_P @  Phi @  VB
        p = 2* VB.T @  Phi.T @ Sigma @ (  Phi @  ( VA @ state + VC ) - G_p) +\
            2* VB.T @  Phi.T @ Delta_P @ Phi @  ( VA @ state + VC ) 
    else:

        raise Exception("Please choose the type of robot")
    
    ##############################
    ####### convex program #######
    ##############################
    
    
    len_U=D*K
    len_E=inter_cons_C.shape[1]
    len_ob_B=len(ob_cons_B)

    l=len_U+len_E+len_ob_B

    M_log=Rho_ij/W/2/epsilon
    
    M_log=np.diag(M_log) 

    P = lg.block_diag(Q,M_log,20000*np.eye(len_ob_B))
    q = np.block([p,-2*M_log @ np.ones(len_E)*epsilon ,np.zeros(len_ob_B)])

    

    # inter avoidance constraints
    len_cons_B = len(inter_cons_B)

    G_1 = np.zeros((len_cons_B,l))
    G_1[0:len_cons_B,0:len_U] = -inter_cons_A @  Phi @ VB
    G_1[0:len_cons_B,len_U:len_U+len_E] = inter_cons_C

    h_1 =  inter_cons_A @  Phi @ ( VA @ state + VC) - inter_cons_B 

    # E lower bound constraint
    G_2 = np.zeros((len_E,l))
    G_2[0:len_E,len_U:len_U+len_E] = -np.eye(len_E)
    
    h_2 = np.zeros(len_E)

    # E upper bound constraint
    G_3 = np.zeros((len_E,l))
    G_3[0:len_E,len_U:len_U+len_E] = np.eye(len_E)
    
    h_3 = np.ones(len_E)*epsilon

    # obstacle aviodance constriants
    len_ob_cons_B = len(ob_cons_B)

    G_4 = np.zeros((len_ob_B,l))
    G_4[0:len_ob_cons_B,0:len_U] = - ob_cons_A @  Phi @ VB
    G_4[0:len_ob_cons_B,len_U+len_E:len_U+len_E+len_ob_B] = -np.eye(len_ob_B)

    h_4 =  ob_cons_A @  Phi @ ( VA @ state + VC) - ob_cons_B -buffer

    # ob_B up nound constraints

    G_5 = np.zeros((len_ob_cons_B,l))
    G_5[0:len_ob_cons_B,len_U+len_E:len_U+len_E+len_ob_B] = np.eye(len_ob_B)

    h_5 = np.ones(len_ob_B)*buffer

    l_nonnegative_orthant = len(h_1) + len(h_2) + len(h_3) + len(h_4) + len(h_5)


    # terminal constraint
    A = np.zeros((D,l))
    A[0:D,0:len_U] = Xi_K @ VB

    b= np.zeros(D) - Xi_K @ ( VA @ state + VC)


    # acceleration constraints
    G_cone_1 = np.zeros(((D+1)*K,l))
    h_cone_1 = np.zeros((D+1)*K)
    for k in range(K):
        e_k = np.zeros((D,D*K))
        e_k[0:D,D*k:D*k+D] = np.eye(D)
        G_cone_1[(D+1)*k+1:(D+1)*(k+1),0:len_U] = e_k
        h_cone_1[(D+1)*k] = Umax


    # velocity constraints
    G_cone_2 = np.zeros(((D+1)*K,l))
    h_cone_2 = np.zeros((D+1)*K)
    for k in range(K):
        e_k = np.zeros((D,D*K))
        e_k[0:D,D*k:D*k+D] = np.eye(D)
        G_cone_2[(D+1)*k+1:(D+1)*(k+1),0:len_U] = e_k @ Xi @ VB 
        h_cone_2[(D+1)*k] = Vmax
        h_cone_2[(D+1)*k+1:(D+1)*(k+1)] = - e_k @ Xi @ (VA @ state + VC)

    
    G = matrix( np.block([ [G_1],[G_2],[G_3],[G_4],[G_5],[G_cone_1],[G_cone_2] ]) )
    h = matrix( np.block([h_1,h_2,h_3,h_4,h_5,h_cone_1,h_cone_2]) )
    A = matrix(A)
    b = matrix(b)
    P=2*matrix(P)
    q=matrix(q)

    dims = {'l': l_nonnegative_orthant, 'q': [D+1 for i in range(K+K)], 's': []}

    options.update({'show_progress':False}) 
    

    res = solvers.coneqp(P=P,q=q,G=G,h=h,A=A,b=b,dims=dims)

    U = (np.array(res['x'][0:len_U]))[:,0]
    E = (np.array(res['x'][len_U:l]))[:,0]
    
    return [VA @ state + VB @ U + VC, U.reshape((K,D)), E[1:inter_cons_C.shape[1]]] 