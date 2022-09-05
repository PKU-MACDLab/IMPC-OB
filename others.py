import numpy as np
import time
from geometry import *


def get_share_data(agent_list):

    pre_traj_list=[]
    type_list=[]
    for agent in agent_list:
        pre_traj_list+=[agent.pre_traj]
        type_list+=[agent.type]

    share_data={'pre_traj':pre_traj_list,'type':type_list}

    return share_data



def check_reach_target(agent_list):

    REACHGOAL=True

    for agent in agent_list:

        if agent.type == 'Searcher' and agent.cost_index == 0:
            flag=True
            for agent in agent_list:
                if np.linalg.norm(agent.pre_traj[0]-agent.pre_traj[-1])>1e-2:
                    flag=False
                    break
            if flag:     
                return True
        if not agent.cost_index == 0:
            return False
        
    return REACHGOAL

def save_data(agent_list):


    data=agent_list[0].data
    
    for i in range(1,len(agent_list)):
        if agent_list[i].type != "Anchor":
            data=np.block([[data,agent_list[i].data]])

    filename='data/data.csv'
    
    np.savetxt(filename,data)

def save_path(agent_list,km):

    data=agent_list[0].path
    
    for i in range(1,len(agent_list)):
        if agent_list[i].type != "Anchor":
            data=np.block([[data],[-9999999*np.ones(2)],[agent_list[i].path]])

    filename='data/'+str(km)+'path.csv'
    
    np.savetxt(filename,data)


def get_sample_position(Num,obstacle_list,r_min,square):

    d=round(len(square)/2)

    a=time.time()*1e8
    a=np.round(a)
    np.random.seed(a.astype(int)%46547)

    while True:

        ini_x=[]

        for i in range(Num):

            for j in range(1000): 
            
                ini=np.array(square[0:d]) + np.random.rand(d)*np.array(square[d:2*d])
                RIGHT=True

                if detect_point_in(obstacle_list,ini):
                    RIGHT=False
                    break

                for k in range(len(ini_x)):
                    if(np.linalg.norm(ini-ini_x[k])<r_min):
                        RIGHT=False 
                        break
                
                if RIGHT:
                    ini_x+=[ini]
                    break 

        if len(ini_x)==Num:
            break     

    print("Get sample positions")
    return ini_x