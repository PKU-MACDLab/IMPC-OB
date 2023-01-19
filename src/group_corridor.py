import numpy as np
from geometry import *
from numpy.core.fromnumeric import argmin
import time
import multiprocessing as   mp
import copy

# 将获得的走廊转化为线性约束
def Get_group_cons(agent,group_corridor_list):

    K=agent.K 
    D=agent.D

    cons_A=np.zeros((1,D*K))
    cons_B=np.array([-1.0])

     
    for k in range(K):

        N=len(group_corridor_list[k])
        cons_a=np.zeros((N,D*K))
        cons_b=np.zeros(N) 
        time=0
        
        for plane in group_corridor_list[k]:
             
            cons_a[time,k*D:k*D+D]=plane[0:D]
            cons_b[time]=-plane[D] 
            time+=1
        
        cons_A=np.row_stack((cons_A,cons_a))
        cons_B=np.append(cons_B,cons_b)

    return  cons_A,cons_B  

# 获得群体生成的安全走廊
def Get_list_of_group_corridor(group_list,agent_list,obstacle_list):

    obstacle_list = copy.deepcopy(obstacle_list)
    # a list includes all group's corridor
    list_of_group_corridor=[]
    
    for i in range(len(agent_list)):
        list_of_group_corridor.append([[] for k in range(agent_list[i].K)])
    

    for group in group_list:

        # including all the agents in this group
        group_agent_list=[]

        for i in group:
            group_agent_list+=[agent_list[i]]
        start=time.time()
        group_corrdior=Get_group_corridor(group_agent_list,obstacle_list)
        # print("corridor time :"+str(time.time()-start))

        for i in group:
            for k in range(agent_list[i].K):
                list_of_group_corridor[i][k]+=group_corrdior[k]

    # items = []
    # for group in group_list:

    #     # including all the agents in this group
    #     group_agent_list=[]

    #     for i in group:
    #         group_agent_list+=[agent_list[i]]

    #     items += [[group_agent_list,obstacle_list]]
    
    # group_corridor_list = [Get_group_corridor(items[i]) for i in range(len(items))]
    # # pool = mp.Pool(len(items))
    # # group_corridor_list=pool.map(Get_group_corridor, items)
    # # pool.close()
    # # pool.join()

    # j=0
    # for group in group_list:
    #     for i in group:
    #         for k in range(agent_list[i].K):
    #             list_of_group_corridor[i][k]+=group_corridor_list[j][k]

    #     j+=1

    return list_of_group_corridor


# 对每一个group划分安全走廊
def Get_group_corridor(agent_list,obstacle_list):


    # a list includes all agents' pre_traj in this group
    group_pre_traj_list=[]
    # a list includes all agents' tractive points
    group_tractive_point_list=[]


    for agent in agent_list:   
        group_pre_traj_list+=[agent.pre_traj]
        group_tractive_point_list+=[agent.tractive_point] 

    len_list = []

    for i in range(len(agent_list)):
        len_list  = len_list + [ len(group_pre_traj_list[i]) ]

    # 这个算法可以应对不同的视界长度
    items=[]

    # pay attention that horizon 0's positon i.e., the current position in next time step is omiited here
    for k in range(1,max(len_list)):
        items+=[[k,group_pre_traj_list,group_tractive_point_list,obstacle_list]]

    group_corridor=[Get_polyhedron(items[i]) for i in range(len(items))]

    # 验证证明以下这个地方不适合使用多进程运算
    # pool = mp.Pool(len(items))
    # group_corridor=pool.map(Get_polyhedron, items)
    # pool.close()
    # pool.join()


    return group_corridor


# 对每个视界进行安全区域划分
def Get_polyhedron(item):
    
    k=item[0] 
    group_pre_traj_list=item[1]
    group_tractive_point_list=item[2]
    obstacle_list=item[3]

    # a list inludes all agents' horizon position
    vertex_list = []

    # a list includes the next posiiton of all agents' horizon position
    next_vertex_list =[]

    for i in range(len(group_pre_traj_list)):

        if k >=len(group_pre_traj_list[i])-1:
            vertex_list+=[group_pre_traj_list[i][-1]]
            next_vertex_list+=[group_tractive_point_list[i]]
        else:
            vertex_list+=[group_pre_traj_list[i][k]]
            next_vertex_list+=[group_pre_traj_list[i][k+1]]


    distance_list = get_distance_list(obstacle_list,polygon(vertex_list))
    
    plane_list=[]
    for i in range(len(obstacle_list)):
        a=argmin(distance_list)
        obstacle=obstacle_list[a]
        distance_list[a]=np.inf
        
        plane=get_polyhedron_plane(obstacle,plane_list,vertex_list,next_vertex_list)
        
        if plane is not None:
            plane_list.append(plane)

    return plane_list


def get_polyhedron_plane(ob,plane_list,vertex_list,next_vertex_list):
    
    if detect_polygon_polygon_collision(ob,polygon(vertex_list)):
        raise Exception("group collision happen")

    for plane in plane_list:
        if ob.is_out_of_plane(plane):
            return None
 

    poly=polygon(vertex_list+next_vertex_list)
    
    if not detect_polygon_polygon_collision(ob,poly):
        return SVM(ob.vertex_list,vertex_list+next_vertex_list)
    else:
        # bisection method 
        a=0
        b=1
        length=len(vertex_list)
        for times in range(8):
            
            t=(a+b)/2
            extra_vertex_list_=[t*next_vertex_list[i]+(1-t)*vertex_list[i] for i in range(length)]
            
            poly=polygon(vertex_list+extra_vertex_list_)
            if detect_polygon_polygon_collision(ob,poly):
                b=t
            else:
                a=t 
        extra_vertex_list_=[a*next_vertex_list[i]+(1-a)*vertex_list[i] for i in range(length)]
        
        return SVM(ob.vertex_list,vertex_list+extra_vertex_list_)