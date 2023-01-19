import numpy as np
import multiprocessing as mp
import SET


def Get_list_of_connection_constraint_list(connection_list,agent_list):

    # this list including all the connection constraints of all agents 
    list_of_connection_constraint_list=[[] for i in range(len(agent_list))]

    items=[[connection_list[i],agent_list,SET.d_connect] for i in range(len(connection_list))]


    # connection_constraints list includes all the constraints related to all the connections
    # pool = mp.Pool(3)
    # connection_constraint_list=pool.map(get_connection_constraints, items)
    # pool.close()
    # pool.join()

    connection_constraint_list=[get_connection_constraints(items[i]) for i in range(len(items))]

    for k in range(len(connection_list)):
        for i in connection_list[k]:
            list_of_connection_constraint_list[i].append(connection_constraint_list[k])

    return list_of_connection_constraint_list


# getting the constraint related to one connection
def get_connection_constraints(item):

    
    connection=item[0]
    agent_list=item[1]
    d_connect=item[2]

    # a connection includes two index i and j
    i=connection[0]
    j=connection[1]

    # get the length of i and j's predetermined trajectories
    l_i=len(agent_list[i].pre_traj)
    l_j=len(agent_list[i].pre_traj)

    D=agent_list[0].D 
    center_list=np.zeros((max(l_i,l_j)-1,D))
    r_list=np.zeros(max(l_i,l_j)-1)

    
    for k in range(1,max(l_i,l_j)):

        if k < l_i-1:
            p_i_=agent_list[i].pre_traj[k+1]
        else:
            p_i_=agent_list[i].tractive_point
        
        if k <= l_i-1:
            p_i=agent_list[i].pre_traj[k]
        else:
            p_i=agent_list[i].terminal_p

        if k < l_j-1:
            p_j_=agent_list[j].pre_traj[k+1]
        else:
            p_j_=agent_list[j].tractive_point

        if k <= l_j-1:
            p_j=agent_list[j].pre_traj[k]
        else:
            p_j=agent_list[j].terminal_p

        if np.linalg.norm(p_i-p_j) > d_connect:
            print(p_i)
            print(p_j)
            error='the distcance between '+str(agent_list[i].index)+' and '\
                +str(agent_list[j].index)+' is out of communicated distance'
            raise Exception(error)

        center,r=get_circle(p_i,p_i_,p_j,p_j_,d_connect)

        center_list[k-1]=center
        r_list[k-1]=r

    # print('#######')
    # print(center_list)
    # print(p_i)
    # print(p_j)
    # print(p_i_)
    # print(p_j_)

    return [center_list,r_list]


# get the constrainted circle for a connection in a horizon
def get_circle(p_i,p_i_,p_j,p_j_,d_connect):

    # determined the dimension of position 
    center=np.zeros(len(p_i))

    r=d_connect/2
    # r_ is the warnning connection distance
    r_=0.95*d_connect/2

    p_center=(p_i+p_i_+p_j+p_j_)/4
    p_i_j=(p_i+p_j)/2

    # if the current distance reach the warnning distance, the center of the constrainted circle will be chosen as their midpoint
    if np.linalg.norm(p_i-p_j)>2*r_:
        center=p_i_j.copy()
    # otherwise, if the center point of four points can includes these two points, it will be chosen as the center of circle
    elif np.linalg.norm(p_center-p_i) < r_ and np.linalg.norm(p_center-p_j) < r_:
        center=p_center.copy()
    # otherwise, the center point will be defined by minimum dichotomy
    else:
        a=0
        b=1
        for i in range(8):
            t=(a+b)/2
            p_t=(1-t)*p_i_j+t*p_center

            if np.linalg.norm(p_t-p_i) < r_ and np.linalg.norm(p_t-p_j) < r_:
                a=t
            else:
                b=t

        t=a
        center=(1-t)*p_i_j+t*p_center

    return center,r