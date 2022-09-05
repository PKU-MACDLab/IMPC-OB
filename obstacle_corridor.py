from geometry import *
from numpy.core.fromnumeric import argmin
from plot import *
import time as Time
 

def Get_ob_cons(agent,obstacle_list):
    

    K=agent.K 
    D=agent.D

    # the first term of pre_traj is its position after caculating
    # pre_traj_augment=agent.pre_traj[1:].copy()  

    # pre_traj_augment[-1]=agent.tractive_point 
    
    j=0
    for i in range(len(obstacle_list)):
        if obstacle_list[j].get_minimum_distance(agent.p) > agent.r_max:
            del obstacle_list[j]
        else:
            j=j+1
    
    if agent.term_overlap:
        pre_traj_augment=agent.pre_traj[1:].copy()
    else:
        pre_traj_augment=np.block([[agent.pre_traj[1:]],[agent.tractive_point]])
    
    segment_list=get_segment_list(obstacle_list,pre_traj_augment)

    if segment_list is None:

        plot_pre_traj([agent],obstacle_list,True,777)

        raise Exception('the predetermined trajectory is collision')

    
    segment_list_polygon=get_segment_list_polygon(segment_list,pre_traj_augment)
    start=Time.time()
    corridor=[]
    for seg in segment_list_polygon:
        corridor+=[get_polyhedron(obstacle_list,seg)]
    # print("get corridor "+str(Time.time()-start))
    if not agent.term_overlap:
        del segment_list[0][0]

    ob_cons_A=np.zeros((1,D*K))
    ob_cons_B=np.array([-1.0])

    
    for i in range(len(segment_list)):
        num=len(segment_list[i])
        N=len(corridor[i])

        cons_a=np.zeros((num*N,D*K))
        cons_b=np.zeros(num*N) 
        time=0

        for k in segment_list[i]: 

            for plane in corridor[i]:
                
                cons_a[time,k*D:k*D+D]=plane[0:D]
                cons_b[time]=-plane[D] 
                time+=1
        
        ob_cons_A=np.row_stack((ob_cons_A,cons_a))
        ob_cons_B=np.append(ob_cons_B,cons_b)

    
    return ob_cons_A,ob_cons_B ,corridor,segment_list

def get_polyhedron(obstacle_list,segment_polygon):

    distance_list = get_distance_list(obstacle_list,segment_polygon)
    
    polyhedron=[]

    for i in range(len(obstacle_list)):
        a=argmin(distance_list)
        obstacle=obstacle_list[a]
        distance_list[a]=np.inf


        flag=True

        for plane in polyhedron:
            if obstacle.is_out_of_plane(plane):
                flag=False
                break

        if flag:
            plane=get_parting_plane(obstacle.vertex_list,segment_polygon.vertex_list)
            polyhedron.append(plane)

    return polyhedron



def get_segment_list(obstacle_list,pre_traj_augment):

    
    length = len(pre_traj_augment)
    
    segment_list=[]
    
    segment_begin = length-1  
    
    
    for i in range(len(pre_traj_augment)+1): 
         
        segment = [segment_begin] 

        for point in range(segment_begin-1,-1,-1):

            flag=True
            for i in range(segment_begin,point,-1):
               
                l=line(pre_traj_augment[point],pre_traj_augment[i])
                
                c=detect_line_collision(obstacle_list,l,inter=False)
                
                if c:
                    flag=False
                    break

            if flag: 
                segment.append(point)
                if point==0:
                    segment_list.append(segment)
            else:
                segment_begin=point+1

             

                segment_list.append(segment)
                break 
        
        if segment_list[-1][-1] == 0:

            # if len(segment_list[-1])>2:
            #     del segment_list[-1][-1]
            # segment_list.append([1,0])

            return segment_list
    
    print('There has something wrong when getting segment list')
    print(segment_list)
    l=line(pre_traj_augment[segment_list[-1][0]],pre_traj_augment[segment_list[-1][0]-1])
    print(detect_line_collision(obstacle_list,l))
    print(pre_traj_augment[segment_list[-1][0]])
    print(pre_traj_augment[segment_list[-1][0]-1])

    return None
    


def get_segment_list_polygon(segment_list,pre_traj):
    segment_list_polygon=[]
    for i in range(len(segment_list)):
        vertex_list=[]
        for j in range(len(segment_list[i])):
            vertex_list+=[pre_traj[segment_list[i][j]]]
        segment_list_polygon+=[polygon(vertex_list)]
    return segment_list_polygon