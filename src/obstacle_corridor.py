from geometry import *
from numpy.core.fromnumeric import argmin
from plot import *
import time as Time
 

def Get_ob_cons(agent,obstacle_list):


    K=agent.K 
    D=agent.D

    # the first term of pre_traj is its position after caculating
    # pre_traj_augment=agent.pre_traj[1:].copy()  

    # 在预设轨迹的末端加入牵引点
    # pre_traj_augment[-1]=agent.tractive_point 
    
    j=0
    for i in range(len(obstacle_list)):
        if obstacle_list[j].get_minimum_distance(agent.p) > agent.r_max:
            del obstacle_list[j]
        else:
            j=j+1
    
    # if agent.term_overlap:
    #     # 这是避免死锁的一种途径
    #     pre_traj_augment=agent.pre_traj[1:].copy()
    # else:
    # 在预设轨迹的末端加入牵引点
    pre_traj_augment=np.block([[agent.pre_traj[1:]],[agent.tractive_point]])
    
    start = Time.time()
    
    # 判断如何将预设轨迹分入哪些块，这里的运算速度太差了，都快赶上convex program了
    segment_list=get_segment_list(obstacle_list,pre_traj_augment)


    if segment_list is None:

        plot_pre_traj([agent],obstacle_list,True,777)

        raise Exception('the predetermined trajectory is collision')

    
    # 将每一个走廊里的点都包装为一个多边形块
    segment_list_polygon=get_segment_list_polygon(segment_list,pre_traj_augment)
    start=Time.time()
    # 将多边形块与障碍物之间通过线性约束生成走廊
    corridor=[]
    for seg in segment_list_polygon:
        corridor+=[get_polyhedron(obstacle_list,seg)]

    # if not agent.term_overlap:
        # 这里要删除它的原因是，这个元素是牵引点，不在序列之内
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

    # 按照距离，由近至远，相对障碍物生成分割面，假如原有的分割面可以将障碍物与片段进行分割，那么就会忽略这一障碍物
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
            plane=SVM(obstacle.vertex_list,segment_polygon.vertex_list)
            polyhedron.append(plane)

    return polyhedron



def get_segment_list(obstacle_list,pre_traj_augment):

    
    length = len(pre_traj_augment)
    
    segment_list=[]
    
    segment_begin = length-1  # 这个变量表示片段的起点，长度减一表示从末尾开始
    
    
    for i in range(len(pre_traj_augment)+1): # 分段数量不应该超过总的视界长度
         
        segment = [segment_begin] # 片段的开头是起点

        # 寻找新片段的可加入点
        for point in range(segment_begin-1,-1,-1):

            

            
            # 这个地方重新写吧
            flag=True
            # for i in range(segment_begin,point,-1):
               
            #     # 判断增加新点（point）之后，这一片段是否会存在碰撞
            #     l=line(pre_traj_augment[point],pre_traj_augment[i])
                
            #     c=detect_line_collision(obstacle_list,l,inter=False)
                
            #     if c:
            #         flag=False
            #         break

            vertex_list=[]
            for i in range(segment_begin,point-1,-1):
                vertex_list += [pre_traj_augment[i]]

            c = detect_polygon_collision(obstacle_list,polygon(vertex_list))

            if c:
                flag = False

            if flag: 
                # 如果不存在碰撞，则加入新的点（point）
                segment.append(point)
                # 如果这个点已经是最后一个点了，那么这一片段自动终结
                if point==0:
                    segment_list.append(segment)
            else:
                # 如果这个点存在碰撞，那么选择这个点的上一个点作为下一片段的开头，并且终结这一片段
                segment_begin=point+1

                # 更改segement划分一共需要两步：下面是第一步
                # if len(segment)>2:
                #     del segment[-1]
                #     segment_begin=point+2

                segment_list.append(segment)
                break 
        
        # 如果片段链的最后一个片段的最后一个点是开始点，那么终结片段划分
        if segment_list[-1][-1] == 0:

            # 更改segement划分一共需要两步：下面是第二步
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