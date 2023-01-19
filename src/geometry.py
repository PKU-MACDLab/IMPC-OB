import time
import numpy as np
from numpy.core.fromnumeric import argmax
from cvxopt import matrix, solvers
from cvxopt.solvers import options
from plot import *
import multiprocessing as mp
import copy
import opengjk.opengjkc as opengjk


# definition of a line
class line():
    
    def __init__(self,p1,p2):
        self.type='line'
        self.p1=p1
        self.p2=p2 
        self.plane=self.get_plane()
        self.num=2
        self.vertex_list=[p1,p2]
        self.line_list=[self]
        return None

    def get_plane(self):

        k=argmax(abs(self.p1-self.p2))
        a=np.zeros(2)
        a[1-k]=1.0
        if max(abs(self.p1-self.p2))<1e-8:
            a[k]=0
        else:
            a[k]=-(self.p1[1-k]-self.p2[1-k])/(self.p1[k]-self.p2[k])
        b=-a @ self.p1
        return [a,b]

    def get_minimum_distance(self,p):

        if np.linalg.norm(self.p1 - self.p2) < 1e-6:
            return np.linalg.norm(self.p1-p)

        t = np.dot(self.p1 -p ,  self.p1 - self.p2) / np.dot(self.p1 -self.p2 ,  self.p1 - self.p2)

        if t<0:
            return np.linalg.norm(self.p1-p)
        elif t>1:
            return np.linalg.norm(self.p2-p)
        else:
            return np.linalg.norm(self.p1+t*(self.p2-self.p1)-p)

    def is_out_of_plane(self,plane):

        a=plane[0:2]
        b=plane[2]

        f1 = a @ self.p1 + b < 1e-8
        f2 = a @ self.p2 + b < 1e-8

        if f1 and f2:
            return True
        else:
            return False

    def offset(self,d):

        R=np.array([[0,1],[-1,0]])

        p1_=self.p1+d*R @ (self.p2-self.p1)/np.linalg.norm(self.p2-self.p1)
        p2_=self.p2+d*R @ (self.p2-self.p1)/np.linalg.norm(self.p2-self.p1)

        return line(p1_,p2_)


# definition of a polygon
class polygon():

    def __init__(self,vertex_list):

        self.type='polygon'
        self.vertex_list=vertex_list
        self.num=len(vertex_list)
        a=np.zeros(2)
        for i in range(self.num):
            a+=vertex_list[i]
        self.center=a/self.num
        self.line_list=[]
        for i in range(self.num):
            self.line_list.append(line(vertex_list[i],vertex_list[(i+1)%self.num]))
        return None

    def get_minimum_distance(self,p):
        # the get minimum distance to a point

        minimum_distance_list=[]

        l=len(self.vertex_list)

        for i in range(l):
            j=(i+1)%l
            Line=line(self.vertex_list[i],self.vertex_list[j])
            minimum_distance_list+=[Line.get_minimum_distance(p)]
               
        return min(minimum_distance_list)
    
    def is_out_of_plane(self,plane):

        a=plane[0:2]
        b=plane[2]

        for v in self.vertex_list:
            if a @ v + b > 0:
                return False 
        
        return True

# define a rectangle based on polygon
def rectangle(p,l,h,w=0.0):

    return polygon([ p - np.array([w,w]), p + np.array([l+w,-w]),\
           p + np.array([l+w,h+w]), p + np.array([-w,h+w]) ])



# 检测点是否在某一个障碍物内
def detect_point_in_ob(ob, p):

    vertex_list=ob.vertex_list
    l=ob.num
    angle_list=[]    

    for i in range(l):

        v1 = vertex_list[i]
        v2 = vertex_list[(i+1)%l]

        l1 = v1-p  
        l2 = v2-p  

        l1_=l1.copy()

        l1_[0]=-l1[1]
        l1_[1]=l1[0]

        if l1_ @ l2 > 0.0:
            sign=1.0
        else:
            sign=-1.0

        cos_angle_i = l1 @ l2 /np.linalg.norm(l1)/np.linalg.norm(l2)
        a=1-cos_angle_i**2
        if a <= 0:
            sin_angle_i = 0.0
        else:
            sin_angle_i = np.sqrt(a)* sign

        angle_i = np.arctan2(sin_angle_i,cos_angle_i)  

        angle_list.append(angle_i)
    
    # Adopting the absolute value instead of just sum, both direction of polygon are ok. 

    if abs(sum(angle_list)) > 0.1:
        return True
    else:
        return False


# 检测点是否在障碍物内
def detect_point_in(obstacle_list,p):

    for ob in obstacle_list: 
        if ob.type != 'line':
            if detect_point_in_ob(ob,p):
                return True
    return False


# 检测线线碰撞
def detect_line_line_collision(line1,line2):

    if (line2.plane[0] @ line1.p1 + line2.plane[1] > 0) == (line2.plane[0] @ line1.p2 + line2.plane[1] > 0):
        return False
    elif (line1.plane[0] @ line2.p1 + line1.plane[1] > 0) == (line1.plane[0] @ line2.p2 + line1.plane[1] > 0):
        return False 
    else:
        return True


# 检测两个多边形的碰撞情况
#### 
def detect_polygon_polygon_collision(poly1,poly2):

    

    l1=len(poly1.vertex_list)
    l2=len(poly2.vertex_list)
    poly1=np.block([[np.array(poly1.vertex_list),np.zeros((l1,1))]]) 
    poly2=np.block([[np.array(poly2.vertex_list),np.zeros((l2,1))]])   

    # print(poly1)
    # print(poly2)

    d=opengjk.gjk(poly1,poly2)

    if abs(d) < 1e-6:
        return True
    else:
        return False


def detect_polygon_line_collision(poly,line,flag=True):

    l1=len(poly.vertex_list)
    l2=len(line.vertex_list)
    poly=np.insert(np.array(poly.vertex_list),2,np.zeros(l1),axis=1) 
    line=np.insert(np.array(line.vertex_list),2,np.zeros(l2),axis=1) 

    d=opengjk.gjk(poly,line)

    if abs(d) < 1e-6:
        return True
    else:
        return False


def detect_line_collision(obstacle_list,line,inter=True):

    for ob in obstacle_list:
        if detect_polygon_line_collision(ob,line,inter):
            return True 
    
    return False 

def detect_polygon_collision(obstacle_list,poly):

    for ob in obstacle_list:

        if detect_polygon_polygon_collision(ob,poly):
            
            return True

    return False



def get_distance_list(obstacle_list,polygon):

    distance_list=np.zeros(len(obstacle_list))
    p=polygon.center

    i=0
    for ob in obstacle_list:
        distance_list[i]=ob.get_minimum_distance(p)
        i=i+1

    return distance_list

def grid(obstacle_list,resolution,map_range):
    
    

    length=int( (map_range['x'][1]-map_range['x'][0])/resolution )
    width=int( (map_range['y'][1]-map_range['y'][0])/resolution )

    
    items=[]

    for i in range(length):
        items+=[[i,width, copy.deepcopy(obstacle_list),resolution,copy.deepcopy(map_range)]]

    start=time.time()
    pool=mp.Pool(20)
    grid_map=pool.map(grid_width, items)
    pool.close()
    pool.join()
    grid_map=np.array(grid_map)

    print("map size is: "+str(length)+" x "+str(width)+" and uses time: "+str(time.time()-start))
    np.savetxt('map/grid_map.csv',grid_map)

    return grid_map

def grid_width(item):

    i,width,obstacle_list,resolution,map_range=item
    x_0=map_range['x'][0]
    y_0=map_range['y'][0]

    grid_map_width=np.zeros(width,dtype=int)
    for j in range(width+1):
        if detect_point_in(obstacle_list,np.array([i*resolution+x_0,j*resolution+y_0])):
            grid_map_width[min(j,width-1)]=1
            grid_map_width[max(0,j-1)]=1
        if detect_point_in(obstacle_list,np.array([(i+1)*resolution+x_0,j*resolution+y_0])):
            grid_map_width[min(j,width-1)]=1
            grid_map_width[max(0,j-1)]=1
    return grid_map_width

def get_parting_plane(list1,list2):

    # list1 是障碍物

    options.update({'show_progress':False}) 

    num=len(list1)
    h=len(list2)
    
    A=np.zeros((h+num+6,3))
    b=np.zeros(h+num+6)
    
    A[0:num,2]=1
    A[num:h+num,2]=-1
    for i in range(0,num):
        A[i][0:2]=list1[i]
    for i in range(num,num+h):
        A[i][0:2]=-list2[i-num]

    A[num+h:num+h+3,0:3]=np.eye(3)
    A[num+h+3:num+h+6,0:3]=-np.eye(3)

    b[num+h:num+h+3]=np.array([1,1,100])
    b[num+h+3:num+h+6]=np.array([1,1,100])

    c=np.zeros(3)
    c[2]=-1
    c[0:2]=-list2[-1] # 最后一个点离分割面的距离尽量的远
    
    res =solvers.lp(matrix(c),matrix(A),matrix(b))
    a=res['x'][0:2]
    
    plane=(np.array(res['x'].T)[0])/np.linalg.norm(a)
    
    return plane 



def SVM(list1,list2):

    # list1是障碍物

    options.update({'show_progress':False}) 
    # list1 is the vertex list of obstacle

    n1=len(list1)
    n2=len(list2)
    
    G=np.zeros((n1+n2,3))
    h=np.zeros(n1+n2)

    
    G[0:n1,2]=1.0
    G[n1:n1+n2,2]=-1.0

    for i in range(0,n1):
        G[i][0:2]=list1[i]
    for i in range(n1,n1+n2):
        G[i][0:2]=-list2[i-n1]

        h[i]=-1
    
    P=np.eye(3)
    P[2][2]=0
    q=np.zeros(3)


    res = solvers.qp(P=matrix(P),q=matrix(q),G=matrix(G),h=matrix(h))
    
    a=res['x'][0:2]
    
    plane=(np.array(res['x'].T)[0])/np.linalg.norm(a)


    return plane




#求两直线交点P
def calculate_IntersectPoint(line_1, line_2):

    #获取两直线的参数a和b
    line_1_co = line_1.get_plane()
    line_2_co = line_2.get_plane()

    #求解线性矩阵方程，获取交点坐标
    equa_A = np.array([line_1_co[0], line_2_co[0]])
    equa_b = np.array([[line_1_co[1]], [line_2_co[1]]])

    equa_A_inv = (1/(equa_A[0][0]*equa_A[1][1] - equa_A[0][1]*equa_A[1][0])) * np.array([[equa_A[1][1], -1*equa_A[0][1]],[-1*equa_A[1][0], equa_A[0][0]]])
    P = -1*(equa_A_inv @ equa_b)        
    P = np.array([P[0][0],P[1][0]])

    return P


# 构建所有三角形障碍物对应的扩展六边形区域
def Build_ExtensionZone(obstacle_list,ExtendWidth):

    extend_path_plan_obstacle_list=[]

    #循环遍历SET中设置的所有三角形障碍物
    for ob in obstacle_list:

        if ob.type=='line':
            line_offset1=ob.offset(-ExtendWidth)
            line_offset2=ob.offset(ExtendWidth)
            l1=line(line_offset1.p1,line_offset2.p1)
            l2=line(line_offset1.p2,line_offset2.p2)
            line_offset1=l1.offset(ExtendWidth)
            line_offset2=l2.offset(-ExtendWidth)
            new_vertex_list=[line_offset1.p1,line_offset2.p1,line_offset2.p2,line_offset1.p2]
            extend_path_plan_obstacle_list+=[polygon(new_vertex_list)]
        else:
            new_vertex_list=[]
            for i in range(ob.num):

                l1=ob.line_list[(i-1)%ob.num]
                l2=ob.line_list[i]
                
                l1_offset=l1.offset(ExtendWidth)
                l2_offset=l2.offset(ExtendWidth)

                p=ob.vertex_list[i]
                p1=l1_offset.vertex_list[1]
                p2=l2_offset.vertex_list[0]
                
                offline=(p2-p+p1-p)/np.linalg.norm(p2-p+p1-p)
                p_=p+offline*ExtendWidth
                vertical=np.array([[0.0,1.0],[-1.0,0.0]]) @ offline
                verticalline=line(p_,p_ + vertical)

                v1=calculate_IntersectPoint(l1_offset,verticalline)
                v2=calculate_IntersectPoint(l2_offset,verticalline)

                new_vertex_list+=[v1,v2]
            extend_path_plan_obstacle_list+=[polygon(new_vertex_list)]
    
    return extend_path_plan_obstacle_list