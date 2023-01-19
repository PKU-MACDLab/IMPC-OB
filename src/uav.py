from turtle import distance
import numpy as np
from scipy import linalg as lg
from geometry import *
from path_planning import * 
import copy
import time

class uav2D():


    # initialization
    def __init__(self,index,ini_x,target,type,ini_K=11):

        import SET

        ##############################
        ####### key cofficient #######
        ##############################

        # the index of this agent 
        self.index=index

        # the type of this UAV:
        self.type=type

        # the length of horizon
        self.K=ini_K

        # the dimension of uav
        self.D=2

        self.h=SET.h

        self.buffer=SET.buffer

        self.REALFLY=SET.REALFLY

        self.obstacle_list=copy.deepcopy(SET.obstacle_list)

        #####################
        ####### state #######
        #####################

        # initial position
        self.ini_p=ini_x.copy()

        ini_v=np.zeros(2)

        # current position            
        self.p=ini_x.copy()

        # current velocity
        self.v=ini_v.copy()

        # input
        self.u=np.zeros(self.D)

        # current state including position and velocity 
        self.state=np.append(self.p,self.v)

        # maximum acc
        self.Umax=2.0

        # maximum velocity
        self.Vmax=3.0

        # the redius of a robot (pay attention: it is diameter!)
        self.r_a=0.3

        self.r_min= 2*self.r_a #np.sqrt(4*self.r_a**2+(self.h*self.Vmax)**2)

        self.r_max=self.Vmax*self.h*self.K

        # dynamic matrix
        self.get_dynamic()

        self.get_coef_matrix()

        ##########################
        ####### Trajectory #######
        ##########################

        # target position
        self.target=target

        # terminal position
        self.terminal_p=self.p.copy()

        if type == 'Obstacle-transitor' or type == 'Searcher':

            # get path
            # self.path=path_plan(self.terminal_p,self.target)
            # np.savetxt("data/path"+str(self.index),self.path)
            self.path=np.loadtxt("data/path"+str(self.index))


        # a coefficient related to the objective
        self.cost_index=ini_K


        # the predetermined trajectory
        self.pre_traj=np.zeros((self.K+1,self.D))

        for i in range(self.K+1):
            self.pre_traj[i]=self.p.copy()


        # tractive position
        self.tractive_point=None

        # the tractive position list for objective
        self.G_p=None

        # get tractive point for obstace transitor
        self.get_tractive_point()

        
        # the list of all time's 
        self.pre_traj_list=[]

        # the list of all past position
        self.position=self.p.copy()



        #######################
        ####### Dealock #######
        #######################

        self.rho_0=10.0

        # warning band width
        self.epsilon=0.20

        self.term_overlap=False

        self.term_last_p=self.p.copy()

        self.E=0.0

        self.term_overlap_again=False

        self.term_last_p=self.p.copy()

        self.term_index=0

        self.eta=1.0

        self.E=0.4*self.epsilon*np.ones(SET.Num-1)

        self.W=0.4*self.epsilon*np.ones(SET.Num)

        self.ever_reach_target = False

        self.priority = 2

        self.contest = False

        self.distance = np.linalg.norm(self.p-self.target)

        self.no_supremacy = True 

        self.min_distance = False

        #########################
        ####### crazyfile #######
        #########################
        
        if self.REALFLY:

            # the height of crazyfile is constantly 1m
            self.height=SET.height

            # the yaw of crazyfile is constantly 0.0 deg
            self.yaw=0.0

            # the input trajectory
            self.input_traj=self.get_input_traj()
        
        ###############################
        ####### data collection #######
        ###############################
        
        self.data=np.block([[np.array([self.index,self.D])],\
            [self.ini_p],[self.target],[-9999999*np.ones(self.D)]])
        


    def post_processing(self,share_data,interval=1):

        # data collection
        U_list=self.cache[1]
        
        
        # get new input
        self.u=U_list[0]

        # get predeterminted trajectory and the terminal point
        P=self.Phi @ self.cache[0]
        P=P.reshape(( self.K, self.D))
        self.terminal_p=P[-1].copy()

        self.data=np.block([[self.data],[self.tractive_point],[self.p],[self.v],\
        [-7777777*np.ones(self.D)],[P],[-9999999*np.ones(self.D)]])

        # here, in our code, PT including the current position in next time's replanning  
        self.pre_traj=np.block([[P],[self.terminal_p]])        
        
        

        if not self.REALFLY:
            # 在仿真状态中认为生成轨迹中的第一个位置就是下一时刻的位置
            # get new state
            self.p[0:self.D]=self.cache[0][0:self.D].copy()
            self.v[0:self.D]=self.cache[0][self.D:2*self.D].copy()
            self.state=np.append(self.p,self.v)

            # get position list
            self.position=np.block([ [self.position],[self.p] ])
        else:
            ####### the realfly #######

            # get the input trajectory
            self.input_traj=self.get_input_traj()

            # 在真实运行过程中，会认为本次生成轨迹之后再过 (interval-1)*h 时间后才是下次计算时初始状态的时刻
            self.p[0:self.D]=self.cache[0][2*self.D*(interval-1)+0:2*self.D*(interval-1)+self.D].copy()
            self.v[0:self.D]=self.cache[0][2*self.D*(interval-1)+self.D:2*self.D*(interval-1)+2*self.D].copy()
            self.state=np.append(self.p,self.v)

            # get new state and predetermined tarjectory for crazyfile
            self.get_pre_traj(interval)

            # get position list
            for i in range(1,interval+1):
                p=self.cache[0][2*self.D*(i-1)+0:2*self.D*(i-1)+self.D].copy()
                self.position=np.block([ [self.position],[p] ])

        # get predtermined trajectory list of each time step
        self.pre_traj_list+=[self.pre_traj]

        # get new cost_index
        for i in range( self.K,-1,-1):
            if( np.linalg.norm( self.pre_traj[i]-self.target ) > 0.01 ):
                break
        self.cost_index=i

        if i < self.K:
            self.ever_reach_target = True

        self.W=0.2*np.block([self.epsilon,self.E])+0.8*self.W

        for i in range(len(self.W)):
            if self.W[i] < 0.005:
                self.W[i] = 0.005

        self.distance = np.linalg.norm(self.target-self.terminal_p)

        # deadlcok resolution
        self.deadlock_resolution(share_data)

        # get tractive position
        self.get_tractive_point()
        
        return None


    # transform the predetermined trajectory to the one that can be parsed by crazyfile
    def get_input_traj(self):    

        P=self.pre_traj

        P=np.block([[self.state[0:2]],[P],[P[-1]]]) 
        self.or_pre_traj=P.copy() 

        t = self.h*np.arange(len(P))
        W=np.ones(len(P))
        polyx = np.polyfit(t, P[:,0], 7,w=W)  
        polyy = np.polyfit(t, P[:,1], 7,w=W)

        polyz = np.zeros(8)
        polyz[7]= self.height
        polyyaw = np.zeros(8)
        polyyaw[7] = self.yaw

        t=self.h*(len(P)-1)

        # 这个地方进行了进一部分进行了进一步的修改
        input_traj = [polyx, polyy, polyz, polyyaw]

        return input_traj

        
    # 在crazyfile情况下获得下时刻的状态估计，并且获得估计的预设轨迹，值得一提的是这里的interval是下次计算所需要的时间
    def get_pre_traj(self,interval):

        l=len(self.pre_traj)

        self.pre_traj[0:l-(interval-1)]=self.pre_traj[interval-1:l].copy()

        for i in range(interval-1):
            self.pre_traj[l-1-i]=self.terminal_p.copy() 

        return  None


    def deadlock_resolution(self,share_data):

        term_p=self.pre_traj[-1].copy()
         
        term_second_p=self.pre_traj[-2].copy()
        # term_thrid_p=self.pre_traj[-3].copy()

        self.E=self.cache[2]


        if self.term_overlap:
            
            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.001
            condition_b=np.linalg.norm(term_p-term_second_p)<0.005
            condition_c=np.linalg.norm(term_p-self.target)>0.02

            if condition_a and condition_b and condition_c:
                self.term_overlap_again=True

        else:

            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.005
            condition_b=np.linalg.norm(term_p-term_second_p)<0.01
            condition_c=np.linalg.norm(term_p-self.target)>0.02

            if condition_a and condition_b and condition_c:
                self.term_overlap=True

                # print(str(self.index)+" begins terminal overlap mode")
        
        flag=False

        if(type(self.E) is np.ndarray):
            if (self.epsilon-self.E < 1e-3).all():
                flag=True
        elif self.epsilon-self.E < 1e-3:
                flag=True

        if flag:
            self.term_overlap=False
            self.term_overlap_again=False
            self.eta=1.0
            self.contest = False
            if self.priority==3:
                self.priority=2


        self.term_index+=1

        if self.term_overlap_again and self.eta < 4.0: # and self.term_index > 3:
            self.term_overlap_again=False
            self.eta += 0.3
            self.term_index = 0
           
        self.term_last_p=term_p.copy()

        # print(str(self.index)+"'s terminal overlap is: "+str(self.term_overlap)+" and "+str(self.E))

        priority_list=share_data['priority']
        no_supremacy = True
        for priority in priority_list:
            if priority == 3:
                no_supremacy=False
                break 
        
        distance_list=share_data['distance']
        contest_list=share_data['contest']
        min_distance = True 
        distance = distance_list[self.index]
        for dis,contest in zip(distance_list,contest_list):
            if dis > distance and contest:
                min_distance = False 
                break

        if self.ever_reach_target:
            self.priority = 1
        else:
            if self.eta > 3.7:

                if self.contest and no_supremacy and min_distance:
                    self.priority = 3
                self.contest =True
        
        return None
        

    # get the list of tractive point which is used for tracting the agent to the tractive point 
    def get_tractive_point_list(self):

        G_p=self.tractive_point
        for i in range(1,self.K):
            G_p=np.append(G_p,self.tractive_point)
        self.G_p=G_p

        return None


    # get the tractive point 
    def get_tractive_point(self):


        if self.type == "Free-transitor":

            self.tractive_point = self.target

        elif self.type == "Obstacle-transitor" or self.type == "Searcher":

            obstacle_list=self.obstacle_list
            
            # if the path is None, i.e, the search based planning doesn't a feasible path, the tractive point will be chosen as the terminal point of predetermined trajectory
            if self.path is None:
            
                self.tractive_point=self.terminal_p.copy() # 这个地方以后可能还需要一定的修改
            
            else:
                

                if self.type == "Obstacle-transitor":

                    self.tractive_point=self.terminal_p.copy()
                    
                    # if a collision-free path exists, then we can find the tractive point
                    for i in range(len(self.path)-1,-1,-1):
                        
                        if not detect_line_collision(obstacle_list,line(self.path[i],self.terminal_p)):
                            
                            self.tractive_point = self.path[i].copy()
                            
                            break

                elif self.type == "Searcher":

                    for i in range(len(self.path)):

                        if np.linalg.norm(self.path[i] - self.tractive_point) < 1e-2:

                            p_start = i
                    
                    
                    for i in range(len(self.path)-1,p_start-1,-1):

                        vertex_list = [self.terminal_p]

                        vertex_list += [self.path[k] for k in range(i,p_start-1,-1)]

                        # print(vertex_list)

                        poly = polygon(vertex_list)

                        if not detect_polygon_collision(obstacle_list,poly):

                            self.tractive_point = self.path[i].copy()

                            break
                    
        else:

            self.tractive_point = self.terminal_p.copy()

        # No matter whether path exists or doesn't, there needs a tractive list for convex programming    
        self.get_tractive_point_list()
        
        return None


    # change the target position
    def get_new_target(self,target):

        self.target=target.copy()
        
        # replanning the path
        self.path=path_plan(self.terminal_p,self.target)
        
        # get the new tractive point
        self.get_tractive_point()

        # get new cost_index
        for i in range( self.K,-1,-1):
            if( np.linalg.norm( self.pre_traj[i]-self.target ) > 0.01 ):
                break
        self.cost_index=i
        
        return None


    def get_nei_objective(self,share_data):

        pre_traj_list=share_data['pre_traj']

        # P_neighbor includes all horizon's position
        P_neighbor=np.zeros(self.K*self.D)
        
        P1=pre_traj_list[self.neighbor[0]][1:self.K+1].reshape(1,-1)[0]
        P2=pre_traj_list[self.neighbor[1]][1:self.K+1].reshape(1,-1)[0]
        P_neighbor+=1.5/2*P1+0.5/2*P2

        return P_neighbor


#######################################################
#                                                     #
#                                                     #
#######################################################
#                                                     #
#                                                     #
#######################################################


    def get_coef_matrix(self):
        
        D=self.D
        K=self.K

        # position matrix
        # get all position matrix
        global Phi
        Phi=np.column_stack( (np.eye(D),np.zeros((D,D))) )
        phi=Phi
        for i in range(1,K):
            Phi=lg.block_diag(Phi,phi)
        self.Phi=Phi


        # get K position matrix
        global Phi_K
        Phi_K=np.zeros((D,K*D))
        for i in range(0,D):
            Phi_K[i][K*D-D+i]=1.0
        self.Phi_K=Phi_K @ Phi

        # velocity matrix
        global Xi
        Xi=np.column_stack( (np.zeros((D,D)),np.eye(D)) )
        xi=Xi
        for i in range(1,K):
            Xi=lg.block_diag(Xi,xi)
        self.Xi=Xi

        # get K velocity matrix
        global Xi_K
        Xi_K=np.zeros((D,K*D))
        for i in range(0,D):
            Xi_K[i][K*D-D+i]=1.0
        self.Xi_K=Xi_K @ Xi
        
        # gamma this matrix is used for the maximium input control constraint 
        theta_u=np.array([1.0,1.0])
        Theta_u=theta_u
        for i in range(1,K):
            Theta_u=lg.block_diag(Theta_u,theta_u)
        self.Theta_u=Theta_u

        self.Theta_v=Theta_u.copy()

        self.Theta_p=Theta_u.copy()


        
        
        # control input change cost
        
        Delta=np.eye(K*D)
        for i in range(D):
            Delta[i][i]=0
        for i in range(D,K*D):
            Delta[i][i-D]=-1

        self.Delta=Delta.T @ Delta

        
        Delta_P=np.zeros((K*D,K*D))
        for i in range(1,K):
            for j in range(D):
                Delta_P[i*D+j][i*D+j]=i/K
                Delta_P[i*D+j][i*D-D+j]=-i/K
        
        self.Delta_P=Delta_P.T @ Delta_P

        return None


    def get_dynamic(self):

        K=self.K
        h=self.h

        # system dynamic in continous time
        A=np.array([[0,0,1,0],[0,0,0,1],[0,0,0,0],[0,0,0,0]])
        B=np.array([[0,0],[0,0],[1,0],[0,1]])

        m=A.shape[0]

        # system dynamic
        A=np.dot(np.linalg.inv(np.eye(m)-h/2*A),(np.eye(m)+h/2*A))
        B=np.dot(np.linalg.inv(np.eye(m)-h/2*A)*h,B)

        VA=A
        for i in range(2,K+1):
            C=np.eye(m)
            for j in range(1,i+1):
                C=np.dot(C,A)
            VA=np.block([[VA],[C]])
        self.VA=VA

        VB=B
        for i in range(1,K):
                VB=np.block( [ [ np.dot( np.zeros((m,m)),B ) ],[VB] ] )
        for i in range(1,K):
            C=np.dot( matrixPow(A,i-K+1),B )
            for j in range(i-K+2,i+1):
                C=np.block([[C],[np.dot(matrixPow(A,j),B)]])
            VB=np.block([[C,VB]])
        self.VB=VB

        self.VC=np.zeros(m*K)

        return None


# the power of matrix
def matrixPow(Matrix,n):
    if(type(Matrix)==list):
        Matrix=np.array(Matrix)
    if(n==1):
        return Matrix
    elif(n==0):
        return np.eye(Matrix.shape[0])
    elif(n<0):
        return np.zeros(Matrix.shape)
    else:
        return np.matmul(Matrix,matrixPow(Matrix,n-1))