import numpy as np
from geometry import *

def initialize_set():

###################
#  personal set   #
###################

    global Num     # the number of agents
    Num=8

    # Searcher: the robot transit to a point which have the same topology woth a given path
    # Connector: 
    # Anchor: Anchor must have the last index, and in planning we will not consdier its collision
    # Leader:  
    # Follower:  
    # Free-transitor: the robot transit in a free-space
    # Obstacle-transitor: the robot transit in a obstacle-rich space

    global type_list
    type_list=["Obstacle-transitor","Obstacle-transitor","Obstacle-transitor","Obstacle-transitor",\
               "Obstacle-transitor","Obstacle-transitor","Obstacle-transitor","Obstacle-transitor"]


    global K       # the length of horizon
    K=8  
    global h       # time step
    h=0.15
    global episodes      # the maximum times for replanning
    episodes=100
    global compute_model # the mode that running the convex programming, 'norm' , 'process' and 'thread'
    compute_model='process'      
    global core_num      # if the mode is 'process', then, choosing the number of core apllied for this computing
    core_num=4
    global map_range
    map_range={'x':[0.0,10.0],'y':[0.0,10.0]}
    


    ## parameter for real fly ##

    global REALFLY       # if REALFLY=True, the real fly experiment for using crayfiles
    REALFLY=False
    global height        # the flying height for 2d-test when using crazyfiles
    height=0.5
    global interval      # For real fly, it is an integer a little larger than computation_time/h
    interval=1
    global next_interval
    next_interval=1
    

    ## about the fig ##

    global show          # if show = True, then the fig will be save in file: savefig. 
    show=True            # Pay attentiion that, in realfly test, show must be chosen as 'False' since it will lead to the time delay

    global format # the format of the printed fig
    format='.jpg'

    global plot_range    # if save figure, determine the plot range of this figure
    plot_range={'x':[map_range['x'][0],map_range['x'][1]],'y':[map_range['y'][0],map_range['y'][1]]}
    ratio=(plot_range['y'][1]-plot_range['y'][0])/(plot_range['x'][1]-plot_range['x'][0])
    plot_range['size']=(20,ratio*20)




    global buffer        # the buffer when adopting parting plane linear constraints
    buffer = 0.03        # "obstacle-transitor":0.02

    global ExtendWidth   # The extended width when considering the radius of agents, pay attention: it is radius
    ExtendWidth = 0.3
    
    global resolution
    resolution=0.1


    global ini_x   # intial position
    ini_x=[np.array([2.0,2.0]),np.array([2.0,4.0]),np.array([2.0,6.0]),np.array([2.0,8.0]),\
            np.array([8.0,8.0]),np.array([8.0,6.0]),np.array([8.0,4.0]),np.array([8.0,2.0]) ]

    # target position: is a variable in some condition
    global target
    target=[np.array([8.0,8.0]),np.array([8.0,6.0]),np.array([8.0,4.0]),np.array([8.0,2.0]),\
            np.array([2.0,2.0]),np.array([2.0,4.0]),np.array([2.0,6.0]),np.array([2.0,8.0])]
    

    # the enviromrnt doen't consider the diameter ofrobots
    global ini_obstacle_list

    ini_obstacle_list=[

        rectangle(np.array([0.0,0.0]),10.0,0.001,0.0),
        rectangle(np.array([0.0,0.0]),0.001,10.0,0.0),
        rectangle(np.array([0.0,10.0]),10.0,0.001,0.0),
        rectangle(np.array([10.0,0.0]),0.001,10.1,0.0),
        rectangle(np.array([4.0,0.0]),2.0,3.5,0.0),
        rectangle(np.array([4.0,6.5]),2.0,3.5,0.0)
    ]

    # the obstacle-inflated enviroment which consider the diameter of robots
    global obstacle_list
    obstacle_list=[

        rectangle(np.array([0.0,0.0]),10.0,0.001,ExtendWidth),
        rectangle(np.array([0.0,0.0]),0.001,10.0,ExtendWidth),
        rectangle(np.array([0.0,10.0]),10.0,0.001,ExtendWidth),
        rectangle(np.array([10.0,0.0]),0.001,10.0,ExtendWidth),
        rectangle(np.array([4.0,0.0]),2.0,3.5,ExtendWidth),
        rectangle(np.array([4.0,6.5]),2.0,3.5,ExtendWidth)

    ]

    # the enviroment used for path planning
    global path_obstacle_list


    path_obstacle_list=Build_ExtensionZone(obstacle_list,0.1)

    path_obstacle_list=grid(path_obstacle_list,resolution,map_range)