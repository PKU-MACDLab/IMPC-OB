from geometry import *
import SET
import numpy as np
import  matplotlib.pyplot as plt
from matplotlib.patches import Circle

color=[ '#1f77b4',
        '#ff7f0e', 
        '#2ca02c', 
        '#d62728',
        '#9467bd',
        '#8c564b', 
        '#e377c2',
        '#7f7f7f', 
        '#bcbd22',
        '#17becf',
        '#2F4F4F',
        '#CD5C5C',
        '#ADD8E6',
        '#663399',
        '#8FBC8F',
        '#00CED1',
        '#6A5ACD',
        '#808000',
        '#A0522D',
        '#FF4500',
        '#708090',
        '#BDB76B',
        '#FF6347',
        '#E9967A',
        '#F5DEB3',
        '#FFB6C1',
        '#556B2F',
        '#008080',
        '#7FFF00',
        '#FFA500',
        '#FF8C00',
        '#00FF7F',
        '#C0C0C0',
        '#483D8B',
        '#F08080',
        '#D3D3D3',
        '#66CDAA',
        '#FA8072',
        '#F4A460',
        '#48D1CC',
        '#8A2BE2',
        '#2E8B57']

def plot_corridor_list(plane_list):

    if plane_list is None:
        return None
    
    
    i=0
    for segment_plane in plane_list:
    
        i=i+1
        for plane in segment_plane:
            if abs(plane[0]) > abs(plane[1]):
                y1=0
                y2=1
                x1=-plane[2]/plane[0]
                x2=-(plane[1]+plane[2])/plane[0]
            else:
                x1=0
                x2=1
                y1=-plane[2]/plane[1]
                y2=-(plane[0]+plane[2])/plane[1]
            plt.axline([x1,y1],[x2,y2],linestyle='--',color=color[i],linewidth=0.5)
    

def plot_obstacle(obstacle_list,extend=False):

    for ob in obstacle_list:
        X=[]
        Y=[]
        for v in ob.vertex_list:
            X+=[v[0]]
            Y+=[v[1]]
        if not extend:
            plt.fill(X, Y,c='forestgreen')
        else:
            plt.fill(X, Y,c='#D3D3D3')

def plot_grid_map(grid_map,res):

    print(grid_map)
    print(res)

    x_0=SET.map_range['x'][0]
    y_0=SET.map_range['y'][0]

    for i in range(grid_map.shape[0]):
        for j in range(grid_map.shape[1]):
            if grid_map[i][j]==1:
                X=[ x_0+i*res, x_0+(i+1)*res, x_0+(i+1)*res, x_0+i*res ]
                Y=[ y_0+j*res, y_0+j*res, y_0+(j+1)*res, y_0+(j+1)*res ]
                plt.fill(X,Y,c='k')


def plot_convex_plane(points,segment_plane,obstacle_list):

    plt.figure(figsize=(10,5))

    plot_obstacle(obstacle_list)
        
    for plane in segment_plane:
        if abs(plane[0]) > abs(plane[1]):
            y1=0
            y2=1
            x1=-plane[2]/plane[0]
            x2=-(plane[1]+plane[2])/plane[0]
        else:
            x1=0
            x2=1
            y1=-plane[2]/plane[1]
            y2=-(plane[0]+plane[2])/plane[1]
        plt.axline([x1,y1],[x2,y2],linestyle='--',linewidth=0.2)

    plt.scatter(points[0],points[1])

    plt.show()
    plt.close()


def plot_connect(connect_list,agent_list):

    for connect in connect_list:

        p1=agent_list[connect[0]].p
        p2=agent_list[connect[1]].p

        x=[p1[0],p2[0]]
        y=[p1[1],p2[1]]

        plt.plot(x,y,"--",linewidth=3,c='r')



def plot_pre_traj(agent_list,obstacle_list,show,episodes):

    

    if show:
        fig=plt.figure(figsize=SET.plot_range['size'])
        axes=fig.subplots(1,1)
        for i in range(len(agent_list)):

            circle=Circle(xy=agent_list[i].pre_traj[0],radius=SET.ExtendWidth,fc=color[i],ec='k',lw=1.0 )

            axes.add_patch(p=circle)

            plt.plot(agent_list[i].pre_traj[0:2,0],agent_list[i].pre_traj[0:2,1],c=color[i],linewidth=3)
            plt.plot(agent_list[i].pre_traj[1:,0],agent_list[i].pre_traj[1:,1],marker='o',zorder=4,\
            markeredgecolor='k',linewidth=3,markersize=10,c=color[i])


            plt.scatter(agent_list[i].target[0],agent_list[i].target[1],marker='d',s=400,zorder=2,\
                edgecolor='k',color=color[i])
            

            x=[agent_list[i].pre_traj[-1][0],agent_list[i].tractive_point[0]]
            y=[agent_list[i].pre_traj[-1][1],agent_list[i].tractive_point[1]]
            plt.plot(x,y,':',c=color[i],linewidth=4)

            if SET.REALFLY:

                polyx=agent_list[i].input_traj[0]
                polyy=agent_list[i].input_traj[1]
                
                X = SET.K*SET.h*np.arange(100)/100 
                plt.plot( np.polyval(polyx,X), np.polyval(polyy,X),':',c=color[i])


        plot_obstacle(obstacle_list)
        
        plt.xlim(SET.plot_range['x'])
        plt.ylim(SET.plot_range['y'])
        # plt.show()
        plt.savefig('savefig/episode-'+str(episodes)+SET.format, bbox_inches='tight')
        plt.close()

    return None

# plot
# plot
def plot_position(agent_list,obstacle_list):

    fig=plt.figure(figsize=SET.plot_range['size'])

    axes=fig.subplots(1,1)
    
    
    for i in range(SET.Num):
        if agent_list[i].type=="Anchor":
            continue

        # plt.scatter(agent_list[i].position[0][0],agent_list[i].position[0][1],marker='s',s=400,zorder=1,edgecolor='k',color=color[i])
        plt.scatter(agent_list[i].target[0],agent_list[i].target[1],marker='d',s=600,zorder=3,edgecolor='k',color=color[i])

    for j in range(len(agent_list[0].position)):
        for i in range(SET.Num):
            if agent_list[i].type=="Anchor":
                continue
            
            circle=Circle(xy=agent_list[i].position[j],radius=SET.ExtendWidth,fc=color[i],ec='k' )

            axes.add_patch(p=circle)

    plot_obstacle(obstacle_list)
    
    plt.xlim(SET.plot_range['x'])
    plt.ylim(SET.plot_range['y'])
    
    plt.savefig('savefig/trajecotry'+SET.format,bbox_inches='tight')
    # plt.show()
    plt.close()


def plot_path_planning(agent_list):

    plt.figure(figsize=SET.plot_range['size'])
    
    # plot_obstacle(SET.path_plot_obstacle_list,extend=True)

    plot_obstacle(SET.obstacle_list)

    i=0

    for agent in agent_list:
        if agent.type == "Obstacle-transitor" or agent.type == "Free-transitor" or agent.type == "Searcher":
            if agent.path is not None:
                plt.plot(agent.path[:,0],agent.path[:,1], c=color[i])
                plt.xlim(SET.plot_range['x'])
                plt.ylim(SET.plot_range['y'])

            plt.scatter(agent.terminal_p[0],agent.terminal_p[1],marker='o',s=400,zorder=2,edgecolor='k',color=color[i])
            plt.scatter(agent.target[0],agent.target[1],marker='d',s=600,zorder=3,edgecolor='k',color=color[i])
            i=i+1

    plt.savefig('savefig/path'+SET.format,bbox_inches='tight')
    
    # plt.show()
    plt.close()



def plot_circle(obstacle_list,connection_constraint_list,episodes):

    
    plt.figure(figsize=SET.plot_range['size'])


    t=np.block([[np.cos(2*np.pi * np.arange(200)/200),np.sin(2*np.pi * np.arange(200)/200 )]])
    t=t.reshape((200,2),order='F')
    
    for connection_constraint in connection_constraint_list:
        center_list=connection_constraint[0]
        r_list=connection_constraint[1]

        for i in range(len(center_list)):
            x_c=center_list[i]+r_list[i]*t
            plt.plot(x_c[:,0],x_c[:,1])


    plot_obstacle(obstacle_list)
    # plot_corridor_list(agent_list[0].ob_corridor_list)
    
    
    plt.xlim(SET.plot_range['x'])
    plt.ylim(SET.plot_range['y'])
    # plt.show()
    plt.savefig('savefig/e'+str(episodes)+SET.format)
    plt.close()

    return None
