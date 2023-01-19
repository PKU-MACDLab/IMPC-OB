import numpy as np
import time
import SET


def begin_trajectory(item):
    
    agent_list,interval=item
    
    # 输入轨迹
    start = time.time()

    Ployx=[]
    Ployy=[]
    Ployz=[]

    
    omega=np.zeros(3)


    for agent in agent_list:
        Ployx+=[agent.input_traj[0]]
        Ployy+=[agent.input_traj[1]]
        Ployz+=[agent.input_traj[2]]

 
    while True:

        runtime=time.time()-start
        if runtime > interval:
            break
        t = time.time()-start + 0.0
        for i in range(len(agent_list)):

            if agent_list[i].type=="Anchor":
                continue
            
            x=np.polyval(Ployx[i],t)
            y=np.polyval(Ployy[i],t)
            z=np.polyval(Ployz[i],t)
            
            vx = np.polyval(np.polyder(Ployx[i]), t) 
            vy = np.polyval(np.polyder(Ployy[i]), t) 
            vz = np.polyval(np.polyder(Ployz[i]), t) 

            ax = np.polyval(np.polyder(Ployx[i], 2), t) * 0.5
            ay = np.polyval(np.polyder(Ployy[i], 2), t) * 0.5
            az = np.polyval(np.polyder(Ployz[i], 2), t) * 0.5

            jx = np.polyval(np.polyder(Ployx[i], 3), t)
            jy = np.polyval(np.polyder(Ployy[i], 3), t)

            omega[0] = jy / 9.8 *0.0
            omega[1] = - jx / 9.8 *0.0

            pos=np.array([x,y,z])
            vel = np.array([vx,vy,vz])
            acc = np.array([ax,ay,az])

            # SET.swarm.allcfs.crazyflies[i].cmdFullState(pos, vel, acc, yaw, omega)

        runtime=time.time()-start
        if runtime > interval-0.01:
            break
        time.sleep(0.02)
    
    return None

def land(targetHeight, duration):

    pos=np.zeros(3)
    vel=np.zeros(3)
    acc=np.zeros(3)
    yaw=0.0
    omega=np.zeros(3)

    pos_list=[]

    for cf in SET.swarm.allcfs.crazyflies:
        pos_list+=[cf.position()]
    
    for i in range(10):
        j=0
        
        for cf in SET.swarm.allcfs.crazyflies:
            pos[0:2]=pos_list[j][0:2]
            pos[2]=pos_list[j][2]*(10-i)/10+targetHeight
            cf.cmdFullState(pos, vel, acc, yaw, omega)
            j+=1
        SET.swarm.timeHelper.sleep(duration/10)
    
    for cf in SET.swarm.allcfs.crazyflies:
        cf.cmdStop()