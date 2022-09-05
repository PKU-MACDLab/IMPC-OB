import      SET
from        run         import *
from        others      import *
from        uav         import *
from        plot        import * 
import      numpy       as     np
import shutil
import os
import multiprocessing as mp


def initialize():

    # inilization 
    agent_list=[]
    for i in range(SET.Num):
        agent_list+=[ uav2D(i,SET.ini_x[i],SET.target[i],SET.type_list[i],SET.K) ]


    return agent_list



def main():

    # buld the file that saving the data and figure
    if os.path.exists('savefig'):
        shutil.rmtree('savefig')
    if os.path.exists('data'):
        shutil.rmtree('data')
    
    os.mkdir('savefig')
    os.mkdir('data')

    # the initialization this program
    SET.initialize_set()

    # initialize the agnets
    agent_list=initialize()

    plot_path_planning(agent_list)

    # mp.set_start_method("forkserver")
    # mp.set_start_method("fork")

    all_time=0.0

    # begin the main loop
    for i in range(1,SET.episodes+1):

        start=time.time()

        print('==============================================')
        # run one step 
        agent_list = run_one_step(agent_list,SET.obstacle_list)
        
        time_interval=time.time()-start
        
        # print running time in this step
        print("Step %s have finished, running time is %s"%(i,time_interval))
        
        print(" ")
        all_time+=time_interval
        # plot the predetermined tarjectories in this time step
        plot_pre_traj(agent_list,SET.ini_obstacle_list,SET.show,i)
    
        # juding whether all robots reach to their target positions
        if check_reach_target(agent_list):
            break 
        
    print('the mean time per replanning is:'+str(all_time/i))
  
    # save the running data
    save_data(agent_list)

    print('The data has been saved at /data/data.csv')
    print('Figures has been saved in /savefig')

    # plot trajectories in this test
    plot_position(agent_list,SET.ini_obstacle_list)




if __name__ == '__main__':

    main()