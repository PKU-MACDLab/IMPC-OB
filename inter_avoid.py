import numpy as np

# this code is appropirate for both 2D and 3D

def Get_inter_cons(agent,share_data):
    
    P=agent.pre_traj 
    target=agent.target 
    K=agent.K 
    D=agent.D
    eta=agent.eta
    r_min=agent.r_min
    index=agent.index
    term_overlap=agent.term_overlap
    p=agent.p
    rho_0=agent.rho_0

    cons_A=np.zeros((1,D*K))
    cons_B=np.array([-1.0])
    cons_C=np.zeros((1,1))
    RHO=np.array([0.0])

    type_list=share_data['type']
    pre_traj_list=share_data['pre_traj']

    for j in range(0,len(pre_traj_list)): # j is the index of other agent

        if j==index: 
            continue
        if type_list[j] == 'Anchor':
            continue

        P_j=pre_traj_list[j]

        # l_max=2*agent.Vmax*agent.h*agent.K + agent.r_min + 2*agent.epsilon

        # # the agent out of rang is negelected
        # if np.linalg.norm(p-P_j[0]) > l_max:
        #     continue

        for t in range(1,len(P)):
            p=P[t]
            l=len(P_j)
            if(t>=l):
                p_j=P_j[-1]
            else:
                p_j=P_j[t]

            if(t==len(P)-1):
                a,b,rho = MBVC_WB(p,p_j,target,r_min,eta,rho_0,term_overlap)
            else:
                a,b = MBVC(p,p_j,r_min)
            
            # add constraints 
            cons_a=[]
            for i in range(0,len(P)-1):
                if(i==t-1):
                    cons_a=np.append(cons_a,a) 
                else:
                    cons_a=np.append(cons_a,np.zeros(D))
            
            cons_A=np.row_stack((cons_A,cons_a))
            cons_B=np.append(cons_B,b)
            cons_C=np.row_stack(( cons_C , np.zeros(cons_C.shape[1]) ))

            if(t==len(P)-1):
                cons_c=np.zeros((len(cons_B),1))
                cons_c[-1]=1.0
                cons_C=np.column_stack((cons_C,cons_c))
            
                RHO=np.append(RHO,rho)

     
    return [cons_A,cons_B,cons_C,RHO]



def MBVC(agent_i,agent_j,r_min):

    p=(agent_i+agent_j)/2
    a=(agent_i-agent_j)/np.linalg.norm(agent_i-agent_j)

    b=a @ p + r_min/2

    return a,b



def MBVC_WB(agent_i,agent_j,target,r_min,eta,rho_0,term_overlap):

    p=(agent_i+agent_j)/2
    a=(agent_i-agent_j)/np.linalg.norm(agent_i-agent_j)

    b=a @ p + r_min/2
    n_j=np.zeros(2)
    n_target=np.zeros(2)

    for i in range(2):
        n_j[i]=(agent_j-agent_i)[i]
        n_target[i]=(target-agent_i)[i]

    if np.linalg.norm(n_j) > 1e-10 and np.linalg.norm(n_target) > 1e-10:

        n_j=n_j/np.linalg.norm(n_j)

        
        n_target=n_target/np.linalg.norm(n_target)

        
        if(term_overlap):
            rho_i_j=rho_0*2.71828**(eta*(n_j[1]*n_target[0]-n_j[0]*n_target[1]))
        else:
            rho_i_j=rho_0
            
    else:
        rho_i_j=rho_0

    return a,b,rho_i_j