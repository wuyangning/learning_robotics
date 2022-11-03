import numpy as np
import rotation as rot
import time
try:
    from zmqRemoteApi import RemoteAPIClient
except:
    print("Can not find zmqRemoteApi lib")
                                                

client = RemoteAPIClient()
sim = client.getObject('sim')

body_euler=sim.getObject('/body_euler')
target_euler=sim.getObject('/target_euler')

body_quat=sim.getObject('/body_quat')
target_quat=sim.getObject('/target_quat')

body_so3=sim.getObject('/body_so3')
target_so3=sim.getObject('/target_so3')

t=0
dt=0.02
Rd=np.identity(3)
last_Rd=np.identity(3)
wd=np.array([0,0,0],dtype=float)
last_wd=np.array([0,0,0],dtype=float)
dwd=np.array([0,0,0,],dtype=float)

J=0.01*np.identity(3) #转动惯量

def euler_ctrl_kine():
    '''
        从仿真得到的一维数组结构为：

            R          t
        0,  1,  2,     3
        4,  5,  6,     7
        8,  9, 10,    11
        '''
    Rb=np.array(sim.getObjectMatrix(body_euler,-1)).reshape(3,4)[0:3,0:3]

    Phi_d=rot.rot_to_euler(Rd)[0]
    Phi_b=rot.rot_to_euler(Rb)[0]

    Phi_e=Phi_b-Phi_d
    T_inv=np.array([[1,0,-np.sin(Phi_b[1])],
                        [0,np.cos(Phi_b[0]),np.sin(Phi_b[0])*np.cos(Phi_b[1])],
                        [0,-np.sin(Phi_b[0]),np.cos(Phi_b[0])*np.cos(Phi_b[1])]])
    K=1
    w=-K*T_inv@Phi_e

    sim.setFloatSignal('ux_euler',w[0])
    sim.setFloatSignal('uy_euler',w[1])
    sim.setFloatSignal('uz_euler',w[2])

def quat_ctrl_kine():
    qd=rot.rot_to_quat(Rd)
    invqd=np.array([qd[0],-qd[1],-qd[2],-qd[3]])
    q=sim.getObjectQuaternion(body_quat,-1)
    q=np.array([q[3],q[0],q[1],q[2]])
    
    
    qe=rot.qtimes(invqd,q)

    K=2
    w=-np.sign(qe[0])*K*qe[1:]

    sim.setFloatSignal('ux_quat',w[0])
    sim.setFloatSignal('uy_quat',w[1])
    sim.setFloatSignal('uz_quat',w[2])

        
def so3_ctrl_kine():
    Rb=np.array(sim.getObjectMatrix(body_so3,-1)).reshape(3,4)[0:3,0:3]
    Re=Rd.T@Rb

    K=1
    w=-K*rot.so3_vee(Re-Re.T)

    sim.setFloatSignal('ux_so3',w[0])
    sim.setFloatSignal('uy_so3',w[1])
    sim.setFloatSignal('uz_so3',w[2])

    
def euler_ctrl_dyna():    
    
    Phi_d=rot.rot_to_euler(Rd)[0]

    sxd=np.sin(Phi_d[0])
    cxd=np.cos(Phi_d[0])

    syd=np.sin(Phi_d[1])
    cyd=np.cos(Phi_d[1])
    tyd=np.tan(Phi_d[1])
    if cyd != 0:
        secyd=1/cyd
    else:
        secyd=1e8

    Td=np.array([[1,sxd*tyd,cxd*tyd],
                [0,cxd,-syd],
                [0,-sxd*secyd,cxd*secyd]])
    
    dPhi_d=Td@wd
    ddPhi_d=Td@dwd


    Rb=np.array(sim.getObjectMatrix(body_euler,-1)).reshape(3,4)[0:3,0:3]
    Phi_b=rot.rot_to_euler(Rb)[0]
    
    Phi_e=Phi_b-Phi_d
    sx=np.sin(Phi_b[0])
    cx=np.cos(Phi_b[0])

    sy=np.sin(Phi_b[1])
    cy=np.cos(Phi_b[1])
    ty=np.tan(Phi_b[1])
    if cy != 0:
        secy=1/cy
    else:
        secy=1e8

    T=np.array([[1,sx*ty,cx*ty],
                [0,cx,-sy],
                [0,-sx*secy,cx*secy]])
    
    T_inv=np.array([[1,0,-sy],
                    [0,cx,sx*cy],
                    [0,-sx,cx*cy]])
    wb=np.array(sim.getObjectVelocity(body_euler)[1])
    A=np.array([[cx*ty,sx*secy*secy,0],
                [-sx,0,0],
                [cx*secy,sx*secy*ty,0]])
    B=np.array([[-sx*ty,cx*secy*secy,0],
                [-cx,0,0],
                [-sx*secy,cx*secy*ty,0]])
    dT=np.c_[np.array([0,0,0]),np.c_[A@T@wb,B@T@wb]]


    K1=2
    K2=2

    Phi_z=T@wb+K1*Phi_e-dPhi_d
    tau=rot.sk(wb)@J@wb-J@T_inv@(K2*Phi_z+dT@wb \
        +K1*T@wb+Phi_e-K1*dPhi_d-ddPhi_d)
    
    sim.setFloatSignal('ux_euler',tau[0])
    sim.setFloatSignal('uy_euler',tau[1])
    sim.setFloatSignal('uz_euler',tau[2])

def quat_ctrl_dyna():

    qd=rot.rot_to_quat(Rd)
    invqd=np.array([qd[0],-qd[1],-qd[2],-qd[3]])

    q=sim.getObjectQuaternion(body_quat,-1)
    q=np.array([q[3],q[0],q[1],q[2]])
    wb=np.array(sim.getObjectVelocity(body_quat)[1])
    
    qe=rot.qtimes(invqd,q)
    qe=np.sign(qe[0])*qe
    se=qe[0]
    ve=qe[1:]

    dve=0.5*(se*(wb-wd)+rot.sk(ve)@(wb+wd))

    K1=2
    K2=2
    
    vz=wb+2*K1*ve-wd
    tau=rot.sk(wb)@J@wb+J@(-K2*vz-2*K1*dve-0.5*ve+dwd)

    sim.setFloatSignal('ux_quat',tau[0])
    sim.setFloatSignal('uy_quat',tau[1])
    sim.setFloatSignal('uz_quat',tau[2])

def so3_ctrl_dyna():

    #当前姿态、角速度
    Rb=np.array(sim.getObjectMatrix(body_so3,-1)).reshape(3,4)[0:3,0:3]
    wb=np.array(sim.getObjectVelocity(body_so3)[1])

    #误差
    Re=Rd.T@Rb
    phi_e=rot.so3_vee(Re-Re.T)
    dRe=Re@rot.sk(wb)-rot.sk(wd)@Re
    dphi_e=rot.so3_vee(dRe-dRe.T)
    Omega_e=dRe.T@wd+Re.T@dwd

    K1=2
    K2=2
    J=0.01*np.identity(3)

    z=wb+K1*phi_e-Re.T@wd
    tau=rot.sk(wb)@J@wb+J@(-K2*z-phi_e-K1*dphi_e+Omega_e)

    sim.setFloatSignal('ux_so3',tau[0])
    sim.setFloatSignal('uy_so3',tau[1])
    sim.setFloatSignal('uz_so3',tau[2])

  
try:
    print('Simulation started')
    client.setStepping(True)
    sim.startSimulation()

    t=sim.getSimulationTime()

    Rd=rot.euler_to_rot([0.5*np.cos(2*t),2*np.cos(2*t),0.5*np.sin(2*t)])
    #Rd=rot.quat_to_rot([-1,0,0,0])
    last_Rd=Rd.copy()

    #仿真环境中的欧拉角是xyz顺序的，因此转为四元数再发送
    #仿真中的四元数是xyzw顺序
    qd=rot.rot_to_quat(Rd)
    sim.setObjectQuaternion(target_euler,-1,[qd[1],qd[2],qd[3],qd[0]])
    sim.setObjectQuaternion(target_quat,-1,[qd[1],qd[2],qd[3],qd[0]])
    sim.setObjectQuaternion(target_so3,-1,[qd[1],qd[2],qd[3],qd[0]])

    print("- Input 0 for kinematics controller ")
    print("- Input 1 for dynamics tracking controller ")

    ctrl_mode=1
    try:
        ctrl_mode=int(input())
        if ctrl_mode:
            print("Start with dynamics controller")
        else:
            print("Start with kinematics controller")
    except:
        print("Defaut start with dynamics controller")


    while True:
        t=sim.getSimulationTime()

        sim.setInt32Signal('ctrl_mode',ctrl_mode)
        #生成一段旋转轨迹
        Rd=rot.euler_to_rot([0.5*np.cos(2*t),2*np.cos(2*t),0.5*np.sin(2*t)])
        
        dRd=(Rd-last_Rd)/dt

        wd=rot.so3_vee(Rd.T@dRd)
        dwd=(wd-last_wd)/dt

        last_wd=wd.copy()
        last_Rd=Rd.copy()

        qd=rot.rot_to_quat(Rd)
        sim.setObjectQuaternion(target_euler,-1,[qd[1],qd[2],qd[3],qd[0]])
        sim.setObjectQuaternion(target_quat,-1,[qd[1],qd[2],qd[3],qd[0]])
        sim.setObjectQuaternion(target_so3,-1,[qd[1],qd[2],qd[3],qd[0]])

        if ctrl_mode:
        #动力学控制
            euler_ctrl_dyna()
            quat_ctrl_dyna()
            so3_ctrl_dyna()
        else:
            #运动学控制
            euler_ctrl_kine()
            quat_ctrl_kine()
            so3_ctrl_kine()

        client.step()
        #print(sim.getSimulationTime()-t)
            
except KeyboardInterrupt:
    pass

sim.stopSimulation() 
print('Simulation stopped')