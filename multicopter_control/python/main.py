import numpy as np
from uav_controller import*

try:
    from zmqRemoteApi import RemoteAPIClient
except:
    print("Can not find zmqRemoteApi lib")
                                                

client = RemoteAPIClient()
sim = client.getObject('sim')

UAV=sim.getObject('./UAV')





dt=0.02 #控制步长，需要与仿真中的步长保持一致
quad=quadcopter(dt)
tri=tricopter(dt)
bi=bicopter(dt)
single=singlecopter(dt)
omni_tri=omni_tricopter(dt)
omni_hex=omni_hexcopter(dt)

def run_quadcoper(t):
    # 设定无人机期望轨迹
    h=1
    a=0.1*np.pi
    b=0.6*np.pi
    pd=np.array([h*np.sin(b*t)*np.cos(a*t),
                h*np.sin(b*t)*np.sin(a*t),
                h*np.cos(b*t)+2]) #球面螺旋形
    dpd=np.array([(h*np.sin(b*(t+dt))*np.cos(a*(t+dt))-h*np.sin(b*t)*np.cos(a*t))/dt,
                    (h*np.sin(b*(t+dt))*np.sin(a*(t+dt))-h*np.sin(b*t)*np.sin(a*t))/dt,
                    (h*np.cos(b*(t+dt))-h*np.cos(b*t))/dt])
    ddpd=0*np.array([(h*np.sin(b*(t+2*dt))*np.cos(a*(t+2*dt))-2*h*np.sin(b*(t+dt))*np.cos(a*(t+dt))+h*np.sin(b*t)*np.cos(a*t))/(dt*dt),
                    (h*np.sin(b*(t+2*dt))*np.sin(a*(t+2*dt))-2*h*np.sin(b*(t+dt))*np.sin(a*(t+dt))+h*np.sin(b*t)*np.sin(a*t))/(dt*dt),
                        (h*np.cos(b*(t+2*dt))-2*h*np.cos(b*(t+dt))+h*np.cos(b*t))/(dt*dt)]) 
    yawd=t

    quad.set_target(pd,dpd,ddpd,yawd)

        
    #获取无人机状态
    p=np.array(sim.getObjectPosition(UAV,-1))
    v,w=sim.getObjectVelocity(UAV)
    v=np.array(v)
    w=np.array(w)
    R=np.array(sim.getObjectMatrix(UAV,-1)).reshape(3,4)[0:3,0:3]

    quad.set_state(p,v,R,w)


    fi=quad.run()

    sim.setFloatSignal('f1',fi[0])
    sim.setFloatSignal('f2',fi[1])
    sim.setFloatSignal('f3',fi[2])
    sim.setFloatSignal('f4',fi[3])

def run_tricoper(t):
    # 设定无人机期望轨迹
    pd=np.array([np.cos(0.25*t),np.sin(0.25*t),0.5*np.cos(t)+1.5])
    dpd=np.array([-0.25*np.sin(0.25*t),0.25*np.cos(0.25*t),-0.5*np.sin(t)])
    ddpd=np.array([-0.0625*np.cos(0.25*t),-0.0625*np.sin(0.25*t),-0.5*np.cos(t)]) 
    yawd=0.5*t

    tri.set_target(pd,dpd,ddpd,yawd)

        
    #获取无人机状态
    p=np.array(sim.getObjectPosition(UAV,-1))
    v,w=sim.getObjectVelocity(UAV)
    v=np.array(v)
    w=np.array(w)
    R=np.array(sim.getObjectMatrix(UAV,-1)).reshape(3,4)[0:3,0:3]

    tri.set_state(p,v,R,w)


    uc=tri.run()

    sim.setFloatSignal('f1',uc[0])
    sim.setFloatSignal('f2',uc[1])
    sim.setFloatSignal('f3',uc[2])
    sim.setFloatSignal('theta',uc[3])

def run_bicoper(t):

    pd=np.array([np.cos(0.25*t),np.sin(0.25*t),0.5*np.cos(t)+1.5])
    dpd=np.array([-0.25*np.sin(0.25*t),0.25*np.cos(0.25*t),-0.5*np.sin(t)])
    ddpd=np.array([-0.0625*np.cos(0.25*t),-0.0625*np.sin(0.25*t),-0.5*np.cos(t)]) 
    yawd=0.5*t

    bi.set_target(pd,dpd,ddpd,yawd)

        
    #获取无人机状态
    p=np.array(sim.getObjectPosition(UAV,-1))
    v,w=sim.getObjectVelocity(UAV)
    v=np.array(v)
    w=np.array(w)
    R=np.array(sim.getObjectMatrix(UAV,-1)).reshape(3,4)[0:3,0:3]

    bi.set_state(p,v,R,w)

    uc=bi.run()

    sim.setFloatSignal('f1',uc[0])
    sim.setFloatSignal('f2',uc[1])
    sim.setFloatSignal('theta1',uc[2])
    sim.setFloatSignal('theta2',uc[3])


def run_singlecoper(t):

    pd=np.array([np.cos(0.5*t),np.sin(0.5*t),0.5*np.cos(t)+1.5])
    dpd=np.array([-0.5*np.sin(0.5*t),0.5*np.cos(0.5*t),-0.5*np.sin(t)])
    ddpd=np.array([-0.25*np.cos(0.5*t),-0.25*np.sin(0.5*t),-0.5*np.cos(t)]) 
    yawd=0



    single.set_target(pd,dpd,ddpd,yawd)

        
    #获取无人机状态
    p=np.array(sim.getObjectPosition(UAV,-1))
    v,w=sim.getObjectVelocity(UAV)
    v=np.array(v)
    w=np.array(w)
    R=np.array(sim.getObjectMatrix(UAV,-1)).reshape(3,4)[0:3,0:3]

    single.set_state(p,v,R,w)

    uc=single.run()

    sim.setFloatSignal('f1',uc[0])
    sim.setFloatSignal('f2',uc[1])
    sim.setFloatSignal('theta1',uc[2])
    sim.setFloatSignal('theta2',uc[3])


def run_omni_tricoper(t):
    pd=np.array([np.cos(0.2*t),np.sin(0.2*t),1.5])
    dpd=np.array([-0.2*np.sin(0.2*t),0.2*np.cos(0.2*t),-0.0*np.sin(t)])
    ddpd=np.array([-0.04*np.cos(0.2*t),-0.04*np.sin(0.2*t),-0.0*np.cos(t)])

    Rd=rot.euler_to_rot([0.3*np.sin(0.2*t),0.3*np.cos(0.2*t),0])
    dRd=(rot.euler_to_rot([0.3*np.sin(0.2*(t+dt)),0.3*np.cos(0.2*(t+dt)),0])
        -rot.euler_to_rot([0.3*np.sin(0.2*t),0.3*np.cos(0.2*t),0]))/dt

    omni_tri.set_target(pd,dpd,ddpd,Rd,dRd)

        
    #获取无人机状态
    p=np.array(sim.getObjectPosition(UAV,-1))
    v,w=sim.getObjectVelocity(UAV)
    v=np.array(v)
    w=np.array(w)
    R=np.array(sim.getObjectMatrix(UAV,-1)).reshape(3,4)[0:3,0:3]

    omni_tri.set_state(p,v,R,w)

    uc=omni_tri.run()

    sim.setFloatSignal('f1',uc[0])
    sim.setFloatSignal('f2',uc[1])
    sim.setFloatSignal('f3',uc[2])
    sim.setFloatSignal('theta1',uc[3])
    sim.setFloatSignal('theta2',uc[4])
    sim.setFloatSignal('theta3',uc[5])

def run_omni_hexcoper(t):


    pd=np.array([np.cos(0.5*t),np.sin(0.5*t),np.cos(2*t)+1.5])
    dpd=np.array([-0.5*np.sin(0.5*t),0.5*np.cos(0.5*t),-2*np.sin(2*t)])
    ddpd=np.array([-0.25*np.cos(0.5*t),-0.25*np.sin(0.5*t),-4*np.cos(2*t)])

    Rd=rot.euler_to_rot([np.sin(0.5*t),np.cos(0.5*t),0])
    dRd=(rot.euler_to_rot([np.sin(0.5*(t+dt)),np.cos(0.5*(t+dt)),0])
        -rot.euler_to_rot([np.sin(0.5*t),np.cos(0.5*t),0]))/dt

    omni_hex.set_target(pd,dpd,ddpd,Rd,dRd)

        
    #获取无人机状态
    p=np.array(sim.getObjectPosition(UAV,-1))
    v,w=sim.getObjectVelocity(UAV)
    v=np.array(v)
    w=np.array(w)
    R=np.array(sim.getObjectMatrix(UAV,-1)).reshape(3,4)[0:3,0:3]

    omni_hex.set_state(p,v,R,w)

    uc=omni_hex.run()

    sim.setFloatSignal('f1',uc[0])
    sim.setFloatSignal('f2',uc[1])
    sim.setFloatSignal('f3',uc[2])
    sim.setFloatSignal('f4',uc[3])
    sim.setFloatSignal('f5',uc[4])
    sim.setFloatSignal('f6',uc[5])


try:
    print('Simulation started')
    client.setStepping(True)

    sim.startSimulation()

    t=sim.getSimulationTime()

    while True:
        t=sim.getSimulationTime()

        # 取消注释选择需要控制的无人机
        #run_quadcoper(t) #四旋翼
        #run_tricoper(t) #倾转三旋翼
        #run_bicoper(t) #倾转双旋翼
        run_singlecoper(t) #共轴双旋翼
        #run_omni_tricoper(t) #全驱动三旋翼
        #run_omni_hexcoper(t) #全驱动六旋翼

        client.step()
        #print(sim.getSimulationTime()-t)

except KeyboardInterrupt:
    pass

sim.stopSimulation() 
print('Simulation stopped')