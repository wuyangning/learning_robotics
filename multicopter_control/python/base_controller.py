import numpy as np
import rotation as rot

class atti_ctrl:
    def __init__(self,dt) -> None:
        self.dt=dt #控制步长
        self.J=0.1*np.identity(3) #转动惯量
        

        #控制增益
        self.k1=np.diag([4,4,3])
        self.k2=np.diag([10,10,5])
        self.ki=np.diag([0,0,0])

        #期望姿态
        self.Rd=np.identity(3) #期望姿态矩阵
        self.dRd=np.identity(3) #期望姿态矩阵的导数
        self.last_wd=np.array([0,0,0],dtype=float) #上一个时刻的期望姿角速度

        #积分项
        self.hatd=np.array([0,0,0],dtype=float)

        #状态量
        self.R=np.identity(3) #当前姿态矩阵
        self.w=np.array([0,0,0],dtype=float) #当前机体角速度

    def set_target(self,Rd,dRd):
        self.Rd=Rd
        self.dRd=dRd


    def update_state(self,R,w):
        self.R=R
        self.w=w
    

    def compute(self):
        #基于so3和反步技术的姿态控制器

        wd=rot.so3_vee(self.Rd.T@self.dRd)
        dwd=(wd-self.last_wd)/self.dt

        self.last_wd=wd.copy()


        #姿态控制器
        Re=self.Rd.T@self.R
        phi_e=rot.so3_vee(Re-Re.T)
        dRe=Re@rot.sk(self.w)-rot.sk(wd)@Re
        dphi_e=rot.so3_vee(dRe-dRe.T)
        Omega_e=dRe.T@wd#+Re.T@dwd #可根据情况忽略高阶项以减少噪声

        z=self.w+self.k1@phi_e-Re.T@wd
        self.hatd=self.hatd+self.dt*self.ki@z
        tau=np.cross(self.w,self.J@self.w)+self.J@(-self.k2@z-phi_e-self.k1@dphi_e+Omega_e)-self.hatd

        return tau


class posi_ctrl:
    def __init__(self,dt) -> None:
        self.dt=dt #控制步长
        self.M=1 # 无人机质量
        self.ge3=np.array([0,0,9.81],dtype=float) #重力加速度

        #控制增益
        self.kp=np.diag([6,6,6])
        self.kd=np.diag([3.,3.,3.])
        self.ki=np.diag([0,0,0])

        #期望轨迹
        self.pd=np.array([0,0,1],dtype=float) #期望位置轨迹
        self.dpd=np.array([0,0,0],dtype=float) #期望速度轨迹
        self.ddpd=np.array([0,0,0],dtype=float) #期望加速度轨迹

        #积分项
        self.hatd=np.array([0,0,0],dtype=float)

        #状态量
        self.p=np.array([0,0,0],dtype=float) #当前位置
        self.v=np.array([0,0,0],dtype=float) #当前速度
        

    def set_target(self,pd,dpd,ddpd):
        self.pd=pd
        self.dpd=dpd
        self.ddpd=ddpd


    def update_state(self,p,v):
        self.p=p
        self.v=v
    

    def compute(self):

        #反馈线性化+PD控制
        e=self.p-self.pd
        de=self.v-self.dpd
        self.hatd=self.hatd+self.dt*self.ki@e
        Rf=self.M*(-self.kp@e-self.kd@de+self.ge3+self.ddpd)-self.hatd

        return Rf

class pid_ctrl:
    def __init__(self,dt):
        #控制增益
        self.kp=1.0
        self.ki=0.0
        self.kd=0.0
        self.dt=dt

        #积分项
        self.inte=0.0 

    def compute(self,e,de):
        #e=x-xd 误差的定义为状态减期望
        self.inte=self.inte+self.ki*e*self.dt
        u=-self.kp*e-self.kd*de-self.inte
        return u

