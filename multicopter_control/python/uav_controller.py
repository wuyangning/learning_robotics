import numpy as np
from base_controller import*
import rotation as rot

class quadcopter():
    def __init__(self,dt):
        self.dt=dt #控制步长
        self.posi_ctrl=posi_ctrl(dt)
        self.atti_ctrl=atti_ctrl(dt)

        self.k=0.005 #旋翼的力矩升力比
        self.l=0.1 #机臂参数
        #混控矩阵
        self.mix=np.array([[self.l*self.k,self.k,-self.k,self.l],
                      [self.l*self.k,self.k,self.k,-self.l],
                      [self.l*self.k,-self.k,self.k,self.l],
                      [self.l*self.k,-self.k,-self.k,-self.l]])/(4*self.l*self.k)
        #期望偏航
        self.yawd=0.0
        
        pass
    
    def set_target(self,pd,dpd,ddpd,yawd):
        self.posi_ctrl.pd=pd
        self.posi_ctrl.dpd=dpd
        self.posi_ctrl.ddpd=ddpd
        self.yawd=yawd

    def set_state(self,p,v,R,w):
        self.posi_ctrl.p=p
        self.posi_ctrl.v=v
        self.atti_ctrl.R=R
        self.atti_ctrl.w=w

    def run(self):

        #计算位置控制器
        f=self.posi_ctrl.compute()

        #由位置控制器的输入计算期望姿态矩阵
        h=np.array([np.cos(self.yawd),np.sin(self.yawd),0])
        fz=np.linalg.norm(f)
        n3=f/fz
        n2=np.cross(n3,h)
        n2=n2/np.linalg.norm(n2)
        n1=np.cross(n2,n3)

        Rd=np.c_[n1,n2,n3]
        self.atti_ctrl.dRd=(Rd-self.atti_ctrl.Rd)/self.dt
        self.atti_ctrl.Rd=Rd

        tau=self.atti_ctrl.compute()
        u=np.array([fz,tau[0],tau[1],tau[2]])

        fi=self.mix@u

        return fi


class tricopter():
    def __init__(self,dt):
        self.dt=dt #控制步长
        self.posi_ctrl=posi_ctrl(dt)
        self.atti_ctrl=atti_ctrl(dt)

        self.k=0.005 #旋翼的力矩升力比
        self.l=0.2 #机臂参数
        
        #混控矩阵
        self.mix=np.array([[self.l**2+self.k**2,np.sqrt(3)*self.l,-self.l,-self.k],
                           [self.l**2+self.k**2,-np.sqrt(3)*self.l,-self.l,-self.k],
                           [self.l**2-2*self.k**2,0,2*self.l,2*self.k],
                           [-3*self.l*self.k,0,0,3*self.l]])/(3*self.l**2)
 
        #期望偏航
        self.yawd=0.0
        
        #调节控制增益
        self.posi_ctrl.kp=np.diag([3,3,3])
        self.posi_ctrl.kd=np.diag([3,3,3])
        self.posi_ctrl.ki=np.diag([0.1,0.1,0.1])
        
        self.atti_ctrl.k1=np.diag([0.5,0.5,0.01])
        self.atti_ctrl.k2=np.diag([8,8,2])
        self.atti_ctrl.ki=np.diag([0.01,0.01,0.001])
    
    def set_target(self,pd,dpd,ddpd,yawd):
        self.posi_ctrl.pd=pd
        self.posi_ctrl.dpd=dpd
        self.posi_ctrl.ddpd=ddpd
        self.yawd=yawd

    def set_state(self,p,v,R,w):
        self.posi_ctrl.p=p
        self.posi_ctrl.v=v
        self.atti_ctrl.R=R
        self.atti_ctrl.w=w

    def run(self):
        
        #计算位置控制器
        f=self.posi_ctrl.compute()
        

        #由位置控制器的输入计算期望姿态矩阵
        h=np.array([np.cos(self.yawd),np.sin(self.yawd),0])
        fz=np.linalg.norm(f)
        n3=f/fz
        n2=np.cross(n3,h)
        n2=n2/np.linalg.norm(n2)
        n1=np.cross(n2,n3)

        Rd=np.c_[n1,n2,n3]
        self.atti_ctrl.dRd=(Rd-self.atti_ctrl.Rd)/self.dt
        self.atti_ctrl.Rd=Rd

        tau=self.atti_ctrl.compute()
        u=np.array([fz,tau[0],tau[1],tau[2]])
        

        fi=self.mix@u

        f3=np.sqrt(fi[2]**2+fi[3]**2)
        theta=np.arctan2(fi[3],fi[2])

        return np.array([fi[0],fi[1],f3,theta])


class bicopter():
    def __init__(self,dt):
        self.dt=dt #控制步长
        self.posi_ctrl=posi_ctrl(dt)
        self.atti_ctrl=atti_ctrl(dt)

        self.k=0.00 #旋翼的力矩升力比
        self.ly=0.15 #机臂参数
        self.lz=0.15

        #混控矩阵
        x=self.ly**2+self.k**2
        self.mix=0.5*np.array([[1,  self.ly/x,   0,          self.k/x],
                               [0,  self.k/x,    1/self.lz,  -self.ly/x],
                               [1,  -self.ly/x,  0,          -self.k/x],
                               [0,  -self.k/x,   1/self.lz,  self.ly/x]])
 
        #期望偏航
        self.yawd=0.0
        
        #调节控制增益
        self.posi_ctrl.kp=np.diag([2,2,3])
        self.posi_ctrl.kd=np.diag([2,2,3])
        self.posi_ctrl.ki=np.diag([0.,0.0,0.])
        
        self.atti_ctrl.k1=np.diag([0.5,0.05,0.005])
        self.atti_ctrl.k2=np.diag([8,3,2])
        self.atti_ctrl.ki=np.diag([0.00,0.00,0.00])
    
    def set_target(self,pd,dpd,ddpd,yawd):
        self.posi_ctrl.pd=pd
        self.posi_ctrl.dpd=dpd
        self.posi_ctrl.ddpd=ddpd
        self.yawd=yawd

    def set_state(self,p,v,R,w):
        self.posi_ctrl.p=p
        self.posi_ctrl.v=v
        self.atti_ctrl.R=R
        self.atti_ctrl.w=w

    def run(self):
        
        #计算位置控制器
        f=self.posi_ctrl.compute()
        
        #由位置控制器的输入计算期望姿态矩阵
        h=np.array([np.cos(self.yawd),np.sin(self.yawd),0])
        fz=np.linalg.norm(f)
        n3=f/fz
        n2=np.cross(n3,h)
        n2=n2/np.linalg.norm(n2)
        n1=np.cross(n2,n3)

        Rd=np.c_[n1,n2,n3]
        self.atti_ctrl.dRd=(Rd-self.atti_ctrl.Rd)/self.dt
        self.atti_ctrl.Rd=Rd

        tau=self.atti_ctrl.compute()
        u=np.array([fz,tau[0],tau[1],tau[2]])
        

        fi=self.mix@u

        f1=np.sqrt(fi[0]**2+fi[1]**2)
        theta1=np.arctan2(fi[1],fi[0])
        f2=np.sqrt(fi[2]**2+fi[3]**2)
        theta2=np.arctan2(fi[3],fi[2])

        return np.array([f1,f2,theta1,theta2])

class singlecopter():
    def __init__(self,dt):
        self.dt=dt #控制步长
        self.posi_ctrl=posi_ctrl(dt)
        self.atti_ctrl=atti_ctrl(dt)

        self.k=0.005 #旋翼的力矩升力比
        self.l=0.1 #机臂参数

        #混控矩阵
        x=self.l**2+self.k**2
        self.mix=np.array([[0.5,  0,          0,          0.5/self.k],
                               [0,    self.l/x,   -self.k/x,  0],
                               [0.5,  0,          0,          -0.5/self.k],
                               [0,    -self.k/x,  -self.l/x,  0]])
 
        #期望偏航
        self.yawd=0
        
        #调节控制增益
        self.posi_ctrl.kp=np.diag([0.5,0.5,3])
        self.posi_ctrl.kd=np.diag([1,1,3])
        self.posi_ctrl.ki=np.diag([0.00,0.00,0.5])
        
        self.atti_ctrl.k1=np.diag([0.1,0.1,0.005])
        self.atti_ctrl.k2=np.diag([2,2,0.6])
        self.atti_ctrl.ki=np.diag([0.00,0.00,0.0])
    
    def set_target(self,pd,dpd,ddpd,yawd):
        self.posi_ctrl.pd=pd
        self.posi_ctrl.dpd=dpd
        self.posi_ctrl.ddpd=ddpd
        self.yawd=yawd

    def set_state(self,p,v,R,w):
        self.posi_ctrl.p=p
        self.posi_ctrl.v=v
        self.atti_ctrl.R=R
        self.atti_ctrl.w=w

    def run(self):
        
        #计算位置控制器
        f=self.posi_ctrl.compute()
        
        #由位置控制器的输入计算期望姿态矩阵
        h=np.array([np.cos(self.yawd),np.sin(self.yawd),0])
        fz=np.linalg.norm(f)
        n3=f/fz
        n2=np.cross(n3,h)
        n2=n2/np.linalg.norm(n2)
        n1=np.cross(n2,n3)

        Rd=np.c_[n1,n2,n3]
        self.atti_ctrl.dRd=(Rd-self.atti_ctrl.Rd)/self.dt
        self.atti_ctrl.Rd=Rd

        tau=self.atti_ctrl.compute()
        u=np.array([fz,tau[0],tau[1],tau[2]])
        

        fi=self.mix@u

        f1=np.sqrt(fi[0]**2+fi[1]**2)
        theta1=np.arctan2(fi[1],fi[0])
        f2=np.sqrt(fi[2]**2+fi[3]**2)
        theta2=np.arctan2(fi[3],fi[2])


        return np.array([f1,f2,theta1,theta2])


class omni_tricopter():
    def __init__(self,dt):
        self.dt=dt #控制步长
        self.posi_ctrl=posi_ctrl(dt)
        self.atti_ctrl=atti_ctrl(dt)

        self.k=0.005 #旋翼的力矩升力比
        self.l=0.2 #机臂参数
        
        #注意：由于这里的混控关系形式比较复杂，这里给出的矩阵是求逆之前的混控矩阵
        self.pmix=np.array([[0,-0.5*np.sqrt(3),0,-0.5*np.sqrt(3),0,0],
                            [0,0.5,0,-0.5,0,-1],
                            [1,  0,                 1,  0,               1,  0],
                            [0.5*np.sqrt(3)*self.l,  -0.5*np.sqrt(3)*self.k, -0.5*np.sqrt(3)*self.l,  0.5*np.sqrt(3)*self.k, 0, 0],
                            [-0.5*self.l,  0.5*self.k,  -0.5*self.l,  0.5*self.k,  self.l,  -self.k],
                            [self.k,  self.l,  -self.k,  -self.l,  self.k,  self.l]],dtype=float)
 
        
        #调节控制增益
        self.posi_ctrl.kp=np.diag([1,1,1])
        self.posi_ctrl.kd=np.diag([1.5,1.5,1.5])
        self.posi_ctrl.ki=np.diag([0.0,0.0,0.1])
        
        self.atti_ctrl.k1=np.diag([0.2,0.2,0.01])
        self.atti_ctrl.k2=np.diag([12,12,2])
        self.atti_ctrl.ki=np.diag([0.0,0.0,0.0])
    
    def set_target(self,pd,dpd,ddpd, Rd, dRd):
        self.posi_ctrl.pd=pd
        self.posi_ctrl.dpd=dpd
        self.posi_ctrl.ddpd=ddpd
        self.atti_ctrl.Rd=Rd
        self.atti_ctrl.dRd=dRd

    def set_state(self,p,v,R,w):
        self.posi_ctrl.p=p
        self.posi_ctrl.v=v
        self.atti_ctrl.R=R
        self.atti_ctrl.w=w

    def run(self):
        
        #计算位置控制器
        f=self.atti_ctrl.R.T@self.posi_ctrl.compute()
        tau=self.atti_ctrl.compute()
        u=np.array([f[0],f[1],f[2],tau[0],tau[1],tau[2]])
        

        fi=np.linalg.inv(self.pmix)@u

        f1=np.sqrt(fi[0]**2+fi[1]**2)
        theta1=np.arctan2(fi[1],fi[0])
        f2=np.sqrt(fi[2]**2+fi[3]**2)
        theta2=np.arctan2(fi[3],fi[2])
        f3=np.sqrt(fi[4]**2+fi[5]**2)
        theta3=np.arctan2(fi[5],fi[4])


        return np.array([f1,f2,f3,theta1,theta2,theta3])



class omni_hexcopter():
    def __init__(self,dt):
        self.dt=dt #控制步长
        self.posi_ctrl=posi_ctrl(dt)
        self.atti_ctrl=atti_ctrl(dt)

        self.k=0.005 #旋翼的力矩升力比
        self.l=0.2 #机臂参数
        lpk=self.l+self.k
        lnk=self.l-self.k
        #混控矩阵
        self.mix=np.array([[lnk,-np.sqrt(3)*lnk,lpk,1,-np.sqrt(3),-1],
                            [lpk,np.sqrt(3)*lpk,lnk,-1,-np.sqrt(3),1],
                            [-2*lpk,  0,   lnk,  2,  0,  1],
                            [-2*lnk,  0,   lpk,  -2,  0,  -1],
                            [lnk,np.sqrt(3)*lnk,lpk,1,np.sqrt(3),-1],
                            [lpk,-np.sqrt(3)*lpk,lnk,-1,np.sqrt(3),1]],dtype=float)*(np.sqrt(2)/(6*self.l))
 
        
        #调节控制增益
        self.posi_ctrl.kp=np.diag([6,6,6])
        self.posi_ctrl.kd=np.diag([3,3,3])
        self.posi_ctrl.ki=np.diag([1,1,1])
        
        self.atti_ctrl.k1=np.diag([0.5,0.5,0.5])
        self.atti_ctrl.k2=np.diag([12,12,12])
        self.atti_ctrl.ki=np.diag([0.0,0.0,0.0])
    
    def set_target(self,pd,dpd,ddpd, Rd, dRd):
        self.posi_ctrl.pd=pd
        self.posi_ctrl.dpd=dpd
        self.posi_ctrl.ddpd=ddpd
        self.atti_ctrl.Rd=Rd
        self.atti_ctrl.dRd=dRd

    def set_state(self,p,v,R,w):
        self.posi_ctrl.p=p
        self.posi_ctrl.v=v
        self.atti_ctrl.R=R
        self.atti_ctrl.w=w

    def run(self):
        
        #计算位置控制器
        f=self.atti_ctrl.R.T@self.posi_ctrl.compute()
        tau=self.atti_ctrl.compute()
        u=np.array([f[0],f[1],f[2],tau[0],tau[1],tau[2]])
        

        fi=self.mix@u

        return np.array([fi[0],fi[1],fi[2],fi[3],fi[4],fi[5]])