import numpy as np

#绕x轴旋转的旋转矩阵
def rot_x(theta):   
    return np.array([[1,0,0],
                     [0,np.cos(theta),-np.sin(theta)],
                     [0,np.sin(theta),np.cos(theta)]],dtype=np.float64)

#绕y轴旋转的旋转矩阵
def rot_y(theta):
    return np.array([[np.cos(theta),0,np.sin(theta)],
                     [0,1,0],
                     [-np.sin(theta),0,np.cos(theta)]],dtype=np.float64)

#绕z轴旋转的旋转矩阵
def rot_z(theta):
    return np.array([[np.cos(theta),-np.sin(theta),0],
                     [np.sin(theta),np.cos(theta),0],
                     [0,0,1]],dtype=np.float64)

#三维向量对应的反对称矩阵
def sk(x):
    return np.array([[0,-x[2],x[1]],
                     [x[2],0,-x[0]],
                     [-x[1],x[0],0]],dtype=np.float64)

#绕机体坐标系zyx顺序旋转的欧拉角对应的旋转矩阵
#Phi=[phi:x,theta:y,psi:z]
def euler_to_rot(Phi):
    return rot_z(Phi[2])@rot_y(Phi[1])@rot_x(Phi[0])

#已知旋转矩阵求绕机体坐标系zyx顺序旋转的欧拉角
def rot_to_euler(R):
    #返回结果是欧拉角的两组解
    #Phi=[phi:x,theta:y,psi:z]
    Phi1=np.array([0,0,0],dtype=np.float64)
    Phi2=np.array([0,0,0],dtype=np.float64)

    if abs(R[2,0])==1:
        sc=np.sign(R[2,0])

        #奇异点，psi可为任意值，有无穷多解，这里设成0和pi/2
        Phi1[2]=0
        Phi2[2]=np.pi/2

        Phi1[1]=sc*np.pi/2
        Phi2[1]=sc*np.pi/2

        Phi1[0]=sc*Phi1[2]+np.arctan2(sc*R[0,1],sc*R[0,2])
        Phi2[0]=sc*Phi2[2]+np.arctan2(sc*R[0,1],sc*R[0,2])
    else:
        Phi1[1]=-np.arcsin(R[2,0])
        Phi2[1]=np.pi-Phi1[1]

        Phi1[0]=np.arctan2(R[2,1]/np.cos(Phi1[1]),R[2,2]/np.cos(Phi1[1]))
        Phi2[0]=np.arctan2(R[2,1]/np.cos(Phi2[1]),R[2,2]/np.cos(Phi2[1]))

        Phi1[2]=np.arctan2(R[1,0]/np.cos(Phi1[1]),R[0,0]/np.cos(Phi1[1]))
        Phi2[2]=np.arctan2(R[1,0]/np.cos(Phi2[1]),R[0,0]/np.cos(Phi2[1]))
        
    return Phi1,Phi2

#四元数乘法
def qtimes(q1,q2):
    s1=q1[0]
    v1=q1[1:]
    Q=np.r_[np.array([np.r_[s1,-v1]]),np.c_[v1,s1*np.identity(3)+sk(v1)]]
    return Q@q2

#四元数对应的旋转矩阵
def quat_to_rot(q):
    q=np.array(q)
    v=q[1:].reshape(3,1)
    A=q[0]*np.identity(3)+sk(q[1:])
    return v@v.T+A@A

#旋转矩阵转四元数
def rot_to_quat(R):
    q=np.array([1,0,0,0],dtype=float)
    q[0]=np.sqrt(np.abs(R[0,0]+R[1,1]+R[2,2]+1))/2
    q[1]=np.sign(R[2,1]-R[1,2])*np.sqrt(np.abs(R[0,0]-R[1,1]-R[2,2]+1))/2
    q[2]=np.sign(R[0,2]-R[2,0])*np.sqrt(np.abs(-R[0,0]+R[1,1]-R[2,2]+1))/2
    q[3]=np.sign(R[1,0]-R[0,1])*np.sqrt(np.abs(-R[0,0]-R[1,1]+R[2,2]+1))/2
    return q

#绕机体坐标系zyx顺序旋转的欧拉角对应的四元数
def euler_to_quat1(Phi):
    qz=np.array([np.cos(Phi[2]/2),0,0,np.sin(Phi[2]/2)],dtype=np.float64)
    qy=np.array([np.cos(Phi[1]/2),0,np.sin(Phi[1]/2),0],dtype=np.float64)
    qx=np.array([np.cos(Phi[0]/2),np.sin(Phi[0]/2),0,0],dtype=np.float64)
    return qtimes(qtimes(qz,qy),qx)

#欧拉角转四元数的展开形式
def euler_to_quat(Phi):
    cx=np.cos(Phi[0]/2)
    sx=np.sin(Phi[0]/2)
    cy=np.cos(Phi[1]/2)
    sy=np.sin(Phi[1]/2)
    cz=np.cos(Phi[2]/2)
    sz=np.sin(Phi[2]/2)

    return np.array([cx*cy*cz+sx*sy*sz,
                     sx*cy*cz-cx*sy*sz,
                     cx*sy*cz+sx*cy*sz,
                     cx*cy*sz-sx*sy*cz],dtype=np.float64)



#so(3)向量到反对称矩阵的映射
def so3_wedge(x):
    return np.array([[0,-x[2],x[1]],
                     [x[2],0,-x[0]],
                     [-x[1],x[0],0]],dtype=np.float64)
#反对称矩阵到so(3)向量的映射
def so3_vee(X):
    return np.array([(X[2,1]-X[1,2])/2,(X[0,2]-X[2,0])/2,(X[1,0]-X[0,1])/2],dtype=np.float64)

#SO(3)空间的指数映射
#so(3)向量对应的旋转矩阵
def so3_exp_map(phi):
    theta=np.linalg.norm(phi)
    a=np.array(phi)/theta
    ac=a.reshape(3,1)
    return np.cos(theta)*np.identity(3)+(1-np.cos(theta))*ac@ac.T+np.sin(theta)*so3_wedge(a)

#SO(3)空间的对数映射
#旋转矩阵对应的so(3)向量
def so3_ln_map(R):
    theta=np.arccos((np.trace(R)-1)*0.5)
    a=(0.5/np.sin(theta))*so3_vee(R-R.T)
    return theta*a

#se(3)向量到R^4*4矩阵
def se3_wedge(xi):
    rho=np.array(xi[0:3])
    X=so3_wedge(xi[3:])
    return np.r_[np.c_[X,rho],np.array([[0,0,0,0]])]

#R^4*4矩阵到se(3)向量
def se3_vee(T):
    x=so3_vee(T[0:3,0:3])
    rho=T[0:3,3:].reshape(1,3)[0]
    return np.r_[rho,x]

def se3_exp_map(xi):
    rho=np.array(xi[0:3])
    theta=np.linalg.norm(xi[3:])
    a=np.array(xi[3:])/theta
    aa_T=a.reshape(3,1)@a.reshape(3,1).T
    wedge_a=so3_wedge(a)
    exp_phi=np.cos(theta)*np.identity(3)+(1-np.cos(theta))*aa_T+np.sin(theta)*wedge_a
    J=(np.sin(theta)/theta)*np.identity(3)+(1-np.sin(theta)/theta)*aa_T+((1-np.cos(theta))/theta)*wedge_a
    return np.r_[np.c_[exp_phi,J@rho],np.array([[0,0,0,1]])]

def se3_ln_map(T):
    R=T[0:3,0:3]
    t=T[0:3,3:].reshape(1,3)[0]
    theta=np.arccos((np.trace(R)-1)*0.5)
    a=(0.5/np.sin(theta))*so3_vee(R-R.T)
    ac=a.reshape(3,1)
    J=(np.sin(theta)/theta)*np.identity(3)+(1-np.sin(theta)/theta)*ac@ac.T+((1-np.cos(theta))/theta)*so3_wedge(a)
    return np.r_[np.linalg.inv(J)@t,theta*a]


