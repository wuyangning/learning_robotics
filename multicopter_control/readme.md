### 
- CoppeliaSim版本：V4.4.0 EDU
- 默认的仿真步长被修改为：0.02s
- python控制程序基于CoppeliaSim提供的zmqRemoteApi实现，依赖numpy
- 位置控制器采用反馈线性化PD控制(带前馈的PD)
- 姿态控制器采用几何反步控制，可参考[刚体姿态控制](https://wuyangning.top/2022/11/03/y2022m11/%E5%A7%BF%E6%80%81%E6%8E%A7%E5%88%B6/)
- 仿真的无人机类型有
    - 正X型四旋翼(Quadcopter)
    - 倾转三旋翼(Tricopter)
    - 倾转双旋翼(Bicopter)
    - 矢量共轴双旋翼(Singlecopter)
    - 全驱动倾转三旋翼
    - 全驱动六旋翼。


### 
- uav_controller.py中实现了不同构型的无人机控制，可参考：[不同构型的旋翼无人机控制](https://wuyangning.top/2023/01/27/y2023m01/%E4%B8%8D%E5%90%8C%E6%9E%84%E5%9E%8B%E6%97%8B%E7%BF%BC%E6%97%A0%E4%BA%BA%E6%9C%BA%E7%9A%84%E6%8E%A7%E5%88%B6/)
- base_controller.py中实现了刚体的姿态和位置控制器，使用方法如下：

**姿态控制器**
```python
from base_controller import *
import numpy as np
import time


dt=0.02 #控制步长
a_ctrl=atti_ctrl(dt)

#调整控制增益
a_ctrl.k1=np.diag([1.,1.,1.])
a_ctrl.k2=np.diag([4.,4.,4.])
a_ctrl.ki=np.diag([2.,2.,2.])

end=time.perf_counter()
while True
    if time.perf_counter()-end>dt:

        #给定期望姿态轨迹
        a_ctrl.Rd=np.diag([1,1,1]) #期望姿态矩阵
        a_ctrl.dRd=np.diag([0,0,0]) #期望姿态矩阵的时间导数

        #从传感器获取姿态和角速度
        a_ctrl.R=IMU() #期望姿态
        a_ctrl.w=Gyro()# 机体角速度

        #计算控制器
        tau=a_ctrl.compute()

        end=time.perf_counter()  

```
**位置控制器**
```python
from base_controller import *
import numpy as np
import time


dt=0.02 #控制步长
p_ctrl=posi_ctrl(dt)

#调整控制增益
p_ctrl.kp=np.diag([1.,1.,1.])
p_ctrl.kd=np.diag([4.,4.,4.])
p_ctrl.ki=np.diag([2.,2.,2.])

end=time.perf_counter()
while True
    if time.perf_counter()-end>dt:

        #给定期望位置轨迹
        p_ctrl.pd=np.array([0,0,0]) #期望位置
        p_ctrl.dpd=np.array[0,0,0]) #期望速度
        p_ctrl.ddpd=np.array[0,0,0]) #期望加速度

        #从感器获取位置和速度
        p_ctrl.p=Positin() # 绝对位置
        p_ctrl.v=Velocity()# 绝对速度

        #计算控制器
        f=p_ctrl.compute()

        end=time.perf_counter()  

```
