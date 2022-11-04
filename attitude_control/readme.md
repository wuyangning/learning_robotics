
###
- CoppeliaSim版本：V4.4.0 EDU
- 默认的仿真步长被修改为：0.02s
- python控制程序基于CoppeliaSim提供的zmqRemoteApi实现：https://github.com/CoppeliaRobotics/zmqRemoteApi
- 控制程序依赖：numpy
- 姿态的运动学控制是在底层实现了一个PD控制，貌似API本身不支持直接设定刚体的角速度

###
- rotation.py为不同旋转表示法的相关计算。具体可参考下文：
    [姿态表示方法--旋转矩阵、欧拉角、四元数和李代数](https://wuyangning.top/2022/10/29/y2022m11/%E5%A7%BF%E6%80%81%E8%A1%A8%E7%A4%BA%E6%96%B9%E6%B3%95/)
    - 1 旋转矩阵
        - 1.1 绕x,y,z轴的旋转
        - 1.2 旋转矩阵的导数
    - 2 欧拉角
        - 2.1 由旋转矩阵计算欧拉角
        - 2.2 欧拉角的导数
    - 3 四元数
        - 3.1 四元数的运算
        - 3.2 四元数旋转变换
        - 3.3 四元数、旋转矩阵和欧拉角
        - 3.4 四元数的导数
    - 4 李群李代数
        - 4.1 $SO(3)$和$SE(3)$群
        - 4.2 $\mathfrak{so}(3)$和$\mathfrak{se}(3)$李代数
        - 4.3 指数映射和对数映射
    - 5 总结和比较
        - 5.1 旋转表示的目标
        - 5.1 旋转表示法的物理含义
        - 5.3 旋转表示法的比较
###
- main.py中实现了不同姿态表示的运动学控制器和动力学控制器。运动学和动力学控制给出了收敛性分析，动力学控制基于反步控制设计方法，具体可参考下文：
    [不同姿态表示法的运动学和动力控制](https://wuyangning.top/2022/11/03/y2022m11/%E5%A7%BF%E6%80%81%E6%8E%A7%E5%88%B6/)
    - 1 姿态运动学控制
        - 1.1 基于欧拉角的运动学控制
        - 1.2 基于四元数的运动学控制
        - 1.3 基于$SO(3)$的运动学控制
    - 2 姿态动力学控制
        - 2.1 基于欧拉角的控制
        - 2.2 基于四元数的控制
        - 2.3 基于$SO(3)$控制

### 
- 仿真的视频链接：

###
> **参考文献**：
[1] Lynch K M, Park F C. Modern robotics[M]. Cambridge University Press, 2017. 

[2] Chaturvedi N A, Sanyal A K, McClamroch N H. Rigid-body attitude control[J]. IEEE control systems magazine, 2011, 31(3): 30-51.

[3] https://github.com/Krasjet/quaternion

[4] 高翔，张涛，"视觉SLAM十四讲"[M]. 电子工业出版社，2017.


