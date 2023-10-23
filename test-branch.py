from Robot import Robot
import numpy as np
import until
import time
import pandas as pd
import math
from scipy.interpolate import CubicSpline
import sys
# Write the path to the IK folder here
# For example, the IK folder is in your Documents folder
import getpass

sys.path.append(f"C:\\Users\\Fanyi\\Desktop\\coppy")
import ikt

#笛卡尔空间直线规划仿真
def inv_based_traj_planning(start,end,t,time):
    if t < time:
        tMatrix = np.matrix([
        [         0,           0,             0,          0,        0,   1],
        [   time**5,     time**4,       time**3,    time**2,     time,   1],
        [         0,           0,             0,          0,        1,   0],
        [ 5*time**4,   4*time**3,     3*time**2,     2*time,        1,   0],
        [         0,           0,             0,          2,        0,   0],
        [20*time**3,  12*time**2,        6*time,          2,        0,   0]
        ])
        
        xArray = []
        for i in range(len(start)):
            xArray.append([start[i], end[i], 0, 0, 0, 0])
        xMatrix = np.matrix(xArray).T
        
        kMatrix = tMatrix.I * xMatrix
        
        timeVector = np.matrix([t**5, t**4, t**3, t**2, t, 1]).T
        x = (kMatrix.T * timeVector).T.A[0]
        
        
    else:
        x = end
    return x

def planning(start, end, time, interval, middle=None, method="NORM"):
    if method is "INV":
        for i in range(time//interval):
            yield inv_based_traj_planning(start, end, interval * i, time)
    elif method is "NORM":
        if middle is not None:
            pass
        else:
            
        raise RuntimeError("TBD") # TODO: FIXME

#三次样条插值
def CubSp_traj_planning(time_array,pose_array,sample_time):
    
    data = np.vstack(pose_array)
    num_time_points = data.shape[0]

    
    splines = [CubicSpline(time_array, data[:, i], bc_type=((1, 0.0), (1, 0.0)) ) for i in range(6)]

    q = [s(sample_time) for s in splines]
    return np.array(q)

def moveTraj(robot: Robot, traj, interval):
    for pos in traj:
        start = time.time()
        robot.syncMove(pos)
        end = time.time()
        spend_time = end - start
        if spend_time < interval:
            time.sleep(interval - spend_time)
        else:
            print("timeout!")


def test():
    r = Robot(com='COM4', baud=250000) 
    # 连接到真实机器人
    r.connect()
    # 控制周期（推荐周期，最好不要改）
    T = 0.02
    # 初始化时间
    
    t = 0
    # 对应三点的关节值
    qA =  np.array([-0.73841174, 0.73410972, 1.65482929, -0.84614813, -0.03075535, -0.06804614])/math.pi*180  
    qB =  np.array([-0.08845569, 0.11414149, 1.54840006, -0.5902079, 0.04097188,-0.07763637])/math.pi*180  
    qC =  np.array([0.73568646, 0.29145575, 1.61017211, -0.2354593 , -0.10481398,  1.51148932])/math.pi*180     
    # 规划代码可以写这里，提前规划好曲线，这里只是匀速规划曲线（效果肯定是不行的）
    # 规划的从零位到A点的时间，以2秒为例
    
    tOA = 2
    # 规划曲线为匀速曲线,仅仅用于从机械臂的零位到A点
    v1 = (qA-0)/tOA
    # A到B点时间
    tAB = 1
    # A到C点时间
    tAC = 2
    # 过B点速度,单位是（度/秒）
    midVel = 15
    # 规划A点到C点经过B点的曲线，quinticCurvePlanning2见源代码
    k = until.quinticCurvePlanning2(qA,qB,qC,midVel,tAB,tAC)
    # 规划完成

    #开始控制机械臂运动
    # 使用该函数可以使机械臂回到零位
    
    # 开始控制机器人
    while(1):
        start = time.time()
        # 重新开始一次循环
        if t >= tOA + tAC:
            print('Control Finished')
            break
        # 通过时间与规划目标关节位置的关系，得到挡墙时刻下期望机械臂关节到达的位置
        elif t < tOA:
            q = ikt.inv_kinematics(q_int_A)
        elif t <tOA + tAC:
            #q = until.quinticCurveExcute2(k,t-tOA)
            position = inv_based_traj_planning([0.370, -0.090, 0.155],[0.288, -0.288, 0.155],t-tOA,tAC)
            pose = [0, 3.1415,0]
            p_p = np.hstack((position,pose))
            q = ikt.inv_kinematics(p_p)[0,:]
            print(p_p)
            # 控制机械臂运动，syncMove输入格式为1*6的np.array，单位为度，代表的含义是当前周期下机械臂关节的位置
        
        r.syncMove(q)
        # 更新时间
        t = t + T
        # 定时器操作
        end = time.time()
        spend_time = end - start
        if spend_time < T:
            time.sleep(T - spend_time)
        else:
            print("timeout!")



if __name__ == '__main__': 
    r = Robot(com='COM4', baud=250000)
    r.connect()
    r.go_home()
    q1 = ikt.inv_kinematics([-0.1,0.35,0.175,0,np.pi,0])[0,:]
    q2 = ikt.inv_kinematics([0.288, -0.288, 0.155,0, np.pi, 0])[0,:]
    traj1 = until.quinticCurvePlanning([0,0,0,0,0,0], q1, 10)
    traj2 = until.quinticCurvePlanning(q1, q2, 10)
    moveTraj(r, traj1, 10)

    position = inv_based_traj_planning([0.1,0.35,0.175],[-0.1,0.35,0.175],t-tOA,tAC)
    pose = [0, 3.1415,0]
    p_p = np.hstack((position,pose))
    q = ikt.inv_kinematics(p_p)[0,:]*180/np.pi

