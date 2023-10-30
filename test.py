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

INTERVAL = 0.02
#逆解器
#逆解器使用范例:q_int_1 = ikt.inv_kinematics([-0.1,0.35,0.175,0,np.pi,0])[0,:]

# go home

# “起始区域”的一号关节关节角应为0

# “终止区域”的一号关节关节角应为90度


#笛卡尔空间直线规划仿真#

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
            xArray.append([0, 1, 0, 0, 0, 0])
        xMatrix = np.matrix(xArray).T
        
        kMatrix = tMatrix.I * xMatrix
        
        timeVector = np.matrix([t**5, t**4, t**3, t**2, t, 1]).T
        x = (kMatrix.T * timeVector).T.A[0]
        
        
    else:
        x = end
    return x

# def invTraj(start, end, duration, interval=INTERVAL):
#     for t in 
#         yield inv_based_traj_planning(start,end,t,duration):

# def invTraj(start, end, t, duration):
#     orientation = (np.array(end) - np.array(start)) * t / duration
#     return # TODO

def trajPlaningDemo(start, end, t, time):
    """ Quintic Polynomial: x = k5*t^5 + k4*t^4 + k3*t^3 + k2*t^2 + k1*t + k0
    :param start: Start point
    :param end: End point
    :param t: Current time
    :param time: Expected time spent
    :return: The value of the current time in this trajectory planning
    """
    if t < time:
        tMatrix = np.matrix([
        [         0,           0,             0,          0,        0,   1],
        [   time**5,     time**4,       time**3,    time**2,     time,   1],
        [         0,           0,             0,          0,        1,   0],
        [ 5*time**4,   4*time**3,     3*time**2,     2*time,        1,   0],
        [         0,           0,             0,          2,        0,   0],
        [20*time**3,  12*time**2,        6*time,          2,        0,   0]])
        
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



#三次样条插值
def CubSp_traj_planning(time_array,pose_array,sample_time):
    
    data = np.vstack(pose_array)
    num_time_points = data.shape[0]

    
    splines = [CubicSpline(time_array, data[:, i], bc_type=((1, 0.0), (1, 0.0)) ) for i in range(6)]

    q = [s(sample_time) for s in splines]
    return np.array(q)

def cubSpTraj(time_array,pose_array, interval=INTERVAL):
    for t in np.arange(time_array[0], time_array[-1], interval):
        yield CubSp_traj_planning(time_array,pose_array, t)

def test():
    global q_0,q_1,q_2
    q_0 = np.array([0,0,0,0,0,0])
    # A点，直线起始处
    q_int_A = ikt.inv_kinematics([0.370, -0.090, 0.155,0,np.pi,0])[0,:]*180/np.pi
    # B点，直线结束处
    q_int_B = ikt.inv_kinematics([0.288, -0.288, 0.155,0, np.pi, 0])[0,:]*180/np.pi
    q_1_prev = ikt.inv_kinematics([0.4, 0, 0.155,0, np.pi, 0])[0,:]*180/np.pi#准备抓取物块一
    q_1 = ikt.inv_kinematics([0.4, 0, 0.1,0, np.pi, 0])[0,:]*180/np.pi#物块一的初始位置
    
    # 打开设备管理器确定连接的COM口，linux和mac只要选择对应串口就行，需要根据具体的串口进行更改，但是波特率不要改
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
    
    tOA = 4
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
    r.go_home()
    # 开始控制机器人
    while(1):
        
        start = time.time()
        # 重新开始一次循环
        if t >= tOA + tAC:
            print('Control Finished')
            
        # 通过时间与规划目标关节位置的关系，得到挡墙时刻下期望机械臂关节到达的位置
        elif t < 7:
            print(q_int_A)
            q = CubSp_traj_planning([0,5,7],[q_0,q_1_prev,q_1],t)
        #elif t <tOA + tAC:
        #     #q = until.quinticCurveExcute2(k,t-tOA)
        #     position = inv_based_traj_planning([0.4,-0.12,0.15],[0.4,0.12,0.15],t-tOA,tAC)
        #     pose = [0, 3.1415,0]
        #     p_p = np.hstack((position,pose))
        #     q = ikt.inv_kinematics(p_p)[0,:]*180/np.pi
        #     print(p_p)
            # 控制机械臂运动，syncMove输入格式为1*6的np.array，单位为度，代表的含义是当前周期下机械臂关节的位置
            
        print(q)
        r.syncMove(q)
        print(r.syncFeedback())
        # 更新时间
        t = t + T
        # 定时器操作
        end = time.time()
        spend_time = end - start
        if spend_time < T:
            time.sleep(T - spend_time)
        else:
            print("timeout!")


def moveTraj(robot: Robot, traj, interval=0.002):  
    lastTime = time.time()
    for pos in traj:
        r.syncMove(pos)
        time.sleep(interval)
        print(pos)
        assert -90 < pos[0] < 90 and -90 < pos[1] < 90 and -120 < pos[2] < 120 and -150 < pos[3] < 150 and -150 < pos[4] < 150 and -180 < pos[5] < 180, "Invalid position: {}".format(pos)
        realPos = robot.syncFeedback()[:, 0]
        # print(realPos, type (realPos))
        assert -90 < realPos[0] < 90 and -90 < realPos[1] < 90 and -120 < realPos[2] < 120 and -150 < realPos[3] < 150 and -150 < realPos[4] < 150 and -180 < realPos[5] < 180, "Invalid position: {}".format(realPos)
        robot.syncMove(pos)
        spend_time = time.time() - lastTime
        lastTime = time.time()
        if spend_time < interval:
            time.sleep(interval - spend_time)
        else:
            print("timeout!")


def getJointAngle(pos):
    return  ikt.inv_kinematics(pos)[0,:]*180/np.pi

if __name__ == '__main__': 
    print("kkkk",getJointAngle([0.05, -0.4, 0.1,0, np.pi, 0]))
    # exit(0)
    # interval = 0.02
    r = Robot(com='COM4', baud=250000)
    r.connect()
    r.go_home()

    # define waypoint
    start = np.array([0,0,0,0,0,0])
    cubic1 = getJointAngle([0.36, 0.075, 0.1,0, np.pi, 0])
    topOfCubic1 = getJointAngle([0.36, 0.075, 0.2,0, np.pi, 0])
    cubic2 = getJointAngle([0.36, -0.050, 0.075,0, np.pi, 0])
    topOfCubic2 = getJointAngle([0.36, -0.050, 0.2,0, np.pi, 0])
    halfwayOfCubic2 = (cubic2 + topOfCubic2) / 2
    lineStart = [0.37, -0.09, 0.2,0,np.pi,0]
    lineEnd = [0.288, -0.288, 0.2,0, np.pi, 0]
   
    linearMoveStart = getJointAngle(lineStart)
    lineMoveEnd = getJointAngle(lineEnd)
    
    topOfDrop = getJointAngle([0.145, -0.38, 0.25,0, np.pi, 0])
    dropCubic1 = getJointAngle([0.145, -0.38, 0.075,0, np.pi, 0])
    dropCubic2 = getJointAngle([0.142, -0.385, 0.12,0, np.pi, 0])

    trajLinear = cubSpTraj(np.linspace(0, 3, 200), list(map(getJointAngle, np.linspace(lineStart, lineEnd, 200))))

    # get cubic 1
    moveTraj(r, cubSpTraj([0,4,5,8,9]
                          + np.linspace(10, 13, 100, endpoint=True).tolist()
                          + [15, 17],
                          
                          [start,topOfCubic1,cubic1,cubic1,topOfCubic1]
                          + list(map(getJointAngle, np.linspace(lineStart, lineEnd, 100, endpoint=True)))
                          + [topOfDrop, dropCubic1]
                          ))
    
    # drop cubic 1
    # moveTraj(r, cubSpTraj([0, 2, 4, 6, 8],[lineMoveEnd, topOfDrop, dropCubic1, dropCubic1, topOfDrop]))

    # go to cubic 2 and get cubic 2
    moveTraj(r, cubSpTraj([0,2,4,6,8,10,12,14]
                          + np.linspace(17, 20, 100, endpoint=True).tolist()
                          + [22, 24, 26, 28],
                          [dropCubic1, dropCubic1, topOfDrop, topOfCubic2, halfwayOfCubic2, cubic2, cubic2, topOfCubic2]
                          + list(map(getJointAngle, np.linspace(lineStart, lineEnd, 100, endpoint=True)))
                          + [topOfDrop, dropCubic2, dropCubic2, topOfDrop]
                          ))

    # move linear trajectory
    # moveTraj(r, trajLinear)
    # moveTraj(r, cubSpTraj(np.linspace(0, 3, 100), list(map(getJointAngle, np.linspace(lineStart, lineEnd, 100)))))

    # drop cubic 2
    # moveTraj(r, cubSpTraj([0, 2, 4, 6, 8],[lineMoveEnd, topOfDrop, dropCubic2, dropCubic2, topOfDrop]))

    print("done")

    

    
    