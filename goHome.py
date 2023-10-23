'''
Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
Date: 2023-10-23 13:47:24
LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
LastEditTime: 2023-10-23 14:16:39
FilePath: \Demo程序（含代码注释）\goHome.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
from Robot import Robot
 

if __name__ == '__main__': 
     # 打开设备管理器确定连接的COM口，linux和mac只要选择对应串口就行，需要根据具体的串口进行更改，但是波特率不要改
    r = Robot(com='COM4', baud=250000) 
    # 连接到真实机器人
    # 发什么电呢

    r.connect()
    # 使用该函数可以使机械臂回到零位
    r.go_home()
  