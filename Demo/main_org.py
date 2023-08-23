#!/bin/bash
import sys
from info_c import INFO_C, Actions, Robot, WorkTable, utils


# 检查是否接收到结束标志位 ’OK‘
def read_util_ok():
    while input() != "OK":
        pass

# 标准化控制命令结束标志
def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()

# 读取地图数据
def ReadMapData():
    """读取地图数据"""
    global DS
    # 地图数据是 100*100 的
    for row in range(100):
        line = sys.stdin.readline()
        for col in range(100):
            # 空地
            if line[col] == '.': continue
            # 机器人
            elif line[col] == 'A':
                DS.robots.append([-1, 0, 0, 0, 0, 0, col*0.5+0.25, (99-row)*0.5+0.25])
            # 工作台
            else:
                DS.work_benches[int(line[col])-1].append([col*0.5+0.25, (99-row)*0.5+0.25, 0, 0, 0])
    # 读取结束标志，防止阻塞后文
    sys.stdin.readline()
    return None


def FrameDataProcessor():
    """信息处理器：每帧数据处理"""
    global DS
    lineIdx = 0
    while True:
        if lineIdx == 0:
            # 正常读取数据
            line = sys.stdin.readline()
            parts = line.split(' ')
            # 读取当前帧信息
            DS.frameId = int(parts[0])
            DS.current_coin = int(parts[1])
            lineIdx = 1
        elif lineIdx == 1:
            # 正常读取数据
            line = sys.stdin.readline()
            parts = line.split(' ')
            # 读取控制台状态信息
            DS.work_benches = [[] for _ in range(9)]
            for _ in range(int(parts[0])):
                res = sys.stdin.readline().split(' ')
                DS.work_benches[int(res[0])-1].append([
                    # 坐标x、坐标y、剩余生产帧数、原材料格状态(二进制)、产品格状态(0,1)
                    float(res[1]), float(res[2]), int(res[3]), int(res[4]), int(res[5])
                ])
            lineIdx = 2
        elif lineIdx == 2:
            # 读取机器人状态信息
            DS.robots = []
            for _ in range(4):      # 全场一共 4 个机器人
                res = sys.stdin.readline().split(' ')
                DS.robots.append(
                    # 所处工作台ID、携带的物品类型、时间价值系数、碰撞价值系数
                    [int(res[0]), int(res[1]), float(res[2]), float(res[3]), float(res[4]),
                    # 角速度、线速度（二维）、朝向
                    float(res[5]), float(res[6]), float(res[7]),
                    # 机器人坐标(x, y)
                    float(res[8]), float(res[9])] )
            lineIdx = 3
        else:
            line = sys.stdin.readline()
            if not line: continue
            # 读取完成标志
            if line[:2] == 'OK':
                break
            # 比赛结束标志
            if line == 'EOF(end of file)\n':
                return False
    return True


def RobotDesicion(rb_idx):
    """针对ID为 rb_idx 的机器人，计算其相应决策"""
    global DS
    # 指令类型有: 【forward[-2.0,6.0]、rotate[-pi,pi]、buy、sell、destroy】
    res = []
    # 机器人不要超出边界，不要与其他机器人碰撞
    # 最大化拥有得到金币数
    res = 'forward {} 6.0\n'.format(rb_idx)
    # if DS.robots_mission[rb_idx] == 0:      #暂无任务
    # 先到已经生成产品的工作台那里拿出物料
    # 从第七个开始倒序往前查询，找到已有产品的工作台

    # 根据工作台坐标找到最近的机器人并控制机器人过来

    # 如果机器人手上持有物品，则寻找需要该物品的工作台，若都不需要就拿去卖了
    # if DS.robots[rb_idx][1] in DS.
    return res


def MenchSate():
    # 计算各个工作台需要什么材料
    for row in range(9):
        for col in range(len(DS.work_benches[row])):
            DS.work_benches[row][col][3] -= DS.bench_need[row]


def FrameOutput():
    """输出当前帧决策"""
    # 先输出当前帧计数
    sys.stdout.write('%d\n' % DS.frameId)
    # 输出格式：str(指令名称   执行指令的机器人ID   执行指令的值)
    for robotIdx in range(4):
        returnStr = RobotDesicion(robotIdx)
        for string in returnStr:
            sys.stdout.write(string)
    return


if __name__ == '__main__':
    # 初始化数据处理类
    DS = DataStructure()
    # 等待初始化完成
    ReadMapData()
    # 需要加一个处理初始数据的函数，地图数据预处理完成后再返回‘OK’
    # finish()
    sys.stdout.write('OK\n')
    sys.stdout.flush()
    while True:
        # 帧数据处理，如果检测到结束标志，则退出程序
        if not FrameDataProcess():
            break
        # 针对当前状态的输出，包括机器人群体决策
        FrameOutput()
        # 最后是当前帧的标准输出
        # finish()
        sys.stdout.write('OK\n')
        sys.stdout.flush()

    # ----------------------------------------------------------------------
    # End of [ main ]
    # ----------------------------------------------------------------------
