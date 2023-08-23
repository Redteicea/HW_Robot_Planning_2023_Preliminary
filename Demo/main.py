#!/bin/bash
import sys
from typing import List

from UsefulTools import LOG
from manager import Manager
from controller import  Controller
from dTree import Tree, workPath, getScatteringRate, unlockPath, lockPath
from info_c import INFO_C, Actions, utils


def negateByBit(num):
    res = []
    for n in bin(num)[2:]:
        if n == '1': res.append('0')
        else: res.append('1')
    return int('0b'+''.join(res), 2)


def main():
    # -----------------------------------------------------------------------
    # 程序参数设置 ===================================
    K = -1  # 树的分支数
    forcast_step = 1    # 预测步数
    actions = Actions()
    INFO = INFO_C()
    log = LOG()
    log.LogInitialize()
    log.LogRecord("------------------------------------------------------")
    # -----------------------------------------------------------------------
    # 程序初始化 ====================================
    log.LogRecord("[main] Start!")
    frameStr = utils.readUntilOK()
    if len(frameStr) == 0: log.LogRecord("[main][initialize] Load map failed")
    sysMgr = Manager(frameStr)  # 信息管理器
    sysCtrl = Controller(sysMgr.robots)   # 控制器
    ## 针对不同地图设置参数
    scattering_rate = getScatteringRate(sysMgr.worktables)
    log.LogRecord('[main][initialize] map scatter rate:{}'.format(scattering_rate))
    ##----------------------------------------------------------------------
    ## 读取地图选择参数---------------------------------------------------
    if 1500 < scattering_rate < 1900:
        INFO.MAP_NUMBER = 1
        K = 1
        forcast_step = 1
    elif 2000 < scattering_rate < 3000 :
        INFO.MAP_NUMBER = 2
        K = 1
        forcast_step = 1
    elif 10000 < scattering_rate < 12000:
        INFO.MAP_NUMBER = 3
        K = -1
        forcast_step = 1
        ### 初始最佳路径，后续由节点树判断
        wtid_list0 = (6,4,0,4,6,2,0,2,6,4,0,4,4,24,6,2,0,2,2,24,6,4,0,4,4,24,6,2,0,2,2,24,6,4,0,4,4,24,6,2,0,2,2,24,6,4,0,4,4,24,6,2,0,2,2,24,6,
                       4,0,4,4,24,6,2,0,2,2,24,6,4,0,4,4,24,6,2,0,2,2,24,6,4,0,4,4,24,6,2,0,2,2,24,6,4,4,24,6,5)
        wtid_list1 = (36,41,42,41,36,45,42,45,41,24,36,45,42,45,45,24,36,45,42,45,45,24,36,45,42,45,45,24,36,45,42,45,45,24,36,45,42,
                       45,45,24,36,45,42,45,45,24,36,45,42,45,45,24,36,45,42,45,45,24,36,45,42,45,45,24,36,45,42,45,45,24,42,45,45,24,36,40)
        wtid_list2 = (16,22,16,11,16,32,28,11,16,32,28,32,28,32,32,24,28,12,16,22,22,24,16,22,28,11,12,24,16,32,28,11,16,11,11,24,28,
                       11,16,11,11,24,16,22,16,11,11,24,16,22,22,24,16,22,28,22,22,24,16,22,22,24,16,11,16,22,22,24,16,32,28,32,28,11,16,22,
                       22,24,32,24,16,11,11,24,16,22,16,22,28,22,10)
        wtid_list3 = (28,32,28,22,16,12,16,22,28,22,22,24,16,11,11,24,16,32,28,32,28,22,16,32,32,24,28,32,32,24,28,22,22,24,28,32,28,
                       22,22,24,16,32,32,24,28,32,28,22,32,24,28,11,16,32,32,24,28,22,28,32,32,24,28,11,11,24,16,32,28,22,11,24,28,22,28,32,
                       16,11,16,32,28,22,22,24,28,32,32,24, 31, 24)     #,11,24,10)
        wtid_list = [wtid_list0, wtid_list1, wtid_list2, wtid_list3]
        for i in range(4):
            sysMgr.targetWtab[i].clear()
            for j in range(len(wtid_list[i])-1, -1, -1):
                sysMgr.targetWtab[i].append(wtid_list[i][j])
            sysCtrl.setObjectloc(i, wtid_list[i][0], sysMgr.worktables[wtid_list[i][0]].loc)
    else:
        INFO.MAP_NUMBER = 4
        K = -1
        forcast_step = 1
        # ### 初始最佳路径，后续由节点树判断
        # wtid_list0 = (1,17,3,17,1,17,3,17,4,17,3,17,1,17,3,17,4,17,3,12,9,11,8,12,9,15,11,0, 8,11)
        # wtid_list1 = (6,17,4,17,17,0,6,17,17,0,0,16,17,0,6,17,4,17,17,0,0,16,17,0,0,16,17,0,0,16,17,0,0,16,17,0)
        # wtid_list2 = (2,10,7,10,5,13,7,13,10,0,6,13,7,13,13,0,1,10,13,0,3,14,5,11,8,11,14,0,3,14,9,14,9,11,11,0,
        #               6,11,8,11,14,0,6,11,13,0,6,11,7,13,10,0,6,11)
        # wtid_list3 = (5,11,8,11,3,14,11,0,9,14,5,11,8,11,14,0,9,14,7,10,2,10,8,10,10,0,8,13,11,0,5,13,7,13,8,10,
        #               10,0,1,10,13,0,5,13,8,10,10,0,1,10,7,10,0,16, 9,12)
        # wtid_list = [wtid_list0, wtid_list1, wtid_list2, wtid_list3]
        # for i in range(4):
        #     sysMgr.targetWtab[i].clear()
        #     for j in range(len(wtid_list[i])-1, -1, -1):
        #         sysMgr.targetWtab[i].append(wtid_list[i][j])
        #     sysCtrl.setObjectloc(i, wtid_list[i][0], sysMgr.worktables[wtid_list[i][0]].loc)
    ## end of 读取地图选择参数
    log.LogRecord('[main][initialize] map number:{}'.format(INFO.MAP_NUMBER))
    ##----------------------------------------------------------------------
    # 节点数----------------------------------------------------------------
    nodeTree = Tree(sysMgr, K)
    # end of '程序初始化‘ ===============================
    log.LogRecord('[main][initialize] System initialize successfully...')
    sys.stdout.write('OK\n')    # 初始化完成后要给系统一个‘OK’标志
    sys.stdout.flush()
    # -----------------------------------------------------------------------
    # 程序执行 ======================================
    order:List[str] = list()
    intensity:List[float] = list()
    orders: List[List[str]] = list(list())
    intensities: List[List[float]] = list(list())
    paths:List[workPath] = [None, None, None, None]
    # -----------------------------------------------------------------------
    log.LogRecord('[main][excute] running')
    while sysMgr.frameId != 'EOF':   # 比赛结束标志
        # 读取帧信息
        frameStr = utils.readUntilOK()
        sysMgr.update(frameStr)
        # 已经下一帧了，解锁物品拿取锁
        for i in range(4):
            if paths[i]:
                paths[i].target.locked_get = False
        # -------------------------------------------------------------------------
        # 路径&决策规划
        for i in range(4):
            # ----------------------------------------------------------------------
            if INFO.MAP_NUMBER == 3 or INFO.MAP_NUMBER == 4:
                # -------------------------------------------------------------------
                # 没有设置目标，进行新的迭代
                if len(sysMgr.targetWtab[i]) == 0:
                    log.LogRecord('[main][planning] robot:{} has not target try to find new one'.format(i))
                    # 解锁
                    if paths[i]:
                        unlockPath(paths[i])
                        paths[i] = None
                    # 选择下一个目标, TreeNode
                    path = nodeTree.getBestNode(sysMgr.robots[i].loc, sysMgr.robots[i].direct,
                                                forcast_step, INFO.frame_end - sysMgr.frameId)
                    if path:    # 能找到目标
                        log.LogRecord('[main][planning] robot {} target is {}.'.format(i, path.begin.wTab_id))
                        # sysMgr 添加2个目标点进堆栈
                        sysMgr.targetWtab[i].append(path.target.wTab_id)
                        sysMgr.targetWtab[i].append(path.begin.wTab_id)
                        log.LogRecord('[main][planning][frame {}] robot{} found path : {} -> {}, path value: {}'
                                      .format(sysMgr.frameId, i, path.begin.class_id, path.target.class_id, path.est_price))
                        # 记录当前路径
                        paths[i] = path
                        # 上锁
                        lockPath(path)
                        # 设置目标
                        idx = sysMgr.targetWtab[i][-1]
                        sysCtrl.setObjectloc(i, idx, sysMgr.worktables[idx].loc)
                    else:
                        log.LogRecord('[main][planning] robot {} can not find target path.'.format(i))
                # end of '没有设置目标，进行新的迭代'
                # 判断是否到达目标工作台
                elif sysCtrl.isArrived(sysMgr.robots[i], order):
                    log.LogRecord('[main][planning] robot {} is arrived. Its order is '.format(i) + order[0])
                    # -----------------------------------------------------------------
                    # 当可 买卖 时
                    if ((order[0] == 'ACT_BUY') and (sysMgr.worktables[sysMgr.targetWtab[i][-1]].sta_produce == 1)) or\
                        ((order[0] == 'ACT_SELL') and negateByBit(sysMgr.worktables[sysMgr.targetWtab[i][-1]].sta_material & (0x01<<sysMgr.robots[i].objectID))):
                        sysMgr.control(i, order[0])
                        log.LogRecord('[main][planning] robot {} :'.format(i) + order[0] + \
                                      ' class_id: ' + str(sysMgr.worktables[sysMgr.targetWtab[i][-1]].classID))
                        # 还有下一个目标则弹出本目标，前往下一个目标，
                        # 没目标则在本目标点附近等待
                        if len(sysMgr.targetWtab[i]) > 0:
                            sysMgr.targetWtab[i].pop()
                        if len(sysMgr.targetWtab[i]) > 0:
                            idx = sysMgr.targetWtab[i][-1]
                            log.LogRecord('[main][planning] robot {} next target:{}, location:{}, class:{}'.format(i,
                                                            idx, sysMgr.worktables[idx].loc, sysMgr.worktables[idx].classID))
                            sysCtrl.setObjectloc(i, idx, sysMgr.worktables[idx].loc)
                    # end of '当可 买卖 时'
                    # -----------------------------------------------------------------
                # end of '判断是否到达目标工作台'
                # -------------------------------------------------------------------
            # end of if INFO.MAP_NUMBER == 3 or INFO.MAP_NUMBER == 4:
            # ----------------------------------------------------------------------
            # 其他地图 且 没有设置目标，进行新的迭代
            elif len(sysMgr.targetWtab[i]) == 0:
                # log.LogRecord('[main][planning] robot:{} has not target try to find new one'.format(i))
                # 解锁
                if paths[i]:
                    log.LogRecord('robot:{}, unlock path[i]:{}'.format(i, paths[i]))
                    unlockPath(paths[i])
                    paths[i] = None
                # 选择下一个目标, TreeNode
                path = nodeTree.getBestNode(sysMgr.robots[i].loc, sysMgr.robots[i].direct,
                                            forcast_step, INFO.frame_end - sysMgr.frameId)
                if path:  # 能找到目标
                    # sysMgr 添加2个目标点进堆栈
                    sysMgr.targetWtab[i].append(path.target.wTab_id)
                    sysMgr.targetWtab[i].append(path.begin.wTab_id)
                    log.LogRecord('[main][planning][frame {}] robot{} found path : {}/{} -> {}/{}, path value: {}'
                                  .format(sysMgr.frameId, i, path.begin.class_id, path.begin.wTab_id,
                                          path.target.class_id, path.target.wTab_id, path.est_price))
                    # 记录当前路径
                    paths[i] = path
                    log.LogRecord('robot:{} lock path:{}, path[i]:{}'.format(i, path, paths[i]))
                    # 上锁
                    lockPath(path)
                    # 设置目标
                    idx = sysMgr.targetWtab[i][-1]
                    sysCtrl.setObjectloc(i, idx, sysMgr.worktables[idx].loc)
                else:
                    log.LogRecord('[main][planning] robot {} can not find target path.'.format(i))
            # end of '其他地图 且 没有设置目标，进行新的迭代'
            # ----------------------------------------------------------------------
            # 其他地图，判断是否到达目标工作台
            elif sysCtrl.isArrived(sysMgr.robots[i], order):
                log.LogRecord('[main][planning] robot {} is arrived.'.format(i))
                # ------------------------------------------------------------
                # 当可 买卖 时
                if ((order[0] == actions.BUY) and (sysMgr.worktables[sysMgr.targetWtab[i][-1]].sta_produce == 1)) or \
                        ((order[0] == actions.SELL) and negateByBit(sysMgr.worktables[sysMgr.targetWtab[i][-1]].sta_material & (
                                0x01 << sysMgr.robots[i].objectID))):
                    sysMgr.control(i, order[0])
                    log.LogRecord('[main][planning] robot {} :'.format(i) + order[0])
                    # 将该工作台物品设为0，防止下一个机器人重复购买
                    paths[i].target.locked_get  = True
                    # 更新路径估值
                    paths[i].updatePrice(sysMgr.robots[i].price)
                    # 还有下一个目标则弹出本目标，前往下一个目标，
                    # 没目标则在本目标点附近等待
                    if len(sysMgr.targetWtab[i]) > 0:
                        sysMgr.targetWtab[i].pop()
                    if len(sysMgr.targetWtab[i]) > 0:
                        idx = sysMgr.targetWtab[i][-1]
                        sysCtrl.setObjectloc(i, idx, sysMgr.worktables[idx].loc)
            # end of '其他地图，判断是否到达目标工作台'
            # ----------------------------------------------------------------------
        # end for i in range(4) 路径&决策规划
        # -------------------------------------------------------------------------
        # 避障决策
        sysCtrl.avoid_update()
        # 控制执行
        for i in range(4):
            if len(sysMgr.targetWtab[i]) > 0:
                # 有目标时，执行控制器导航
                # order, intensity 会在该函数里被赋值
                sysCtrl.Control(order, intensity, i)
                for j in range(len(order)):
                    sysMgr.control(i, order[j], intensity[j])
            else:
                # 没目标时，缓慢旋转
                sysMgr.control(i, 'ACT_FORWARD', 2)
                sysMgr.control(i, 'ACT_ROTATE', INFO.PI / 2)
        # -----------------------------------------------------------------------
        # 执行决策命令
        sysMgr.doActions()
    # end while sysMgr.frameId != 'EOF':
    # -----------------------------------------------------------------------
    return None


if __name__ == '__main__':
    main()
