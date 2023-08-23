import math
from typing import List
from UsefulTools import LOG
from manager import Manager
from info_c import INFO_C, WorkTable, utils

log = LOG('dTree')
log.LogInitialize()
INFO = INFO_C()
maxSpeed_sec = 4.5    # 用于计算的最大速度（秒）
maxRad_sec = INFO.PI/2      # 用于计算的最大角速度
factor_forecast = 0.1       # 预测价值比的衰减因子
PATH_LR = 0.3     # 路径价值更新的学习率
if INFO.MAP_NUMBER == 1:
    maxSpeed_sec = 4.5
    factor_forecast = 0.2
    PATH_LR = 0.00


def negateByBit(num):
    # for n in bin(num)[2:]:
    #     if n == '1': res.append('0')
    #     else: res.append('1')
    numStr = bin(num)[2:]
    res, n = [], len(numStr)
    for i in range(8):
        if 8 - i > n:
            res.append('1')
        else:
            if numStr[i - 8 + n] == '1': res.append('0')
            else: res.append('1')
    return int('0b'+''.join(res), 2)


class TreeNode(object):
    def __init__(self, wTab:WorkTable):
        self.forward = list() # 向前传递的路径
        self.wTab = wTab # 工作台指针
        self.wTab_id = wTab.ID   # 工作台ID
        self.class_id = wTab.classID  #  类别[1-9]
        self.locked = False # 加锁
        self.putLock = 0x00 # 放入锁，防止重复放入
        self.locked_next = False  # 第二份产品锁
        self.locked_get = False # 物品拿取锁，防止重复拿取

    def addForward(self, node) -> None:
        """
        添加前向节点
        :param node:
        :return: None
        """
        self.forward.append(workPath(self, node))
        return None

    def getPutLock(self, obj_id) -> bool:
        """
        读取放入锁
        :param obj_id: 查询物品id是否已有人准备放入
        :return:
        """
        return True if self.putLock & (0x01 << obj_id) else False

    def getWantPutCnt(self) -> int:
        """
        读取有多少机器人想来放
        :return: 想放的机器人数量
        """
        res = 0
        for idx in range(8):
            if self.putLock & (0x01 << idx):
                res += 1
        return res

    def getPutOK(self, obj_id, allowExtern=False) -> bool:
        """
        是否可以放入某物品
        :param obj_id: 物品类别
        :param allowExtern: 是否允许当前有物品（晚点会合成）
        :return: True 可放入 False 不可放入
        """
        if self.class_id >= 8:  # 工作台类型8和9可以放入任意类型物品
            return True
        if allowExtern:
            # 当前有槽位 或 不阻塞且要合成了
            bHasVacancy = negateByBit(self.putLock | self.wTab.sta_material) & (0x01<<obj_id) & INFO.TAB_WTAB_INPUT_CLASS[self.class_id]
            # if not bHasVacancy:
            #     log.LogRecord('[Tree][GetPutOk] node {}/{} has not vacancy.'.format(self.class_id, self.wTab_id))
            #     log.LogRecord('\t put_lock:' + bin(self.putLock)[2:] + '\tsta_material:' + bin(self.wTab.sta_material)[2:] + '\tput in class' + bin(0x01<<obj_id)[2:])

            # 不阻塞且要合成了
            bToSynthesis = self.wTab.last != 0 and (self.putLock | self.wTab.sta_material) == INFO.TAB_WTAB_INPUT_CLASS[obj_id]
            # if not bToSynthesis:
            #     log.LogRecord('[Tree][GetPutOk] node {}/{} has not synthesis.'.format(self.class_id, self.wTab_id))
            return bHasVacancy or bToSynthesis
            # return (negateByBit(self.putLock | self.wTab.sta_material) & (0x01<<obj_id) & INFO.TAB_WTAB_INPUT_CLASS[self.class_id]) or \
            #        (self.wTab.last != 0 and (self.putLock | self.wTab.sta_material) == INFO.TAB_WTAB_INPUT_CLASS[obj_id])
        else:
            # 当前有槽位
            return negateByBit(self.putLock | self.wTab.sta_material) & (0x01<<obj_id) & INFO.TAB_WTAB_INPUT_CLASS[self.class_id]

    def lock(self, obj_id) -> None:
        """
        放入锁 锁定
        :param obj_id:
        :return:
        """
        self.putLock |= (0x01<<obj_id)
        return None

    def unlock(self, obj_id) -> None:
        """
        解锁
        :param obj_id:
        :return:
        """
        self.putLock &= negateByBit(0x01<<obj_id)
        return None


class Tree(object): # 索引树
    def __init__(self, sys:Manager, K:int):
        # 树的不同层，0为最高层, 层数存放工作台分别为[1,2,3], [4,5,6], [7], [8,9]
        self.nodes:List[List[TreeNode]] = list(list())
        # 查询表，存放的都是指针，若该工作台不为树节点，则为NULL
        self.searchTab:List[TreeNode] = list()
        # 地图上已存在的各个材料的数量[1~7]；[0]为占位用
        self.p_extern_material = [0] * 8
        # 地图上可以拿取的各个材料的数量[1~7]；[0]为占位用
        self.p_extern_materia_available  = [0] * 8
        # 每个层存放的类别 [8], [7], [4,5,6], [1,2,3]
        self.b_layerClass = [0x01 << 8, 0x01 << 7, 0x07 << 4, 0x07 << 1]
        # self.initialize_org(sys, K)
        self.initialize(sys, K)


    def initialize_org(self, sys:Manager, K:int):
        self.p_extern_material = sys.extern_materia     # 指向材料余量
        self.p_extern_materia_available = sys.extern_materia_available    # 指向可拿取材料余量
        wTabs = sys.worktables    # 指向工作台列表，用于录入节点信息
        # --------------------------------------------------------------------
        # 1.根据层数建立树系统
        ## 先将所有节点录入对应类
        self.nodes:List[List[TreeNode]] = [[] for _ in range(10)]
        for wTab in wTabs:
            cur_node = TreeNode(wTab)
            self.nodes[wTab.classID].append(cur_node)
        for idx in range(9, 0, -1):
            log.LogRecord('[Tree][initialize] layer {} cnt: {}'.format(idx, len(self.nodes[idx])))
        # --------------------------------------------------------------------
        ## 寻找最高节点等级，b_top_layer为二进制运算，用于在第一层不用检测是否有path
        b_top_layer = 0x00
        if len(self.nodes[8]) > 0:
            b_top_layer = self.b_layerClass[0]
        if len(self.nodes[7]) > 0:
            b_top_layer = self.b_layerClass[1]
        if len(self.nodes[4]) > 0:
            b_top_layer = self.b_layerClass[2]
        else:
            b_top_layer = self.b_layerClass[3]
        # --------------------------------------------------------------------
        ## 自上而下，连接K个节点
        for c in range(9, 3, -1):
            for i in range(len(self.nodes[c])):
                father = self.nodes[c][i]   # dtype = TreeNode
                # 非顶层时，不选择没有path的节点
                if (not (b_top_layer & (0x01 << father.class_id))) and c != 9 and len(father.forward) == 0:
                    continue
                # ---------------------------------------------------------------
                # 在可选子类中寻找子节点
                for c2 in range(1, 8):
                    # -------------------------------------------------------------
                    # 选取可输入的子类，若不是可输入的子类，则跳过
                    if not INFO.TAB_WTAB_INPUT_CLASS[c] & (0x01 << c2):
                        continue
                    # 只允许9链接最顶层可售卖节点
                    if c == 9:
                        # 最顶层为[8]，则连接[7]。若最顶层为8且当前类不是7时跳过
                        if (b_top_layer == self.b_layerClass[0]) and not (self.b_layerClass[1] & (0x01 << c2)):
                            continue
                        elif not b_top_layer & (0x01 << c2):    # 当前类不是顶层时跳过
                            continue
                    # -------------------------------------------------------------
                    distance, idxs = list(), list()
                    # 计算每个节点的距离
                    for s in range(len(self.nodes[c2])):
                        son = self.nodes[c2][s]
                        idxs.append(s)
                        distance.append(utils.getDistance(son.wTab.loc, father.wTab.loc))
                    # -------------------------------------------------------------
                    # 将距离进行排序（递增）
                    order = [i for i in range(len(idxs))]
                    order = [k for k, _  in sorted(list(zip(order, distance)), key=lambda pair:pair[1])]
                    # 取前K个节点作为子节点
                    count = len(idxs)
                    if K != -1 and count > K: count = K
                    # -------------------------------------------------------------
                    if INFO.MAP_NUMBER == 1:
                        if c2 == 4 or c2 == 5:
                            son = self.nodes[c2][idxs[order[0]]]
                            son.addForward(father)
                            continue
                        elif c2 == 6 or c2 == 7:
                            son = self.nodes[c2][idxs[order[2]]]
                            son.addForward(father)
                            continue
                        else:
                            for j in range(count):
                                son = self.nodes[c2][idxs[order[j]]]
                                son.addForward(father)
                    # end if NFO.MAP_NUMER == 1
                    # -------------------------------------------------------------
                    elif INFO.MAP_NUMBER == 2:# 地图2 选择右侧的点
                        if c2 == 4: #12
                            for j in range(len(self.nodes[c2])):
                                son = self.nodes[c2][j] # dtype = TreeNode
                                if son.wTab_id == 12:
                                    son.addForward(father)
                        elif c2 == 5: # 1， 23
                            for j in range(len(self.nodes[c2])):
                                son = self.nodes[c2][j] # dtype = TreeNode
                                if son.wTab_id == 1 or son.wTab_id == 23:
                                    son.addForward(father)
                        elif c2 == 6: # 2， 24
                            for j in range(len(self.nodes[c2])):
                                son = self.nodes[c2][j] # dtype = TreeNode
                                if son.wTab_id == 2 or son.wTab_id == 24:
                                    son.addForward(father)
                        elif c2 == 7: # 3， 21
                            for j in range(len(self.nodes[c2])):
                                son = self.nodes[c2][j] # dtype = TreeNode
                                if son.wTab_id == 3 or son.wTab_id == 21:
                                    son.addForward(father)
                        else:   # 其他节点使用普通方法
                            for j in range(len(self.nodes[c2])):
                                son = self.nodes[c2][idxs[order[j]]] # dtype = TreeNode
                                son.addForward(father)
                    # end if NFO.MAP_NUMER == 2
                    # -------------------------------------------------------------
                    else:   # 其他地图使用普通方法
                        for j in range(count):
                            son = self.nodes[c2][idxs[order[j]]] # dtype = TreeNode
                            son.addForward(father)
                    # -------------------------------------------------------------
                # end for c2 in range(1, 8)
                # ---------------------------------------------------------------
            # end for i in range(len(self.nodes[c]))
            # ------------------------------------------------------------------
        # end for c in range(9, 3, -1)
        # --------------------------------------------------------------------
        ## 针对不同地图类型设置参数
        if INFO.MAP_NUMBER == 1: # 地图1使用了两个7
            for i in range(len(self.nodes[1])):
                if self.nodes[7][i].wTab_id == 11:  # dtype = TreeNode
                    # -------------------------------------------------------------
                    for c in range(4, 7):   # 连接已有的4，5，6
                        for j in range(len(self.nodes[c])):
                            if len(self.nodes[c][j].forward) == 0: continue
                            self.nodes[c][j].addForward(self.nodes[7][i])
                    # -------------------------------------------------------------
                    for j in range(len(self.nodes[7])): # 连已有的8
                        if len(self.nodes[7][j].forward) == 0: continue
                        self.nodes[7][i].addForward(self.nodes[7][j].forward[0].target)
                    # -------------------------------------------------------------
        # end if INFO.MAP_NUMBER == 1
        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # 2.将选中的节点放入填充查询表（有链接的节点）
        self.searchTab = [None for _ in range(len(wTabs))]
        for layer in range(8):  # 把节点对应的地址放进去
            for j in range(len(self.nodes[layer])): # 首选节点
                # 跳过没有path链接的非顶层节点，第9层虽然也没有path，但是不用连，因为索引的时候用不着
                if len(self.nodes[layer][j].forward) == 0 and not (b_top_layer & (0x01 << self.nodes[layer][j].class_id)):
                    continue
                self.searchTab[self.nodes[layer][j].wTab_id] = self.nodes[layer][j]
        # --------------------------------------------------------------------
        # 打印树关系
        log.LogRecord("[Tree][initialize] Node Tree:")
        for layer in range(9, 0, -1):
            log.LogRecord('[Tree][initialize][layer {}]'.format(layer))
            for a in range(len(self.nodes[layer])):
                for b in range(len(self.nodes[layer][a].forward)):
                    p_path = self.nodes[layer][a].forward[b]
                    string = '\t [Paths] classID:{} --- target: {} / {} <- begin: {} / {} \tpath_value: {}'.format(
                        layer, p_path.target.wTab.classID, p_path.target.wTab.ID,
                        p_path.begin.wTab.classID, p_path.begin.wTab.ID,
                        p_path.est_price / p_path.time_path_f)
                    log.LogRecord(string)
        # --------------------------------------------------------------------
        return None


    def initialize(self, sys:Manager, K:int):
        self.p_extern_material = sys.extern_materia     # 指向材料余量
        self.p_extern_materia_available = sys.extern_materia_available    # 指向可拿取材料余量
        wTabs = sys.worktables    # 指向工作台列表，用于录入节点信息
        # --------------------------------------------------------------------
        # 1.根据层数建立树系统
        ## 先将所有节点录入对应类
        self.nodes:List[List[TreeNode]] = [[] for _ in range(10)]
        for wTab in wTabs:
            cur_node = TreeNode(wTab)
            self.nodes[wTab.classID].append(cur_node)
        for idx in range(9, 0, -1):
            log.LogRecord('[Tree][initialize] layer {} cnt: {}'.format(idx, len(self.nodes[idx])))
        # --------------------------------------------------------------------
        ## 寻找最高节点等级，b_top_layer为二进制运算，用于在第一层不用检测是否有path
        b_top_layer = 0x00
        if len(self.nodes[8]) > 0:
            b_top_layer = self.b_layerClass[0]
        elif len(self.nodes[7]) > 0:
            b_top_layer = self.b_layerClass[1]
        elif len(self.nodes[4]) > 0:
            b_top_layer = self.b_layerClass[2]
        else:
            b_top_layer = self.b_layerClass[3]
        # --------------------------------------------------------------------
        ## 自上而下，连接K个节点
        for c in range(9, 3, -1):
            for i in range(len(self.nodes[c])):
                father = self.nodes[c][i]   # dtype = TreeNode
                # 非顶层时，不选择没有path的节点
                if (not (b_top_layer & (0x01 << father.class_id))) and c != 9 and len(father.forward) == 0:
                    continue
                # ---------------------------------------------------------------
                # 在可选子类中寻找子节点
                for c2 in range(1, 8):
                    # -------------------------------------------------------------
                    # 选取可输入的子类，若不是可输入的子类，则跳过
                    if not INFO.TAB_WTAB_INPUT_CLASS[c] & (0x01 << c2):
                        continue
                    # 只允许9链接最顶层可售卖节点
                    if c == 9:
                        # 最顶层为[8]，则连接[7]。若最顶层为8且当前类不是7时跳过
                        if (b_top_layer == self.b_layerClass[0]) and not (self.b_layerClass[1] & (0x01 << c2)):
                            continue
                        elif not b_top_layer & (0x01 << c2):    # 当前类不是顶层时跳过
                            continue
                    # -------------------------------------------------------------
                    distance, idxs = list(), list()
                    # 计算每个节点的距离
                    for s in range(len(self.nodes[c2])):
                        son = self.nodes[c2][s]
                        idxs.append(s)
                        distance.append(utils.getDistance(son.wTab.loc, father.wTab.loc))
                    # -------------------------------------------------------------
                    # 将距离进行排序（递增）
                    order = [i for i in range(len(idxs))]
                    order = [k for k, _  in sorted(list(zip(order, distance)), key=lambda pair:pair[1])]
                    # 取前K个节点作为子节点
                    count = len(idxs)
                    # if K != -1 and count > K: count = K
                    for j in range(count):
                        self.nodes[c2][idxs[order[j]]].addForward(father)   # dtype = TreeNode
                    # -------------------------------------------------------------
                # end for c2 in range(1, 8)
                # ---------------------------------------------------------------
            # end for i in range(len(self.nodes[c]))
            # ------------------------------------------------------------------
        # end for c in range(9, 3, -1)
        # --------------------------------------------------------------------
        ## 针对不同地图类型设置参数
        if INFO.MAP_NUMBER == 1: # 地图1使用了两个7
            for i in range(len(self.nodes[1])):
                if self.nodes[7][i].wTab_id == 11:  # dtype = TreeNode
                    # -------------------------------------------------------------
                    for c in range(4, 7):   # 连接已有的4，5，6
                        for j in range(len(self.nodes[c])):
                            if len(self.nodes[c][j].forward) == 0: continue
                            self.nodes[c][j].addForward(self.nodes[7][i])
                    # -------------------------------------------------------------
                    for j in range(len(self.nodes[7])): # 连已有的8
                        if len(self.nodes[7][j].forward) == 0: continue
                        self.nodes[7][i].addForward(self.nodes[7][j].forward[0].target)
                    # -------------------------------------------------------------
        # end if INFO.MAP_NUMBER == 1
        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # 2.将选中的节点放入填充查询表（有链接的节点）
        self.searchTab = [None for _ in range(len(wTabs))]
        for layer in range(8):  # 把节点对应的地址放进去
            for j in range(len(self.nodes[layer])): # 首选节点
                # 跳过没有path链接的非顶层节点，第9层虽然也没有path，但是不用连，因为索引的时候用不着
                if len(self.nodes[layer][j].forward) == 0 and not (b_top_layer & (0x01 << self.nodes[layer][j].class_id)):
                    continue
                self.searchTab[self.nodes[layer][j].wTab_id] = self.nodes[layer][j]
        # --------------------------------------------------------------------
        # 打印树关系
        log.LogRecord("[Tree][initialize] Node Tree:")
        for layer in range(9, 0, -1):
            log.LogRecord('[Tree][initialize][layer {}]'.format(layer))
            for a in range(len(self.nodes[layer])):
                for b in range(len(self.nodes[layer][a].forward)):
                    p_path:workPath = self.nodes[layer][a].forward[b]
                    string = '\t [Paths] classID:{} --- begin: {} / {} -> target: {} / {} \tpath_value: {}'.format(
                        layer, p_path.begin.wTab.classID, p_path.begin.wTab.ID,
                        p_path.target.wTab.classID, p_path.target.wTab.ID,
                        p_path.est_price / p_path.time_path_f)
                    log.LogRecord(string)
        # --------------------------------------------------------------------
        return None


    def getBestNode(self, loc:list, angle:float=-1.0, forecastStep:int=0,
                    last_frame:int=999999, offset_frame:int=0):
        """
        寻找该坐标对应的最佳节点
        :param loc: 机器人的坐标
        :param angle: 机器人的角度，-1则为不使用角度预测
        :param forecastStep: 预测的步长
        :param last_frame: 剩余的帧数，若时间不够完成则不会选择
        :param offset_frame: 时间偏移(用于预测，调用的时候默认0即可)
        :return: 其他，最佳路径workPath;NULL，找不到
        """
        best = None     # 最佳路径
        # 最大获益比，最佳方案所需等待时间，最佳方案的路程时间
        best_value, best_wait_f, best_dist2node_f = 0.0, 999999.9, 0.0
        # --------------------------------------------------------------------
        for i in range(len(self.searchTab)):
            p_node = self.searchTab[i]      # dtype = TreeNode
            if not p_node or p_node.locked_next or (p_node.wTab.sta_produce == 0 and p_node.wTab.last == -1)\
                    or (p_node.locked and INFO.MAP_NUMBER == 1 and not (self.b_layerClass[3] & (0x01 << p_node.class_id)))\
                    or (p_node.locked and INFO.MAP_NUMBER != 1)\
                    or (p_node.putLock and 2 < utils.getDistance(loc, p_node.wTab.loc)):
                    # 有其他机器人准备来这里放东西，且我不在这个位置，让他来这个节点就好
                continue
            # if not p_node:
            #     log.LogRecord('[Tree][GetBestNode] node {} is None.'.format(i))
            #     continue
            # if p_node.locked_next:
            #     log.LogRecord('[Tree][GetBestNode] node {} is locked_next.'.format(i))
            #     continue
            # if p_node.wTab.sta_produce == 0 and p_node.wTab.last == -1:
            #     log.LogRecord('[Tree][GetBestNode] node {} have not produce.'.format(i))
            #     continue
            # if p_node.locked and INFO.MAP_NUMBER == 1 and not (self.b_layerClass[3] & (0x01 << p_node.class_id)):
            #     log.LogRecord('[Tree][GetBestNode] node {} have not produce.'.format(i))
            #     continue
            # if p_node.locked and INFO.MAP_NUMBER != 1:
            #     log.LogRecord('[Tree][GetBestNode] node {} have not produce.'.format(i))
            #     continue
            # if p_node.putLock and 2 < utils.getDistance(loc, p_node.wTab.loc):
            #     # 有其他机器人准备来这里放东西，且我不在这个位置，让他来这个节点就好
            #     log.LogRecord('[Tree][GetBestNode] node {} put lock.'.format(i))
            #     continue
            # ------------------------------------------------------------------
            # 计算 路程时间 和 等待时间
            ## 机器人到达该节点所需时间（帧）
            dist2node_f = utils.getDistance(loc, p_node.wTab.loc) / maxSpeed_sec * 50
            waitTime_f = 0      ## 等待原材料的时间
            if not p_node.wTab.sta_produce:
                waitTime_f = p_node.wTab.last - offset_frame - dist2node_f
            if p_node.locked:   # 第一份是别人的，要加一份生产时间
                waitTime_f += INFO.TAB_WTAB_OUTPUT_F[p_node.class_id]
            if waitTime_f < 0: waitTime_f = 0
            ## 时间还多的时候不允许在没有产品的工作台等待超过1秒
            if waitTime_f > 2*50 and not p_node.wTab.sta_produce and last_frame > 5*50:
                continue
            # ------------------------------------------------------------------
            # 寻找获益最大的路径
            for path in range(len(p_node.forward)):
                p_path = p_node.forward[path]   # dtype = TreeNode
                if not p_path.target.getPutOK(p_node.class_id, True):   # 不可用
                    # log.LogRecord('[Tree][GetBestNode] node {}/{} can not put produce {}/{}.'.format(
                    #     p_path.target.class_id, p_path.target.wTab_id, p_node.class_id, p_node.wTab_id))
                    continue
                if p_path.target.getWantPutCnt() >= 2:  # 太多人要去了，不去
                    # log.LogRecord('[Tree][GetBestNode] node {} has over 2 robots.'.format(p_path.target.wTab_id))
                    continue
                # ---------------------------------------------------------------
                turn_f = 0.0    # 转弯时间
                if angle != -1:
                    turn_f = utils.angelSub(p_path.angle-angle) / maxSpeed_sec * 50
                # 总时间 = 前往时间 + 路程时间 + 等待时间 + 转弯时间
                wholeTime = dist2node_f + p_path.time_path_f + waitTime_f + turn_f
                ## 剩余时间不够执行完该任务（在预估时间上再加时间，防止误差）
                if wholeTime*1.1 > last_frame: continue
                # 计算该路径带来的获益比
                profit = getPathProfit(p_path)
                # 鼓励合成当前还没有的东西
                if self.p_extern_material[p_path.target.class_id] == 0:
                    profit *= 1.1
                profit2 = 0
                # ---------------------------------------------------------------
                #  往后预测
                if forecastStep:
                    # 由于8以后就没有物品了，所以用本路径获益来算
                    if p_path.target.class_id == 8 or p_path.target.class_id == 9:
                        profit2 = profit
                    else:
                        p_nextPath = self.getBestNode(p_path.target.wTab.loc, p_path.angle, forecastStep - 1, last_frame - wholeTime, wholeTime)
                        if p_nextPath: profit2 = getPathProfit(p_nextPath)
                # end if forecastStep
                # ---------------------------------------------------------------
                profit = (profit + factor_forecast * profit2) / wholeTime
                if (profit > best_value) and ((dist2node_f < best_wait_f + best_dist2node_f) or (dist2node_f + waitTime_f < best_wait_f)):
                    best_value = profit # 最佳获益比
                    best_wait_f = dist2node_f + waitTime_f  # 最佳时间
                    best_dist2node_f = dist2node_f
                    best = p_path   # 最佳路径对应的结构体
            # end for 寻找获益最大的路径
            # ------------------------------------------------------------------
        # end for i in range(len(self.searchTab))
        # --------------------------------------------------------------------
        return best


class workPath(object): # 前向传递路径
    def __init__(self, begin:TreeNode, target:TreeNode):
        self.angle = -1.0   # 角度（起点->终点）
        self.est_price = 0  # 预测获利
        self.begin = begin
        self.target = target
        self.distance = utils.getDistance(begin.wTab.loc, target.wTab.loc)# 距离
        # 走完路径所需最少时间（帧）
        self.time_path_f = self.distance / maxSpeed_sec * 50
        self.angle = math.atan2(target.wTab.loc[1]-begin.wTab.loc[1], target.wTab.loc[0]-begin.wTab.loc[0])
        self.est_price = utils.valFunc(INFO.TAB_OBJ_SELL[begin.class_id], self.time_path_f) - INFO.TAB_OBJ_BUY[begin.class_id]

    def updatePrice(self, price_final):
        """
        路径价值更新
        :param price_final: 结束时的价值
        :return:None
        """
        self.est_price += (price_final - self.est_price) * PATH_LR
        return None


# other function
def getPathProfit(path:workPath) -> float:
    """
    计算路径利润
    :param path: 路径对象
    :return: 利润
    """
    begin, end = path.begin, path.target
    profit = path.est_price  # 计算获利=收入-支出
    # 统计售卖的合成台物品数量
    obj_cnt = 0
    binary_objs = end.wTab.sta_material
    for i in range(8):
        if binary_objs & (0x01 << i):
            obj_cnt += 1
    # 激励去有东西的地方合成
    if obj_cnt != 0 and obj_cnt < INFO.TAB_WTAB_INPUT_CNT[end.class_id] != 1:
        obj_cnt += 1
    # 计算最终的结果=利润* (添加后的物品数量 / 总物品数量)
    profit = profit * (obj_cnt + 1) / INFO.TAB_WTAB_INPUT_CNT[end.class_id]
    # -----------------------------------------------------------------------
    # 针对不同地图设置参数
    if INFO.MAP_NUMBER == 1:
        # 让456价值相同，防止一直合6
        if end.class_id == 4 or end.class_id == 5 or end.class_id == 6:
            profit = 50 * (obj_cnt + 1) / INFO.TAB_WTAB_INPUT_CNT[end.class_id]
        # 456快点送到7，7应该是未来还不能合成的
        if end.class_id != 11 and end.class_id == 7 and end.wTab.last == -1 and\
            (end.putLock | end.wTab.sta_material != INFO.TAB_WTAB_INPUT_CLASS[end.class_id]):
            profit*=1.5
        elif end.wTab.last == -1:
            profit *= 1.2
        if end.wTab.last > 0:
            profit *= 1 - (end.wTab.last / INFO.TAB_WTAB_OUTPUT_F[end.class_id])
    # -----------------------------------------------------------------------
    elif INFO.MAP_NUMBER == 2:
        if end.class_id == 5: profit *= 1.1
        elif end.class_id == 6: profit *= 1.6
        if begin.class_id == 6 and end.class_id == 7: profit *= 1.1 # 6快点送到7
        if end.class_id == 7 and not end.wTab.sta_produce: profit *= 1.2    # 保证7一直生产
    # -----------------------------------------------------------------------
    elif INFO.MAP_NUMBER == 4:
        if end.class_id == 4: profit *= 4   # 尽可能拿4
        if begin.class_id == 4 and end.class_id == 7:   # 4快点送到7
            profit *= 6
        if begin.class_id == 7 and not end.wTab.sta_produce:   # 456快点送到7
            profit *= 1.2
    # -----------------------------------------------------------------------
    return profit


def getScatteringRate(wTabs) -> float:
    """
    计算工作台的离散度
    :param wTabs: 工作台
    :return: 离散度
    """
    # 平均值
    aveX, aveY = 0.0, 0.0
    for i in range(len(wTabs)):
        aveX += wTabs[i].loc[0]
        aveY += wTabs[i].loc[1]
    aveX /= len(wTabs)
    aveY /= len(wTabs)
    # 计算方差
    sigmaX, sigmaY = 0.0, 0.0
    for i in range(len(wTabs)):
        sigmaX += math.pow(wTabs[i].loc[0] - aveX, 2)
        sigmaY += math.pow(wTabs[i].loc[1] - aveY, 2)

    return sigmaX*0.5 + sigmaY*0.5


def lockNode(node:TreeNode):
    """
    节点加锁
    :param node: 要加锁的节点
    :return: None
    """
    if node:
        if not node.locked: node.locked = True
        else: node.locked_next = True
    return None


def lockPath(path:workPath):
    """
    路径加锁
    :param path: 要加锁的路径
    :return:
    """
    if path:
        lockNode(path.begin)
        path.target.lock(path.begin.class_id)
    return None


def unlockNode(node:TreeNode):
    """
    节点解锁
    :param node: 要解锁的节点
    :return: None
    """
    if node:
        if node.locked_next:    # 第二份产品被要了，解第二份的锁
            node.locked_next = False
        else:   # 第二份产品没有被要，解第一份的锁
            node.locked = False
    return None


def unlockPath(path:workPath):
    """
    路径解锁
    :param path: 要解锁的路径
    :return: None
    """
    if path:
        unlockNode(path.begin)
        path.target.unlock(path.begin.class_id)
    return None

