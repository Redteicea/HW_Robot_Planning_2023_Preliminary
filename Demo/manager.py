import copy
import sys
import math
from typing import List
from UsefulTools import LOG
from info_c import INFO_C, Robot, WorkTable

log = LOG('Manager')
log.LogInitialize()
INFO = INFO_C()

class Manager(object):
    def __init__(self, mapStr):
        # 帧ID
        self.frameId = 0
        # 场上存在物品的数量(1~7)，
        # 此数量包括放在工作台里的 和 即将生产的；[0][8][9]为占位用
        self.extern_materia = [0]*10
        # 场上可用的材料（1~7），此数量只有可以拿取的物品
        self.extern_materia_available = [0]*10
        # 当前金钱数
        self.curCoin = 0
        # 工作台数量
        self.amount_workTable = 0
        # 工作台
        self.worktables:List[WorkTable] = list()
        # 四台机器人
        self.robots = [Robot(), Robot(), Robot(), Robot()]
        # 每个机器人的目标工作台
        self.targetWtab = [[] for _ in range(4)]
        # 控制指令
        self.ctrlStr = list()
        self.initialize(mapStr)


    def initialize(self, mapStr):
        cnt_robot, cnt_wTable = 0, 0
        for i in range(len(mapStr)):   # 按行读取地图信息
            obj = mapStr[i]
            for j in range(len(obj)):
                ele = obj[j]
                if ele == '.':  # 空的
                    continue
                elif ele == 'A':    # 机器人
                    self.robots[cnt_robot].ID = cnt_robot
                    self.robots[cnt_robot].loc[0] = float(j)*0.5 + 0.25
                    self.robots[cnt_robot].loc[1] = float(99-i)*0.5 + 0.25
                    cnt_robot += 1
                elif '1' <= ele <= '9': # 工作台
                    wTable = WorkTable()
                    wTable.ID = cnt_wTable
                    wTable.classID = int(ele)
                    wTable.loc[0] = float(j)*0.5 + 0.25
                    wTable.loc[1] = float(99-i)*0.5 + 0.25
                    self.worktables.append(wTable)
                    cnt_wTable += 1
                    self.amount_workTable = cnt_wTable
        return None


    def update(self, frameStr):
        """
        获取并更新帧信息
        @:param frameStr:帧信息字符串
        @:return None
        """
        # log.LogRecord('[Manager][update] updating current frame^s state')
        for idx in range(1, 8):
            self.extern_materia[idx] = 0
            self.extern_materia_available[idx] = 0
        # --------------------------------------------------------------------
        # 第1行：帧ID、当前金币数
        lineStr = frameStr[0]
        parts = lineStr.split(' ')
        self.frameId = int(parts[0])
        self.curCoin = int(parts[1])
        # --------------------------------------------------------------------
        # 第2行：工作台数量
        wTableNum = int(frameStr[1].split(' ')[0])
        # --------------------------------------------------------------------
        # 第 3 到 3+wTableNum 行为对应的工作台信息
        for idx in range(2, 2+wTableNum):
            parts = frameStr[idx].split(' ')
            self.worktables[idx-2].classID = int(parts[0])
            self.worktables[idx-2].loc = [float(parts[1]), float(parts[2])]   # 坐标
            # 剩余生产帧数
            self.worktables[idx-2].last = int(parts[3])
            if self.worktables[idx-2].last >= 0: # 计数生产完成的及在原材料格里准备生产的
                self.extern_materia[self.worktables[idx-2].classID] += 1
            if self.worktables[idx-2].last == 0: # 生产完成，场上可用的材料+1
                self.extern_materia_available[self.worktables[idx-2].classID] += 1
            # 原材料格状态(二进制)
            self.worktables[idx-2].sta_material = int(parts[4])
            if self.worktables[idx-2].classID > 3:
                # 4-7 是加工工作台，需要收购指定的原材料才能生产
                for j in range(1, 8):
                    # 为该物品对应类型计数 +1
                    if self.worktables[idx-2].sta_material & 0x01 << j:
                        self.extern_materia[j] += 1
            # 产品格状态(0,1)
            self.worktables[idx-2].sta_produce = int(parts[5])
            if self.worktables[idx-2].sta_produce:  # 若产品格上有材料，则计数+1
                self.extern_materia[self.worktables[idx-2].classID] += 1
                self.extern_materia_available[self.worktables[idx-2].classID] += 1
        # --------------------------------------------------------------------
        # 最后4行是机器人的状态信息
        for idx in range(2+wTableNum, 2+wTableNum+4):
            parts = frameStr[idx].split(' ')
            rb_idx = idx - 2 - wTableNum
            self.robots[rb_idx].worktableID = int(parts[0]) # 所处工作台ID
            self.robots[rb_idx].objectID = int(parts[1]) # 携带的物品类型
            self.robots[rb_idx].val_time = float(parts[2]) # 时间价值系数
            self.robots[rb_idx].val_crash = float(parts[3]) # 碰撞价值系数
            self.robots[rb_idx].v_rad = float(parts[4]) # 角速度
            self.robots[rb_idx].v_line = [float(parts[5]), float(parts[6])] # 线速度
            self.robots[rb_idx].direct = float(parts[7]) # 机器人的正面朝向
            self.robots[rb_idx].loc = [float(parts[8]), float(parts[9])] # 机器人坐标
            self.robots[rb_idx].price_last = self.robots[rb_idx].price

            if self.robots[rb_idx].objectID != 0:  # 有携带物品
                # 更新价值为出售该物品可获得的价值
                self.robots[rb_idx].price = INFO.TAB_OBJ_SELL[self.robots[rb_idx].objectID] * \
                                            self.robots[rb_idx].val_time * self.robots[rb_idx].val_crash
            else:   # 无携带物品
                self.robots[rb_idx].price = 0
        return None


    def sortWorktable(self, robotID, tables):
        """
        根据机器人与工作台的距离，对工作台进行排序
        :param robotID: 机器人编号
        :param tables: 工作台对象 list(class)
        :return: table_quene: 排序完成的数组，按距离递增，为工作台的下标
        """
        table_quene, distance = list(), list()
        for idx, table in enumerate(tables):
            table_quene.append(idx)
            value = math.pow((table.loc[0] - self.robots[robotID].loc[0]), 2) + math.pow((table.loc[1] - self.robots[robotID].loc[1]), 2)
            distance.append(value)
        # 按机器人与table的距离进行递增排序
        table_quene = [k for k, _  in sorted(list(zip(table_quene, distance)), key=lambda pair:pair[1])]
        return table_quene


    def chooseWorktable(self, robotID):
        """
        选择机器人需要去的工作台
        :param robotID: 机器人编号
        :return: 工作台编号；-1，没有合适的
        """
        tables = copy.deepcopy(self.worktables)
        # --------------------------------------------------------------------
        # 先去除被其他机器人选中的工作台
        exclude = list()
        for rob_idx in range(4):
            for tab_idx in range(len(self.targetWtab[rob_idx])):
                exclude.append(self.targetWtab[rob_idx][tab_idx])
        exclude.sort()
        while len(exclude) > 0:
            tables.pop(exclude.pop())
        # --------------------------------------------------------------------
        tables_quene = self.sortWorktable(robotID, tables) # 获得距离排序后的数组
        # --------------------------------------------------------------------
        if self.robots[robotID].objectID > 0:   # 机器人携带了物品
            material_idx = 0x01 << self.robots[robotID].objectID    # 携带的具体物品
            # ------------------------------------------------------------------
            # 就近放置物品
            for tab_idx in tables_quene:
                # 该类型工作台需要的原材料类型是这个物品类型，且该工作台上没有这个原材料
                # 那么这个机器人的目标工作台就是这个工作台，直接返回其ID
                if (INFO.TAB_WTAB_INPUT_CLASS[tables[tab_idx].classID] & material_idx) and \
                    not (tables[tab_idx].sta_material & material_idx):
                    return tables[tab_idx].ID
            # ------------------------------------------------------------------
            # 若机器人所有的材料没有合适的工作台需要，则尝试投放到类型为9的工作台
            for tab_idx in tables_quene:
                if tables[tab_idx].classID == 9:
                    return tables[tab_idx].ID
        # --------------------------------------------------------------------
        else:   # 机器人没有携带物品
            # 寻找最近的且有材料的工作台
            for tab_idx in tables_quene:
                if tables[tab_idx].sta_produce == 1:
                    return tables[tab_idx].ID
        # --------------------------------------------------------------------
        return -1


    def control(self, robotID, action, intensity=0.0):
        """
        机器人行为控制
        :param robotID: 机器人编号
        :param action: 机器人动作
        :param intensity: 动作对应的操作值（如果需要的话），默认是不需要操作值
        :return: None
        """
        res = str()
        if action == 'ACT_FORWARD':
            res = 'forward %d %f\n' % (robotID, intensity)
        elif action == 'ACT_ROTATE':
            res = 'rotate %d %f\n' % (robotID, intensity)
        elif action == 'ACT_BUY':
            res = 'buy %d\n' % robotID
        elif action == 'ACT_SELL':
            res = 'sell %d\n' % robotID
        elif action == 'ACT_DESTROY':
            res = 'destroy %d\n' % robotID
        else:
            return None
        self.ctrlStr.append(res)
        return None


    def doActions(self):
        """
        发送控制指令
        :return:
        """
        # 先发送帧ID
        sys.stdout.write('%d\n' % self.frameId)
        # 发送机器人动作
        while len(self.ctrlStr) > 0:
            sys.stdout.write(self.ctrlStr.pop())
        # 最后是当前帧的标准输出
        sys.stdout.write('OK\n')
        sys.stdout.flush()
        return None