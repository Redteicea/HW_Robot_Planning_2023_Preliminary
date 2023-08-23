import math
from typing import List

from ORCA import Agent
from vectorFunc import Vector
from UsefulTools import LOG
from info_c import INFO_C, utils

ROBOT_MAX_V = 6 # 机器人的最大速度
ROBOT_MAX_SCAN_R = 10  # 最大检测范围

log = LOG('Controller')
log.LogInitialize()
INFO = INFO_C()

class Controller(object):
    def __init__(self, robots, worktables=None):
        self.robots = robots  # 4个机器人
        self.obj_worktable = [-1 for _ in range(4)] # 各个机器人对应的目标工作台ID
        self.worktable_loc = [[-1,-1] for _ in range(4)]    # 目标工作台对应的坐标
        self.worktable_loc_List = [[-1,-1] for _ in range(4)]    # 目标工作台对应的坐标
        # --------------------------------------------------------------------
        self.target_dir = [-1 for _ in range(4)] # 目标朝向[-180,180]
        self.target_rot = [-1 for _ in range(4)] # 目标角速度[-π,π]
        self.target_lineV = [-1 for _ in range(4)] # 目标速度[-2,6]
        # --------------------------------------------------------------------
        self.colli_obj_loc = [[-1,-1] for _ in range(4)]    # 需要进行碰撞规避时的目标坐标，不需要规避时等于工作台坐标
        self.colli_dir = [-1 for _ in range(4)] # 检测到碰撞后的矫正方向
        self.colli_lineV = [-1 for _ in range(4)] # 检测到碰撞后的线速度大小
        # --------------------------------------------------------------------
        self.agents:List[Agent] = []    # 用于自动避障的四个智能体
        self.initialize(robots, worktables)


    def initialize(self, robots, worktables=None):
        for idx in range(4):
            self.obj_worktable[idx] = -1
            self.worktable_loc[idx] = robots[idx].loc
        if worktables:
            for idx in range(len(worktables)):
                self.worktable_loc_List.append(worktables[idx].loc)
        else:
            self.avoid_init()


    def setObjectloc(self, RobotID, worktableID, location):
        """
        设置目标工作台坐标
        :param RobotID: 机器人ID, [0,3]
        :param worktableID: 目标工作台ID
        :param location: 工作台坐标
        :return: None
        """
        self.obj_worktable[RobotID] = worktableID
        self.worktable_loc[RobotID] = location
        return None


    def isArrived(self, robots, order)->bool:
        """
        判断是否到达目标工作台，并执行买卖操作
        :param robots: 要判断的机器人
        :param order: 要执行的买卖操作，若未到达目标工作台则order为空
        :return: 若已到达返回true，反之返回false
        """
        order.clear()
        # 尚未到达目标工作台位置，返回False
        if robots.worktableID != self.obj_worktable[robots.ID]:
            return False
        if robots.objectID == 0:    # 没有携带物品，则来这购买
            order.append('ACT_BUY')
        else:   # 携带了物品，则出售该物品
            order.append('ACT_SELL')
        return True


    def Control(self, order, intensity, robotID) -> None:
        """
        返回机器人的控制指令，计算控制指令前要先调用update_robots函数更新机器人状态信息
        :param order: 控制指令
        :param intensity: 控制参数
        :param robotID: 机器人ID
        :return: None
        """
        order.clear()
        intensity.clear()
        # --------------------------------------------------------------------
        robot_v = self.agents[robotID].bestVec * 1.2  # 机器人智能体速度
        robot_dir = self.robots[robotID].direct    # 机器人朝向
        dx, dy = robot_v.x, robot_v.y
        robot_tar_dir, M_distance = 0.0, 0.0
        # --------------------------------------------------------------------
        if abs(dx) < 0.001:     # x 接近0，相当于垂直线，角度为 ±90°
            if dy > 0: robot_tar_dir = INFO.PI / 2  # pi / 2，90°
            elif dy <0: robot_tar_dir = -INFO.PI / 2
        else:
            robot_tar_dir = math.atan2(dy, dx)  # 将斜率转换成角度
        # --------------------------------------------------------------------
        M_distance = math.sqrt(dx * dx + dy * dy)
        self.target_dir[robotID] = robot_tar_dir * 180 / INFO.PI    # 转换成弧度
        # --------------------------------------------------------------------
        dir_err = robot_tar_dir - robot_dir     # 机器人朝向的误差
        if dir_err > INFO.PI: dir_err = dir_err - 2 * INFO.PI # 等效到 -PI 到 PI 之间
        elif dir_err < -INFO.PI: dir_err = dir_err + 2 * INFO.PI
        # --------------------------------------------------------------------
        vec_descend = abs(dir_err) * 15 / INFO.PI
        V_tar = ROBOT_MAX_V - utils.limit(vec_descend, 0, 5) - utils.limit((1 - self.agents[robotID].bestVec.mod()) * 2, 0, 2)
        # --------------------------------------------------------------------
        V_tar = utils.limit(V_tar, -2, 6)
        Rot_tar = utils.limit(dir_err * 6, -INFO.PI, INFO.PI)
        # --------------------------------------------------------------------
        # if robotID == 0:
        #     log.LogRecord('[Controller][robot {}] best_v:{}'.format(robotID, [robot_v.x, robot_v.y]))
        #     log.LogRecord('[Controller][robot {}] tar_dir:{}, org_dir:{}'.format(robotID, robot_tar_dir, robot_dir))
        #     log.LogRecord('[Controller][robot {}] cur_loc:{}, tar_loc:{}'.format(robotID, self.robots[robotID].loc, self.worktable_loc[robotID]))
        #     log.LogRecord('[Controller][robot {}] V_tar:{}, Rot_tar:{}'.format(robotID, V_tar, Rot_tar))
        # --------------------------------------------------------------------
        # 检测是否有机器人在目标点，若有则停下排队
        horizon_thre, workTab_thre, thre = 5.0, 2.0, 0.0
        robotLoc = self.robots[robotID].loc
        # 针对不同地图设置
        if INFO.MAP_NUMBER == 1:
            workTab_thre = 2.5
        elif INFO.MAP_NUMBER == 2:
            workTab_thre = 2
        elif INFO.MAP_NUMBER == 3:
            workTab_thre = 3
        elif INFO.MAP_NUMBER == 4:
            workTab_thre = 3
        # --------------------------------------------------------------------
        # 在边界附近，范围要大一些
        if robotLoc[0] < horizon_thre or robotLoc[0] > 50-horizon_thre or\
            robotLoc[1] < horizon_thre or robotLoc[1] < 50-horizon_thre:
            thre = horizon_thre
        # 不在边界附近，范围要适当小一些
        else:
            thre = workTab_thre
        # --------------------------------------------------------------------
        # 前方有机器人，减速
        if thre > utils.getDistance(self.worktable_loc[robotID], self.robots[robotID].loc):
            # 朝向目标点的方向
            toCenterVec = utils.loc2vec(self.worktable_loc[robotID]) - utils.loc2vec(self.robots[robotID].loc)
            # 如果是靠近目标点，则判断是否要停住
            if toCenterVec.dot(robot_v) > 0.0:
                for idx in range(4):
                    if robotID == idx or thre < utils.getDistance(self.robots[robotID].loc, self.robots[idx].loc):
                        continue
                    # 相对位置
                    relativeLoc = utils.loc2vec(self.robots[idx].loc) - utils.loc2vec(robotLoc)
                    # 点乘检查是否同方向
                    dotLoc = relativeLoc.dot(robot_v)
                    if dotLoc > 0.0: V_tar *= 0.35; break
        # --------------------------------------------------------------------
        # 无携带物品且前边是边界，说明要去边界拿东西，则减速
        dist2target = abs(utils.getDistance(self.worktable_loc[robotID], self.robots[robotID].loc) - 0.4)
        if robotLoc[0] < 2 or robotLoc[0] > 48 or robotLoc[1] < 2 or robotLoc[1] > 48:  # 在边界
            # 朝向中心点的方向
            toCenterVec = Vector(25, 25) - utils.loc2vec(self.robots[robotID].loc)
            # 速度远离中心点，则减速防止撞墙
            speed = Vector(self.robots[robotID].v_line[0], self.robots[robotID].v_line[1])
            if toCenterVec.dot(Vector(math.cos(self.robots[robotID].direct),
                                    math.sin(self.robots[robotID].direct))) < 0.0 and\
                                    dist2target < speed.mod() * 5 / 50:
                V_tar /= 10
            # 角度差过大则原地转圈
            if utils.angelSub(robot_tar_dir - self.robots[robotID].direct) > INFO.PI/3:
                V_tar = -0.0
        # --------------------------------------------------------------------
        order.append('ACT_FORWARD')
        intensity.append(V_tar)
        order.append('ACT_ROTATE')
        intensity.append(Rot_tar)
        # if robotID == 0:
        #     log.LogRecord('[Controller][robot {}] V_tar:{}, Rot_tar:{}'.format(robotID, V_tar, Rot_tar))
        return None


    def avoid_init(self) -> None:
        """
        初始化避障算法智能体
        :return: None
        """
        RADIUS = 0.53 + 0.15    # 机器人半径
        scanRad = 3     # 扫描半径
        maxSpeed = 6    #最大速度
        timeHorizon = 2     # 避障时间阈值
        timeStep = 1 / 50   # 帧步长

        # 针对不同地图设置
        if INFO.MAP_NUMBER == 2:
            RADIUS = 0.53 + 0.1
            scanRad = 5
            timeHorizon = 10
        elif INFO.MAP_NUMBER == 3:
            RADIUS = 0.53 + 0.1
            scanRad = 3.6
            timeHorizon = 10
        elif INFO.MAP_NUMBER == 4:
            RADIUS = 0.53 + 0.1
            scanRad = 5
            timeHorizon = 15

        for idx in range(4):
            self.agents.append(Agent(self.robots[idx], scanRad, timeHorizon))
        return None


    def avoid_update(self) -> None:
        """
        避障更新，更新后的值在m_agents[i]里
        :return: None
        """
        for idx in range(4):
            # 目标速度向量，朝向工作台即为最近路径
            goalVec = utils.loc2vec(self.worktable_loc[idx]) - utils.loc2vec(self.robots[idx].loc)
            if goalVec.mod() > 6.0: # 若大于最大速度，则裁剪到最大速度
                goalVec = goalVec.normal() * 6
            # self.agents[idx].update(self.robots[idx], utils.loc2vec(self.worktable_loc[idx]))
            self.agents[idx].update(self.robots[idx], goalVec)
        for idx in range(4):
            # 检测周围机器人
            self.agents[idx].nearAgents.clear()
            for idx2 in range(4):
                if idx == idx2: continue
                dist = utils.getDistance(self.robots[idx].loc, self.robots[idx2].loc)
                # 在检测范围外则不采用
                if dist > self.agents[idx].scanR: continue
                # 在检测范围内则加入列表
                self.agents[idx].nearAgents.append(self.agents[idx2])
            # 计算新值
            self.agents[idx].ORCA()
        return None
