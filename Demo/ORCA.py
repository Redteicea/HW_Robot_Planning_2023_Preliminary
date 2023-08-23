import copy
import math
from typing import List

from vectorFunc import Vector
from info_c import INFO_C
from UsefulTools import LOG

log = LOG('ORCA')
log.LogInitialize()
INFO = INFO_C()

MAX_VEC = 6     # 最大线速度
MAX_W = INFO.PI     # 最大角速度
TAU = 1 / 50      # 帧时间
EPSILON = 0.00001

class Agent(object):
    def __init__(self, robot, scanR, timeHorizon):
        self.nearAgents = list()    # 附近的机器人
        self.loc = Vector(robot.loc[0], robot.loc[1])     # 位置
        self.vec = Vector(0, 0)     # 速度
        self.scanR = scanR      # 扫描范围
        self.timeHorizon = timeHorizon    # 避障时间阈值
        self.ideaVec = Vector(0, 0)     # 理想速度
        self.bestVec = Vector(0, 0)     # 最优速度
        self.radius = 0.53 + 0.1     # 机器人圆柱体半径


    def update(self, robot, targetVec):
        """
        更新机器人坐标、速度及理想速度
        :param robot: 机器人对象
        :param targetVec: 指向目标位置的速度向量
        :return: None
        """
        self.loc = Vector(robot.loc[0], robot.loc[1])
        self.vec = Vector(robot.v_line[0], robot.v_line[1])

        # targetVec = targetLoc - self.loc
        # if targetVec.mod() > MAX_VEC:
        #     targetVec = targetLoc.normal() * MAX_VEC

        self.ideaVec = targetVec
        return None


    def VO(self, agent):
        """
        VO避障算法
        :param agent: 障碍对象
        :return: a Vector 速度矢量
        """
        dx = agent.loc.x - self.loc.x
        dy = agent.loc.y - self.loc.y

        dir_AB = Vector(dx, dy)     # 本体指向障碍对象的向量
        dir_rotate = dir_AB.rotate(INFO.PI / 2)  # 垂直矢量

        if dir_rotate.dot(self.vec) < 0:
            dir_rotate *= -1

        return dir_AB + dir_rotate * (self.radius + agent.radius/2)


    def ORCA(self):
        """
        ORCA避障算法
        :return: a Vector 速度矢量
        """
        orcaLines = list()  # 算出来的半平面切割线集合
        invTimeHorizon:float = 1.0 / self.timeHorizon
        # --------------------------------------------------------------------
        for obj_idx in range(len(self.nearAgents)):
            relativePos:Vector = self.nearAgents[obj_idx].loc - self.loc   # 相邻机器人相对本机器人的位置
            relativeVec:Vector = self.vec - self.nearAgents[obj_idx].vec   # 相对速度
            relateDistSq:float = relativePos.powMod()     # 相对距离的平方
            combinedRadius:float = self.radius + self.nearAgents[obj_idx].radius   # 半径和
            combinedRadiusSq:float = combinedRadius ** 2  # 半径和的平方

            lineDirection = Vector(-1, -1)
            # ------------------------------------------------------------------
            # 相对距离的平方比两个圆柱体的半径和还大，两者还没碰撞
            if relateDistSq > combinedRadiusSq:
                # 相对速度对于发生碰撞的临界速度余量
                # 其中（invTimeHorizon * relativePos）是根据碰撞时间阈值计算的平均速度
                w:Vector = relativeVec - relativePos * invTimeHorizon
                wLengthSq = w.powMod()    # 发生碰撞的临界速度余量的平方值
                dotProduct1 = w.dot(relativePos)    # 向量在碰撞方向的投影

                # case 1:  当前速度相对碰撞速度的余量 在 碰撞方向的投影小于0 -> 目前的速度还没到碰撞需要的速度
                #               所以其在速度空间中的位置应该位于障碍区域的前方
                # case 2: 同时开根，公式变为 dotProduct1 > combinedRadius * wLength
                #               消掉两边的length -> |大圆中心到w的投影| > |combinedRadius|
                #               若w与两边的其中一条射线垂直，此时两边取等号
                #               （圆本来有4条，但是向上的那两条不会与射线相交的被 case 1 排除了）
                # 此处if的意义是：VO是由两条射线和半圆周组合起来的，用if把它们分为半圆部分和两条射线部分
                if dotProduct1 < 0.0 and dotProduct1 ** 2 > combinedRadiusSq * wLengthSq:
                    wLength:float = math.sqrt(wLengthSq)  # 发生碰撞的临界速度余量
                    unitW:Vector = w / wLength     # 碰撞的临界速度余量的平均值
                    lineDirection:Vector = Vector(unitW.y, -unitW.x)
                    u = unitW * (combinedRadius * invTimeHorizon - wLength)
                else:   # 两条射线部分
                    # 用勾股定理算大圆心到腿的距离
                    leg = math.sqrt(relateDistSq - combinedRadiusSq)    # 临界碰撞的线段长度
                    # 构建行列式：第一列->relativePos，第二列->w
                    # 求行列式，可以根据行列式正负，判断第一列向量到第二列向量的旋转方向(180以内)
                    # 大于0是逆时针旋转，小于0是顺时针旋转
                    # 以此可以判断左腿右腿
                    if relativePos.det(w) > 0.0:
                        # 设左腿与x轴负方向的夹角为θ，则：
                        # leg / relativePositionLength -> cosθ
                        # combinedRadius / relativePositionLength -> sinθ
                        # 可得：把 relativePositionLength 沿原点逆时针旋转黄金三角型下方角度所得到的向量
                        lineDirection = Vector(relativePos.x  * leg - relativePos.y * combinedRadius,
                                               relativePos.x * combinedRadius + relativePos.y * leg) / relateDistSq
                    else:
                        lineDirection = -Vector(relativePos.x  * leg - relativePos.y * combinedRadius,
                                               -relativePos.x * combinedRadius + relativePos.y * leg) / relateDistSq
                    # 当前速度到腿的垂线
                    u = lineDirection * relativeVec.dot(lineDirection) - relativeVec
            # ------------------------------------------------------------------
            # 已经碰撞，两圆柱体存在重叠部分
            else:
                invTimeStep = 1.0  / TAU    # 帧间隔
                # 小圆中心到当前速度的向量(即下一帧位置)
                w = relativeVec - relativePos * invTimeStep
                wLength = w.mod()   # 模长
                unitW = w / wLength     # 碰撞的临界速度余量的平均值(单位向量)
                lineDirection = Vector(unitW.y, -unitW.x)   # 向量顺时针旋转90°
                u = unitW * (combinedRadius * invTimeStep - wLength)    # 到VO区域外的最短向量
            # end if (relateDistSq < combinedRadiusSq) else
            # ------------------------------------------------------------------
            linePoint = self.vec + u * 0.5
            orcaLines.append([linePoint, lineDirection])
        # end for obj_idx
        # --------------------------------------------------------------------
        # 求解
        lineFail = calLinear2(orcaLines, MAX_VEC, self.ideaVec, False, self.bestVec)
        if lineFail < len(orcaLines):
            calLinear3(orcaLines, lineFail, MAX_VEC, self.bestVec)
        # 更新数据
        self.vec = self.bestVec
        self.loc += self.vec * TAU  # 机器人下一帧的位置
        return self.bestVec


def calLinear1(lines:list, line_idx:int, radius:float, vec_opt:Vector,
               flag_optDirect:bool, result:Vector) -> bool:
    """
    解一维线性问题
    :param lines:
    :param line_idx:
    :param radius:
    :param vec_opt:
    :param flag_optDirect:
    :param result:
    :return:
    """
    dotProduct:float = lines[line_idx][0].dot(lines[line_idx][1])
    discriminant:float = dotProduct ** 2 + radius ** 2 - lines[line_idx][0].powMod()
    if discriminant < 0.0: return False
    discriminant_sq:float = math.sqrt(discriminant)
    tLeft = -dotProduct - discriminant_sq
    tRight = -dotProduct + discriminant_sq
    # -----------------------------------------------------------------------
    for idx in range(line_idx):
        denominator = lines[line_idx][1].det(lines[idx][1])
        numerator =lines[idx][1].det(lines[line_idx][0] - lines[idx][0])

        if math.fabs(denominator) <= EPSILON:    # 平衡线，返回False
            if numerator < 0.0: return False
            continue

        t = numerator / denominator
        if denominator >= 0.0:  # 在左侧
            tRight = min(tRight, t)
        else:   # 在右侧
            tLeft = max(tLeft, t)

        if tLeft > tRight: return False
    # end for idx in range(line_idx)
    # -----------------------------------------------------------------------
    if flag_optDirect:  # 优化方向
        if vec_opt.dot(lines[line_idx][1]) > 0.0:   # 右侧
            res = lines[line_idx][0] +  lines[line_idx][1] * tRight
        else:   # 左侧
            res = lines[line_idx][0] + lines[line_idx][1] * tLeft
    else:   # 不优化方向，最近点
        t = lines[line_idx][1].dot(vec_opt - lines[line_idx][0])
        if t < tLeft:
            res = lines[line_idx][0] + lines[line_idx][1] * tLeft
        elif t > tRight:
            res = lines[line_idx][0] + lines[line_idx][1] * tRight
        else:
            res = lines[line_idx][0] + lines[line_idx][1] * t
    # end of 优化方向
    # -----------------------------------------------------------------------
    result.copyFrom(res)
    return True


def calLinear2(lines:List[List[Vector]], radius:float, vec_opt:Vector, flag_optDirect:bool, result:Vector) -> int:
    """
    解二维线性问题
    :param lines:
    :param radius:
    :param vec_opt:
    :param flag_optDirect:
    :param result:
    :return:
    """
    if flag_optDirect:
        result.copyFrom(vec_opt * radius)
    elif vec_opt.powMod() > radius ** 2:
        result.copyFrom(vec_opt.normal() * radius)
    else:
        result.copyFrom(vec_opt)

    for idx in range(len(lines)):
        if lines[idx][1].det(lines[idx][0] - result) > 0.0:
            tmpResult = result
            if not calLinear1(lines, idx, radius, vec_opt, flag_optDirect, result):
                result.copyFrom(tmpResult)
                return idx
    return len(lines)


def calLinear3(lines:List[List[Vector]], begin:int, radius:float, result:Vector) -> None:
    """
    解三维线性问题
    :param lines:
    :param begin:
    :param radius:
    :param result:
    :return: None
    """
    dist = 0.0
    # -----------------------------------------------------------------------
    for idxi in range(begin, len(lines)):
        if lines[idxi][1].det(lines[idxi][0] - result) > dist:
            proLines = list()
            line = [Vector(0, 0), Vector(0, 0)]
            # ------------------------------------------------------------------
            for idxj in range(idxi):
                determinant:float = lines[idxi][1].det(lines[idxj][1])
                if math.fabs(determinant) <= EPSILON:   # 平行
                    if lines[idxi][1].dot(lines[idxj][1]) > 0.0: continue  # 同向跳过
                    line[0] = (lines[idxi][0] + lines[idxj][0]) * 0.5
                else:   # 不平行
                    line[0] = lines[idxi][0] + lines[idxi][1] * (lines[idxj][1].det(lines[idxi][0] - lines[idxj][0]) / determinant)
                line[1] = (lines[idxj][1] - lines[idxi][1]).normal()
                proLines.append(line)
            # end for for idxj in range(idxi)
            # ------------------------------------------------------------------
            tmpResult = result
            if calLinear2(proLines, radius, Vector(-lines[idxi][1].y, lines[idxi][1].x), True, result) < len(proLines):
                result.copyFrom(tmpResult)
            dist = lines[idxi][1].det(lines[idxi][0] - result)
    # end for idxi in range(begin, len(lines))
    # -----------------------------------------------------------------------
    return None