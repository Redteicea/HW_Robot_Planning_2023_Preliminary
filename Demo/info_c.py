import sys
import math
from vectorFunc import Vector


class INFO_C(object):
    def __init__(self):
        self.frame_end = 180 * 50  # 结束帧数
        self.PI = math.acos(-1)
        self.MAP_NUMBER = -1  # 地图序号
        # 工作台收购的原材料编号
        self.TAB_WTAB_INPUT_CLASS = (
            0,  # 占位，因为物品ID是从1开始算的
            0, 0, 0,
            0x06, 0x0a, 0x0c,  # [1, 2], [1, 3], [2, 3]
            0x70, 0x80, 0xfe,  # [4, 5, 6], [7], [1~7]
        )
        # 工作台收购的原材料数量
        self.TAB_WTAB_INPUT_CNT = (
            0,  # 占位，因为物品ID是从1开始算的
            0, 0, 0,
            2, 2, 2,  # [1, 2], [1, 3], [2, 3]
            3, 1, 1,  # [4, 5, 6], [7], [1~7]
        )
        # 工作台生产时间（帧）
        self.TAB_WTAB_OUTPUT_F = (
            0,  # 占位，因为物品ID是从1开始算的
            50, 50, 50,
            500, 500, 500,
            1000, 1, 1,
        )
        # 物品购入价格
        self.TAB_OBJ_BUY = (
            0,  # 占位，因为物品ID是从1开始算的
            3000, 4400, 5800,
            15400, 17200, 19200,
            76000
        )
        # 物品收购价格
        self.TAB_OBJ_SELL = (
            0,  # 占位，因为物品ID是从1开始算的
            6000, 7600, 9200,
            22500, 25000, 27500,
            105000
        )


# 机器人动作
class Actions(object):
    def __init__(self):
        self.ACT_FORWARD = 1  # 前进（米/秒）[-2, 6]
        self.ACT_ROTATE = 0  # 旋转速度（弧度/秒）[-π,π]，正值表示逆时针旋转
        self.BUY = 'ACT_BUY'  # 购买当前工作台的物品
        self.SELL = 'ACT_SELL'  # 出售当前物品给当前工作台
        self.DESTROY = 'ACT_DESTROY'  # 摧毁物品


# 机器人
class Robot(object):
    def __init__(self):
        self.ID = -1
        self.loc = [-1, -1]  # 机器人所处坐标
        self.price = 0  # 身上带的物品的价值
        self.price_last = 0  # 上一帧的物品价值
        self.worktableID = -1  # 所处工作台ID，-1表示没有
        self.objectID = 0  # 携带物品ID，0表示没有，[1,7]为对应物品
        self.val_time = 0.0  # 时间价值系数[0.8, 1]，0表示无物品
        self.val_crash = 0.0  # 碰撞价值系数[0.8, 1]，0表示无物品
        self.v_rad = 0.0  # 角速度（弧度/秒），正数表示顺时针
        self.v_line = [0.0, 0.0]  # 线速度（m/s）
        self.direct = 0.0  # 朝向[-π,π]；0，右方向；π/2，上方向


#  工作台
class WorkTable(object):
    def __init__(self):
        self.ID = -1
        self.classID = -1  # 类别[1,9]
        self.loc = [0, 0]  # 工作台所处坐标
        self.last = -1  # 剩余生产时间(帧数)；-1，没有生产；0，生产因输出格满而阻塞；>=0，剩余生产帧数
        self.sta_material = 0  # 原材料格状态，二进制表示；如 48(000110000) 表示拥有物品4和5
        self.sta_produce = 0  # 产品格状态；0，无；1，有

INFO = INFO_C()
# 工具函数
class utils(object):
    def __init__(self):
        pass

    @staticmethod
    def getDistance(point1, point2):
        # 计算两点距离
        return math.sqrt(math.pow(point2[0] - point1[0], 2) + math.pow(point2[1] - point1[1], 2))

    @staticmethod
    def factorFunc(x, maxX, minRate):
        # 计算物品的时间价值和碰撞价值系数（该公式由官方提供）
        if x < maxX:
            return (1 - math.sqrt(1 - math.pow(1 - (x / maxX), 2))) * (1 - minRate) + minRate
        else:
            return minRate

    @staticmethod
    def valFunc(price, time):
        # 计算价值（不计算碰撞）
        return price * utils.factorFunc(time, 9000, 0.8)

    @staticmethod
    def readUntilOK():
        """读取软件反馈的帧信息"""
        strings = list()
        while True:
            line = sys.stdin.readline()
            # 读取完成标志
            if line[:2] == 'OK': break
            strings.append(line)
        return strings

    @staticmethod
    def limit(value, minVal, maxVal):
        """
        裁剪函数，限制value的大小
        :param value: 目标裁剪值
        :param minVal: 下限
        :param maxVal: 上限
        :return: 裁剪后的值
        """
        if value < minVal: y = minVal
        elif value > maxVal: y = maxVal
        else: y = value
        return y

    @staticmethod
    def loc2vec(loc):
        """
        坐标转矢量
        :param loc: 输入坐标
        :return: 转换后的矢量
        """
        return Vector(loc[0], loc[1])

    @staticmethod
    def angelSub(n):
        """
        角度相减
        :param n:
        :return:
        """
        return INFO.PI - abs(INFO.PI - abs(n))