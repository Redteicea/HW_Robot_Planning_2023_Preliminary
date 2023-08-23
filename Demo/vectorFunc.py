import math
from enum import Enum
from functools import singledispatch


class Vector(object):
    def __init__(self, x_, y_):
        self.x = x_
        self.y = y_

    def __add__(self, other):   # 相加
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):   # 相减
        return Vector(self.x - other.x, self.y - other.y)

    def __neg__(self):  # 取负
        return Vector(-self.x, -self.y)

    def dot(self, other):   # 点积
        return self.x * other.x + self.y * other.y

    def __mul__(self, other):   # 乘上某个数
        return Vector(self.x * other, self.y * other)

    def fork_div(self, other):    # 叉除
        return Vector(self.x / other.x, self.y / other.y)

    def __truediv__(self, other:float):
        return Vector(self.x / other, self.y / other)

    def __eq__(self, other):    # 判断是否相等
        return self.x == other.x and self.y == other.y

    def mod(self):  # 取模
        return math.sqrt(self.x * self.x + self.y * self.y)

    def det(self, other):
        return self.x * other.y - self.y * other.x

    def powMod(self):   # 取模的平方
        return self.x * self.x + self.y * self.y

    def normal(self):   # 归一化
        if self.x == 0 or self.y ==0:
            return Vector(0, 0)
        rs = 1.0  / math.sqrt(self.x ** 2 + self.y ** 2)
        return Vector(self.x / rs, self.y / rs)

    def angle(self):    # 转化为角度
        return math.atan2(self.y, self.x)

    def rotate(self, angle):    # 旋转角度
        s = math.sin(angle)
        c = math.cos(angle)
        return Vector(self.x*c-self.y*s, self.x*s+self.y*c)

    def copyFrom(self, other):
        self.x = other.x
        self.y = other.y