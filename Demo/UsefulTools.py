#-*-coding: utf-8-*-
from typing import List

from pylab import *
import os
import copy
import logging
import datetime
mpl.rcParams['font.sans-serif'] = ['SimHei']
mpl.rcParams['axes.unicode_minus'] = False


# 日志记录类，初始化日志记录模组
class LOG(object):
    def __init__(self, logHandle='LogRecord'):
        # 获取logger对象,LogRecord
        self.logger = logging.getLogger(logHandle)
        # 输出DEBUG及以上级别的信息，针对所有输出的第一层过滤
        self.logger.setLevel(level=logging.DEBUG)

    def LogInitialize(self):
        """日志头初始化"""
        # 获取文件日志句柄并设置日志级别，第二层过滤
        curr_time = datetime.datetime.now()     # 获取当前时间
        print(curr_time.year, curr_time.month, curr_time.day)
        year = int(curr_time.year)
        month = int(curr_time.month)
        day = int(curr_time.day)
        log_name = "./log/%s%s%s.log"% (year, month, day)

        # 检查目标路径，文件夹不存在则创建文件夹
        bPathExist = os.path.exists('./log')
        if not bPathExist:
            os.makedirs('./log')

        handler = logging.FileHandler(log_name)
        handler.setLevel(logging.INFO)

        # 生成并设置文件日志格式，其中name为上面设置的mylog
        formatter = logging.Formatter('%(asctime)s - %(message)s')
        handler.setFormatter(formatter)

        # 获取流句柄并设置日志级别，第二层过滤
        console = logging.StreamHandler()
        console.setLevel(logging.WARNING)

        # 为logger对象添加句柄
        self.logger.addHandler(handler)
        self.logger.addHandler(console)
        return None

    def LogRecord(self, str_record, bPrint = False):
        """记录日志"""
        # 是否输出到控制台
        if bPrint:
            print(str_record)

        # 记录日志
        self.logger.info(str_record)

