#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 裁判系统入口（对应 ROS1 的 judge_system_node.py）。
# ROS1 原版 `from judge_system import judger` 依赖 Cython .so，此处改为导入纯 Python 重写的 Judger。

import judge_system


if __name__ == '__main__':
    judge_system.main()
