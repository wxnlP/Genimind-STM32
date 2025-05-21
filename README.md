## 工程介绍

我的第一个机器人，名为Genimind，基于ROS2、STM32等开发，本仓库为STM32底盘的代码。

## 外设使用

本工程支持使用`STM32CubeMX IDE`和`Clion`打开，推荐使用`Clion`，[Clion开发环境搭建链接](https://tonmoon.top/study/STM32/0.阅读声明/)。

![image-20250521102251583](https://tonmoon.obs.cn-east-3.myhuaweicloud.com/img/tonmoon/image-20250521102251583.png)

## 目录介绍

```
BSP --------- 基础外设驱动（注意`can_fd`实际为高速CAN，而非CAN_FD协议）
Robotics ---- 机器人中间件
```

