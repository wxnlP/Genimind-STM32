## 工程介绍

我的第一个机器人，名为Genimind，基于ROS2、STM32等开发，本仓库为STM32底盘的代码。

## 外设使用

本工程支持使用`STM32CubeMX IDE`和`Clion`打开，推荐使用`Clion`，[Clion开发环境搭建链接](https://tonmoon.top/study/STM32/0.阅读声明/)。

![image-20250521102251583](https://tonmoon.obs.cn-east-3.myhuaweicloud.com/img/tonmoon/image-20250521102251583.png)

## 目录介绍

```
BSP --------- 基础外设驱动（注意`can_fd`实际为高速CAN，而非CAN_FD协议）
Robotics ---- 机器人中间件
```

## Clion最新版适配

对 2025 版的 Clion 进行适配，最新版的 Clion 配合 STM 官方的 STM32CubeCLT 工具可以更方便的完成项目构建工作。但是，最新版目前发现仅支持 Cmake 工具链，因此对项目进行了简单移植，可以在最新版 Clion 编译烧录。

关于最新版的 Clion 的嵌入式开发环境搭配，推荐：[爽！手把手教你用CLion开发STM32【大人，时代变啦！！！】_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1pnjizYEAk/?spm_id_from=333.1387.homepage.video_card.click&vd_source=9360af603fa540663a17ba65dbad6a7d)