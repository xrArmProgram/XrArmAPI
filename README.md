# README

## XrArmAPI概述

​			这是一个基于XR的AI机械臂的二次开发框架。框架集成了rospy接口、moveit控制接口，用户无需了解ROS和moveit技术即可完成对XR的AI机械臂的控制。仅需要一点点的Python基础。该框架使用抽象工厂模式设计，集成了机器人控制组件、音频播放组件、图像显示组件、语音模块解析组件、用户自定义插件、流程控制器组件。框架通过builder将这些组件组装到一起，并提供阻塞和非阻塞两种运行方式。用户可根据需求自行选择运行模式。所有配置信息都存放在setting中。如果需要开发自定义功能插件，请查看[二次开说明文档](https://github.com/xrArmProgram/XrArmAPI/blob/main/doc/Development_guidance.md)，也可以参考[预设的二次开发案例](https://github.com/xrArmProgram/XrArmAPI/blob/main/doc/simple.md)。

-----

## 使用说明

​	整个框架的程序入口是`main.py`。执行`python main.py`即可运行程序。

​	Python版本：python 2.7

​	依赖的Python包：

|   包名    |  版本  |
| :-------: | :----: |
|  pyaudio  | 0.2.11 |
| playsound | 1.2.2  |



---------

 ## 开发说明
 - [API接口定义](https://github.com/xrArmProgram/XrArmAPI/blob/main/doc/base_class.md)

    ​		定义了接口的抽象定义。实现接口类需要继承自对应功能的抽象类，该类不能实例化。继承了抽象接口的类必须实现抽象接口定义的所有抽象方法。如`class ArmRobot(AbstractRobot, BaseSingleton4py2)`.表示该接口为robot类型的接口的实现，且是一个使用单例模式设计的类。并且该robot类必须实现AbstractRobot所定义的被@abstractmethod修饰的方法。所有的抽象接口都定义在`API/BASE`中。

- [实现的API接口类](https://github.com/xrArmProgram/XrArmAPI/blob/main/doc/API.md)

    ​		定义了接口的具体实现，这是软件运行过程中实际调用的接口。所有系统接口都定义在`API`中

 - [二次开发实例](https://github.com/xrArmProgram/XrArmAPI/blob/main/doc/simple.md)

    ​		这是框架预设的二次开发案例，有颜色识别、形状识别、人脸跟随、物体分拣和ROS通信示例。这些模块默认已经插入到框架中，在运行`python main.py`后，可以通过语音功能呼出相应的功能。

 - [二次开发说明](https://github.com/xrArmProgram/XrArmAPI/blob/main/doc/Development_guidance.md)

    ​		这里是关于自定义二次开发的一些指导和说明。如果需要开发自定一功能，务必查看该文档。

- [配置信息](https://github.com/xrArmProgram/XrArmAPI/blob/main/doc/setting.md)

  ​		这里存储了框架中所有的配置信息，包括语音模块通信协议配置、功能模块的插入和映射等。

- [音频配置](https://github.com/xrArmProgram/XrArmAPI/blob/main/doc/audio_config.md)

  ​		这里存放所有的音频文件，音频文件的配置也在这里。

- [环境配置](https://github.com/xrArmProgram/XrArmAPI/blob/main/doc/env_installer.md)

  ​		这里有放有框架所需python包信息，运行安装脚本即可完成安装。

