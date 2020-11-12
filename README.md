# README

## XrArmAPI功能说明

- SimpleRobot(robot_name)<br>
创建一个机器人对象<br>
参数说明：
    - robot_name: 创建的机器人节点的节点名称，在系统中唯一。

- init() -> none

    将机械臂恢复到初始状态
- set_pose(pose) -> none

    设置末端执行器的空间坐标
    参数说明:
    - pose: [x, y, z]，空间坐标数组。指定末端执行器的位置。
- update(angle) -> none
    修改机械臂各个关节的角度。
    参数说明：
    - angle： 5个关节的角度值的数组（从0-4，分别对应机械臂从下到上5个关节），弧度制。数组长度必须大于等于5。
    前5个数据有效，第5个之后的数据舍弃。

- loop() -> none

    阻塞循环等待后台事件处理，直到用户终止程序（执行loop_stop或者终止整个进程）。
- loop_start() -> none

    新建一个线程在后台处理后台事件，不会在当前程序阻塞。调用loop_stop时结束该进程。
- loop_stop() -> none

    终止loop*函数。
- read() -> tuple

    读取当前机械臂各个关节的角度。返回值为5个数据的数组。从0-4分别代表机械臂从下到上5个关节的角度。
- speak(audio_file, block)
    播放音频。
    参数说明：
    - audio_file: 音频文件的路径
    - block: 如果为False， 非阻塞执行
    
---------
 
 ## 文件结构说明
 - BASE 基类定义
     - BaseRobot.py
     
        - class AbstractRobot：
        
            定义机械臂的操作接口（抽象类）。之后需要实现机械臂接口继承该类即可。
        - class SimpleRobot(AbstractRobot):
        
            实现类一个用于调试的简单机械臂接口类。不能操纵机械臂。只用于接口调试。
    
     - BaseSpeaker.py
        - class AbstractSpeaker
        
            抽象音频播放接口。
        - class PlaySoundSpeaker(AbstractSpeaker)
        
            基于playsound的音频播放接口
- 实现的API接口
    - PlaySoundSpeaker
    - ArmRobot
 - test.py
 
    基于SimpleRobot的简单测试示例程序。