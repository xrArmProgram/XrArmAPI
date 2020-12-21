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