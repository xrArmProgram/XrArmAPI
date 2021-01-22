

## class BaseSingleton4py2:

使用了单例模式的基类（Singleton for py2.7)，继承了该类的子类都会工作在单例模式下。

**单例模式：**程序运行过程中只会存在一个实例。不论进行多少次实例化（即使在多线程中，该类为线程安全设计），得到的都是同一个实例对象。该设计模式主要用于需要资源共享时或实例化操作费时灯情况。

该类重写了`__new__`方法。如果继承自该类的子类需要重写`__new__`方法，需要用`super()`调用`BaseSingleton4py2`的`__new__`方法。

------

## class OperationRepetitionError

在程序运行过程中，如果重复执行一些只能执行一次的方法会抛出这个异常。

------

## class AbstractRobot:

定义机械臂的操作接口（抽象类）。之后需要实现机械臂接口继承该类即可。

**接口说明：**

- **~init() -> None:**
      """Return the manipulator to its original position."""
  
  使机械臂回复到初始状态。
  
- **~update(angle: list) -> None:**
      """ Modifying the pose of the manipulator.
          angle: An array of angle parameters for each joint."""

  设置机械臂各个舵机的角度值。

- **~read() -> list:**
      """Read the angle array of each joint of the manipulator."""

  读取机械臂各个舵机的角度值。

- **~loop() -> None:**
      """Loop handle manipulator feedback"""

  阻塞循环handle网络事件，直到执行**~loop_stop()。

- **~loop_start() -> None:**
      """Loop the handle manipulator feedback in the background"""

  开启一个线程循环handle网络事件，直到执行**~loop_stop()才退出线程。

- **~loop_stop() -> None:**
      """Stop background loop"""

  停止正在运行的*loop()

- **~speak(audio_file: str, block=: bool True) -> None:**

  播放语音，`audio_file`音频文件的路径，建议使用【xrarm_audio】中保存定义的音频，或者在里面增加音频。`block`表示是否阻塞播放。默认为True，阻塞播放。

- **~set_pose(pose: list) -> None:**

  设置机械臂末端执行器的空间坐标`[x, y, z]`单位是m。该方法需要和rviz通信，响应较慢（5s）。

------

## class AbstractSpeaker:

抽象音频播放接口。

**接口说明：**

- **~ speak( audio_file: str, block=: bool True) -> None:**

  播放语音，`audio_file`音频文件的路径，建议使用【xrarm_audio】中保存定义的音频，或者在里面增加音频。`block`表示是否阻塞播放。默认为True，阻塞播放。

------

## class AbstractSoundPlayer:

系统音频播放统一封装抽象接口，将不同的系统音频接口封装成统一的接口，方便框架调用。

- **~playsound(audio_file: str) -> None:**

  将系统音频接口调用过程封装到该方法中，即可完成对系统音频播放接口的封装。

------

## class AbstractImgPlayer:

- **~show(img: array, window_name: str) -> None:**

  图片显示接口。该接口不会立即显示图片，而是将图片添加到待显示队列中。该列表是一个队列的数据结构（先进先出）。

- **~async_play() -> generator:**

  返回一个生成器。调用生成器的next()方法会将待显示队列中的第一个对象出队并显示。需要循环调用生成器的next()方法才可以持续的显示图像。

- **~destroy() -> None:**

  销毁GUI资源，结束生成器。

------

## class AbstractRunner：

二次开发的功能类的基类，二次开发的功能类**必须**继承自该类，否则功能将无法启用。该基类为抽象类，强制用户实现run()方法和stop()方法，且stop()方法必须能够使run()方法立即正常结束并释放相关资源。否则系统将不会正常工作。

- **~run() -> None:**

  功能类的功能入口，当系统选择该功能时会开启一个新线程运行该函数。所以该函数必须是可控的，必须在调用stop()方法之后能够立即正常结束，并释放资源。

- **~stop() -> None:**

  结束功能的入口。调用该方法后run()必须能够立即正常结束。

------

## class AbstractVoice:

抽象语音模块接口，

- **~run() -> None:**

  阻塞循环读取语音模块发送的消息。

- **~stop() -> None:**

  结束语音模块消息读取任务。

- **~init() -> None:**

  初始化语音模块相关资源。

------

## class AbstractController:

流程控制器，控制整个框架的业务流程。是整个框架的业务逻辑的核心。

- **~run() -> None:**

  流程控制入口，阻塞执行。

- **~stop() -> None:**

  结束流程。

- **~run_iterable() -> generator:**

  流程控制入口，非阻塞执行。该方法会返回一个生成器，调用该生成器的next()方法会完成一次流程。循环调用会循环执行流程，直到调用stop()方法。

------

## class AbstractBuilder:

- **~build() -> None:**
- **~run() -> None:**
- **~run_iterable() -> generator:**
- **~destroy() -> None:**