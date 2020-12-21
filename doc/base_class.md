

## class AbstractRobot：

定义机械臂的操作接口（抽象类）。之后需要实现机械臂接口继承该类即可。

**接口说明**

- **~init() -> None:**
      """Return the manipulator to its original position."""

使机械臂回复到初始状态。

- **~update(angle: list) -> None:**
      """ Modifying the pose of the manipulator.
          angle:An array of angle parameters for each joint."""

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

- **~speak(audio_file, block=: bool True) -> None:**

  播放语音，`audio_file`音频文件的路径，建议使用【xrarm_audio】中保存定义的音频，或者在里面增加音频。`block`表示是否阻塞播放。默认为True，阻塞播放。

- **~set_pose(pose: list) -> None:**

  设置机械臂末端执行器的空间坐标`[x, y, z]`单位是m。该方法需要和rviz通信，相应较慢（5s）。

## class AbstractSpeaker

抽象音频播放接口。

