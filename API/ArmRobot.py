import sys
from collections import Iterable
from threading import Lock, Thread
from numpy import array as np_array
import rospy
from moveit_commander import roscpp_initialize, PlanningSceneInterface
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from API.BASE import AbstractRobot, BaseSingleton4py2
from API.BASE import OperationRepetitionError
from API.SoundSpeaker import SoundSpeaker
from CAPI import MoveMK2ik

lock = Lock()


class ArmRobot(AbstractRobot, BaseSingleton4py2):
    """XR M1 arm robot controller"""

    def __init__(self, node_name="xr_arm_controller", local_rospy=rospy, speaker=None, anonymous=True, img_player=None):
        # got rospy module
        self.__rospy = local_rospy

        # set speaker
        self.speaker = speaker
        if speaker is None:
            self.speaker = SoundSpeaker()
        else:
            self.speaker = speaker

        self.__img_player = img_player

        # config ros node
        self.__node_name = node_name
        self.__anonymous = anonymous
        self.__using_external_ros_handle = True

        # set running flag
        self.__is_run = False

        # config ros publisher and msg
        self.__angle = None
        self.__angle_publisher = None
        self.__join_state = JointState()
        self.__join_state.name = ['a', 'b', 'c', 'd', 'e']
        self.__join_state.position = [0.0, 0.0, 0.0, 0.0, 0.0]

        # config for moveit
        self.__XrIK = None
        self.__scene = None
        self.__pose = Pose()
        self.__quat = np_array([-3.32607856193e-10,
                                -4.39052018621e-05,
                                -7.57575235874e-06,
                                0.999999999007])

        # set loop thread
        self.__loop_thread = Thread(target=self.loop)

    def res_init(self):
        self.__angle_publisher = self.__rospy.Publisher("/move_group/fake_controller_joint_states",
                                                        JointState, queue_size=2)

        # init moveit api
        self.__scene = PlanningSceneInterface()
        roscpp_initialize(sys.argv)

        # If the ros node is not initialized, initialize the node
        if self.__rospy.get_name() == "/unnamed":
            self.__rospy.init_node(name=self.__node_name,
                                   anonymous=self.__anonymous,
                                   disable_signals=True)

            self.__using_external_ros_handle = False

        # initialize the arm planner
        self.__XrIK = MoveMK2ik()

    def loop(self):
        if self.__is_run:
            raise OperationRepetitionError

        if self.__using_external_ros_handle:
            return None

        # handle ros event
        self.__is_run = True
        self.__rospy.spin()

    def loop_start(self):
        if self.__using_external_ros_handle:
            return None

        self.__loop_thread.start()

    def loop_stop(self):
        if self.__using_external_ros_handle:
            print("__using_external_ros_handle")
            return None

        self.__rospy.signal_shutdown("user stopped controller")
        # self.__loop_thread.join()
        print("ros end")

    def set_pose(self, pose):
        if not (pose, Iterable):
            print("{} is required, but the input is {}".format(type(Iterable), type(pose), ))
            raise TypeError(pose)

        self.__pose.position.x = pose[0]
        self.__pose.position.y = pose[1]
        self.__pose.position.z = pose[2]

        self.__XrIK.create_move_group_pose_goal(self.__pose.position,
                                                self.__quat,
                                                group="arm",
                                                end_link_name="end_Link",
                                                plan_only=False)

    def update(self, angle):
        if not (angle, Iterable):
            print("{} is required, but the input is {}".format(type(Iterable), type(angle), ))
            raise TypeError(angle)

        self.__angle = angle
        self.__join_state.position[0:4] = angle[0:4]
        if len(angle) > 4:
            self.__join_state.position[4] = angle[4]

        self.__angle_publisher.publish(self.__join_state)

    def read(self):
        return self.__angle

    def init(self):
        self.__angle = [0.0 for i in xrange(5)]
        self.update(self.__angle)

    def speak(self, audio_file, block=True):
        self.speaker.speak(audio_file, block)

    def show(self, window_name, img):
        self.__img_player.show(window_name, img)
