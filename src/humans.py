# coding=utf-8
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, AccelWithCovarianceStamped


class Human():
    def __init__(self):
        self.current_pose = None   # type: PoseWithCovarianceStamped
        self.last_pose = None      # type: PoseWithCovarianceStamped
        self.current_speed = None  # type: TwistWithCovarianceStamped
        self.last_speed = None     # type: TwistWithCovarianceStamped
        self.acceleration = None    # type: AccelWithCovarianceStamped