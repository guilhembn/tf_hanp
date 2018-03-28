#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, Quaternion, AccelWithCovarianceStamped
from hanp_msgs.msg import TrackedHumans, TrackedHuman, TrackedSegment, TrackedSegmentType
from visualization_msgs.msg import MarkerArray, Marker
import signal
import tf.listener, tf.transformations
import re
import humans

HUMAN_PUB_TOPIC_NAME = "/tracked_humans"
HUMAN_MARKER_PUB_TOPIC_NAME = "/tracked_humans_markers"
REGEXP_HUMAN_FRAME = "(mocap_)?human-([0-9]+)"

MAP_FRAME = "map"

pub_humans = None
pub_humans_markers = None
tf_sub = None  # type: tf.TransformListener

humans_dict = {}  # type: dict[int, humans.Human]

human_frame_re = re.compile(REGEXP_HUMAN_FRAME)


def fillSpeed(human):
    dt = (human.current_pose.header.stamp - human.last_pose.header.stamp).to_sec()
    diffSpeed = TwistWithCovarianceStamped()
    diffSpeed.header = human.current_pose.header
    diffSpeed.twist.twist.linear.x = (human.current_pose.pose.pose.position.x - human.last_pose.pose.pose.position.x) / dt
    diffSpeed.twist.twist.linear.y = (human.current_pose.pose.pose.position.y - human.last_pose.pose.pose.position.y) / dt
    diffSpeed.twist.twist.linear.z = (human.current_pose.pose.pose.position.z - human.last_pose.pose.pose.position.z) / dt
    last_rotation = tf.transformations.inverse_matrix(tf.transformations.quaternion_matrix([human.last_pose.pose.pose.orientation.x,
                                                                                human.last_pose.pose.pose.orientation.y,
                                                                                human.last_pose.pose.pose.orientation.z,
                                                                                human.last_pose.pose.pose.orientation.w]))
    current_rotation = tf.transformations.quaternion_matrix([human.current_pose.pose.pose.orientation.x,
                                              human.current_pose.pose.pose.orientation.y,
                                              human.current_pose.pose.pose.orientation.z,
                                              human.current_pose.pose.pose.orientation.w])
    rot_diff = current_rotation * last_rotation

    roll, pitch, yaw = tf.transformations.euler_from_matrix(rot_diff)
    diffSpeed.twist.twist.angular.x = roll / dt
    diffSpeed.twist.twist.angular.y = pitch / dt
    diffSpeed.twist.twist.angular.z = yaw / dt
    human.last_speed = human.current_speed
    human.current_speed = diffSpeed

def fillAcceleration(human):
    dt = (human.current_speed.header.stamp - human.last_speed.header.stamp).to_sec()
    human.acceleration = AccelWithCovarianceStamped()
    human.acceleration.accel.accel.linear.x = (human.current_speed.twist.twist.linear.x - human.last_speed.twist.twist.linear.x) / dt
    human.acceleration.accel.accel.linear.y = (human.current_speed.twist.twist.linear.y - human.last_speed.twist.twist.linear.y) / dt
    human.acceleration.accel.accel.linear.z = (human.current_speed.twist.twist.linear.z - human.last_speed.twist.twist.linear.z) / dt
    human.acceleration.accel.accel.angular.x = (human.current_speed.twist.twist.angular.x - human.last_speed.twist.twist.angular.x) / dt
    human.acceleration.accel.accel.angular.y = (human.current_speed.twist.twist.angular.y - human.last_speed.twist.twist.angular.y) / dt
    human.acceleration.accel.accel.angular.z = (human.current_speed.twist.twist.angular.z - human.last_speed.twist.twist.angular.z) / dt
    human.acceleration.header = human.current_speed.header

def parse_tf(time):
    global humans_dict, tf_sub
    for frame in tf_sub.getFrameStrings():
        match = human_frame_re.match(frame)
        if match is not None:
            id = int(match.group(2))
            is_mocap = match.group(1) is not None
            try:
                position, orientation = tf_sub.lookupTransform(MAP_FRAME, frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            if id not in humans_dict:
                humans_dict[id] = humans.Human()
            if not is_mocap:
                orientation = tf.transformations.quaternion_multiply(orientation, [0, 0.707, 0, 0.707])
            humans_dict[id].last_pose = humans_dict[id].current_pose
            h_pose = PoseWithCovarianceStamped()
            h_pose.header.frame_id = MAP_FRAME
            h_pose.header.stamp = rospy.Time.now()
            h_pose.pose.pose.position.x, h_pose.pose.pose.position.y, h_pose.pose.pose.position.z = position
            h_pose.pose.pose.orientation.x, h_pose.pose.pose.orientation.y, h_pose.pose.pose.orientation.z, h_pose.pose.pose.orientation.w = orientation
            humans_dict[id].current_pose = h_pose
            if humans_dict[id].last_pose is not None:
                fillSpeed(humans_dict[id])
            if humans_dict[id].last_speed is not None:
                fillAcceleration(humans_dict[id])
    if len(humans_dict) != 0:
        send_hanp()

def fill_hanp_markers(id, human):
    human_arrow = Marker()
    human_cylinder = Marker()

    human_arrow.header.stamp = rospy.Time.now()
    human_arrow.header.frame_id = human.current_pose.header.frame_id
    human_arrow.type = Marker.ARROW
    human_arrow.action = Marker.MODIFY
    human_arrow.id = 10000 + id
    human_arrow.pose.position.x = human.current_pose.pose.pose.position.x
    human_arrow.pose.position.y = human.current_pose.pose.pose.position.y
    _, _, yaw = tf.transformations.euler_from_quaternion([human.current_pose.pose.pose.orientation.x, human.current_pose.pose.pose.orientation.y,
                                                         human.current_pose.pose.pose.orientation.z, human.current_pose.pose.pose.orientation.w])
    human_arrow.pose.orientation = Quaternion()
    x,y,z,w = tf.transformations.quaternion_from_euler(0, 0, yaw)
    human_arrow.pose.orientation.x, human_arrow.pose.orientation.y, human_arrow.pose.orientation.z, human_arrow.pose.orientation.w = x,y,z,w
    human_arrow.scale.x = 0.5
    human_arrow.scale.y = 0.1
    human_arrow.scale.z = 0.1
    human_arrow.color.a = 1.0
    human_arrow.color.r = 0
    human_arrow.color.g = 150
    human_arrow.color.b = 200
    human_arrow.lifetime = rospy.Duration(4)

    human_cylinder.header.stamp = rospy.Time.now()
    human_cylinder.header.frame_id = human.current_pose.header.frame_id
    human_cylinder.type = Marker.CYLINDER
    human_cylinder.action = Marker.MODIFY
    human_cylinder.id = id
    human_cylinder.pose.position.x = human.current_pose.pose.pose.position.x
    human_cylinder.pose.position.y = human.current_pose.pose.pose.position.y
    human_cylinder.pose.position.z += (1.5 / 2)
    human_cylinder.scale.x = 0.25 * 2
    human_cylinder.scale.y = 0.25 * 2
    human_cylinder.scale.z = 1.5
    human_cylinder.color.a = 1.0
    human_cylinder.color.r = 0
    human_cylinder.color.g = 150
    human_cylinder.color.b = 200
    human_cylinder.lifetime = rospy.Duration(4)

    return human_arrow, human_cylinder

def send_hanp():
    global humans_dict, pub_humans, pub_humans_markers
    msg = TrackedHumans()
    visi_msg = MarkerArray()
    for id, human in humans_dict.items():
        if human.acceleration is None:
            continue
        h = TrackedHuman()
        segment = TrackedSegment()
        segment.pose = human.current_pose.pose
        segment.twist = human.current_speed.twist
        segment.accel = human.acceleration.accel
        segment.type = TrackedSegmentType.TORSO
        h.segments.append(segment)
        msg.humans.append(h)

        marker_arrow, marker_cylinder = fill_hanp_markers(id, human)
        visi_msg.markers.append(marker_arrow)
        visi_msg.markers.append(marker_cylinder)
    pub_humans.publish(msg)
    pub_humans_markers.publish(visi_msg)


if __name__ == '__main__':
    rospy.init_node("tf2hanp")
    pub_humans = rospy.Publisher(HUMAN_PUB_TOPIC_NAME, TrackedHumans, queue_size=10)
    pub_humans_markers = rospy.Publisher(HUMAN_MARKER_PUB_TOPIC_NAME, MarkerArray, queue_size=10)
    tf_sub = tf.listener.TransformListener()
    rospy.Timer(rospy.Duration(0.2), parse_tf)
    rospy.spin()
