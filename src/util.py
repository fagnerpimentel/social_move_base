#!/usr/bin/env python3
import rospy
import numpy
import actionlib

from std_msgs.msg import Header
from visualization_msgs.msg import Marker

def euler_to_quaternion(roll, pitch, yaw):
  qx = numpy.sin(roll/2) * numpy.cos(pitch/2) * numpy.cos(yaw/2) - numpy.cos(roll/2) * numpy.sin(pitch/2) * numpy.sin(yaw/2)
  qy = numpy.cos(roll/2) * numpy.sin(pitch/2) * numpy.cos(yaw/2) + numpy.sin(roll/2) * numpy.cos(pitch/2) * numpy.sin(yaw/2)
  qz = numpy.cos(roll/2) * numpy.cos(pitch/2) * numpy.sin(yaw/2) - numpy.sin(roll/2) * numpy.sin(pitch/2) * numpy.cos(yaw/2)
  qw = numpy.cos(roll/2) * numpy.cos(pitch/2) * numpy.cos(yaw/2) + numpy.sin(roll/2) * numpy.sin(pitch/2) * numpy.sin(yaw/2)
  return [qx, qy, qz, qw]

def quaternion_to_euler(x, y, z, w):
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll = numpy.arctan2(t0, t1)
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch = numpy.arcsin(t2)
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw = numpy.arctan2(t3, t4)
  return [yaw, pitch, roll]

def init_publisher(_topic, _msg_type, _queue_size=10):
    rospy.loginfo('Starting publisher: {}'.format(_topic))
    return rospy.Publisher(_topic, _msg_type, queue_size=_queue_size)
def init_subscriber(_topic, _msg_type, _callback):
    rospy.loginfo('Starting subscriber: {}'.format(_topic))
    return rospy.Subscriber(_topic, _msg_type, _callback)
def init_service_server(_topic, _msg_type, _callback):
    rospy.loginfo('Starting service server: {}'.format(_topic))
    return rospy.Service(_topic, _msg_type, _callback)
def init_service_client(_topic, _msg_type):
    rospy.loginfo('Starting service client: {}'.format(_topic))
    rospy.wait_for_service(_topic)
    return rospy.ServiceProxy(_topic, _msg_type)
def init_action_server(_topic, _msg_type, _callback):
    rospy.loginfo('Starting action server: {}'.format(_topic))
    action = actionlib.SimpleActionServer(_topic, _msg_type, _callback)
    action.start()
    return action
def init_action_client(_topic, _msg_type):
    rospy.loginfo('Starting action client: {}'.format(_topic))
    action = actionlib.SimpleActionClient(_topic, _msg_type)
    action.wait_for_server()
    return action
