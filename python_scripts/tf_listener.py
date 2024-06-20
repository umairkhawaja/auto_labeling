#!/usr/bin python2.7

import rospy
import tf
from tf.transformations import euler_from_quaternion

if __name__ == '__main__':
    rospy.init_node('nclt_tf_listener')
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('os_sensor', 'base_link', rospy.Time(0)) # This will give you the coordinate of the child in the parent frame
            rpy = euler_from_quaternion(rot)
            print("os_sensor -> base_link: ", trans, rpy)

            (trans,rot) = listener.lookupTransform('base_link', 'os_sensor', rospy.Time(0)) # This will give you the coordinate of the child in the parent frame
            rpy = euler_from_quaternion(rot)
            print("base_link -> os_sensor: ",trans, rpy)

            (trans,rot) = listener.lookupTransform('os_sensor', 'imu_link', rospy.Time(0)) # This will give you the coordinate of the child in the parent frame
            rpy = euler_from_quaternion(rot)
            print("os_sensor -> imu_link: ", trans, rpy)

            (trans,rot) = listener.lookupTransform('imu_link', 'os_sensor', rospy.Time(0)) # This will give you the coordinate of the child in the parent frame
            rpy = euler_from_quaternion(rot)
            print("imu_link -> os_sensor: ",trans, rpy)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()