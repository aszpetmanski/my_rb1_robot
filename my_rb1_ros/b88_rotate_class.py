#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as t
from math import radians, pi

class RotateBB8():
    
    def __init__(self):
        self.bb8_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bb8_odom_subscriber = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(10)     # 10 Hz for smoother motion
        self.yaw = 0.0                 # current yaw from odom
        self._odom_ready = False       # flag indicating we got at least one odom
        rospy.on_shutdown(self.shutdownhook)
        
    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.bb8_vel_publisher.get_num_connections()
            if connections > 0:
                self.bb8_vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop_bb8()
        self.ctrl_c = True
        
    def stop_bb8(self):
        rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()
    
    def odom_callback(self, msg):
        """
        Store the current yaw (rotation around Z) from odom.
        """
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y,
                            orientation_q.z,
                            orientation_q.w]
        (roll, pitch, yaw) = t.euler_from_quaternion(orientation_list)
        self.yaw = yaw
        self._odom_ready = True

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to be between -pi and +pi.
        """
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle




    def rotate_bb8(self, degrees):
        """
        Rotate robot by the requested degrees (positive = CCW, negative = CW).
        Uses odometry yaw to stop at the desired angle.
        """
        # Wait until we have odom
        rospy.loginfo("Waiting for odom...")
        while not self._odom_ready and not rospy.is_shutdown():
            self.rate.sleep()
        rospy.loginfo("Odom received, starting rotation")

        # Convert degrees to radians
        target_angle = radians(abs(degrees))

        # Choose direction: +1 for CCW, -1 for CW
        direction = 1.0 if degrees >= 0.0 else -1.0

        # Rotation speed (rad/s) – you can tune this
        angular_speed = 0.3

        # Initial yaw
        start_yaw = self.yaw

        # Set command velocities
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = direction * angular_speed

        rospy.loginfo("Rotating BB8 by {} degrees".format(degrees))

        # Rotate until we've reached the desired angle
        while not self.ctrl_c and not rospy.is_shutdown():
            # Current angle turned (handle wrap-around)
            current_yaw = self.yaw
            delta_yaw = self.normalize_angle(current_yaw - start_yaw)

            if abs(delta_yaw) >= target_angle:
                rospy.loginfo("Reached target angle: {:.2f} rad (~{:.2f} deg)".format(
                    delta_yaw, delta_yaw * 180.0 / pi))
                break

            # Publish commanded rotation
            self.bb8_vel_publisher.publish(self.cmd)
            self.rate.sleep()

        # Stop robot at the end
        self.stop_bb8()
        rospy.loginfo("Rotation finished")


if __name__ == '__main__':
    rospy.init_node('rotate_bb8_test', anonymous=True)
    rotatebb8_object = RotateBB8()
    try:
        # Example: rotate 90 degrees CCW if you run this file directly
        rotatebb8_object.rotate_bb8(90.0)
    except rospy.ROSInterruptException:
        pass