#! /usr/bin/env python

import rospy
from my_rb1_ros.srv import Rotate, RotateResponse
from b88_rotate_class import RotateBB8

def my_callback(request):
    rospy.loginfo("Service Requested")
    rotatebb8_object = RotateBB8()
    rotatebb8_object.rotate_bb8(request.degrees)
    rospy.loginfo("Service Completed")
    response = RotateResponse()
    response.result = 'success'
    return response

rospy.init_node('service_rotate_bb8_server', log_level=rospy.INFO) 
my_service = rospy.Service('/rotate_robot', Rotate , my_callback)
rospy.loginfo("Service Ready")
rospy.spin()
