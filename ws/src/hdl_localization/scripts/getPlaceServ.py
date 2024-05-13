#!/usr/bin/env python

import rospy
from hdl_localization.srv import getPlace,getPlaceRequest,getPlaceResponse

def handle_action(req):
    rospy.loginfo(f"Received request: {req.action}")
    if req.action=="get":
        
        result = True
        rospy.loginfo(f"Sending response: {result}")
    return getPlaceResponse(result)

def get_place_server():
    rospy.init_node('get_place_server')
    rospy.Service('get_place_server', getPlace, handle_action)
    rospy.spin()

if __name__ == "__main__":
    get_place_server()
