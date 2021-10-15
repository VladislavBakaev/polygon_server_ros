#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import numpy as np
import cv2

import motorcortex
import mcx_tracking_cam_pb2 as tracking_cam_msg

import sys
import os

from cv_bridge import CvBridge
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
cvBridge = CvBridge()
req = 0


def onBlob(val):
	try:
		blobs = tracking_cam_msg.Blobs()
		if blobs.ParseFromString(val[0].value):
			#print(blobs.value[0].cx, blobs.value[0].cy)
			#print(type(blobs.value[0].cx), type(blobs.value[0].cy))
			for bl in blobs.value:
				blob.x = float(bl.cx)
				blob.y = float(bl.cy)
				blob.z = bl.type
				pub_blob.publish(blob)
	except Exception as e:
	    print(e)


def imageParsing(data):
	cv_image_original = cvBridge.imgmsg_to_cv2(data, "bgr8")
	retval, buffer = cv2.imencode('.jpg', cv_image_original)
	try:
        	handle = req.setParameter("root/Processing/image", buffer.tobytes())
        	res = handle.get()
        	print("res: {}".format(req))
	except Exception:
		print("Unexpected error imageParsing:", sys.exc_info()[0])


if __name__ == '__main__':

	# Creating empty object for parameter tree
	parameter_tree = motorcortex.ParameterTree()

	# Loading protobuf types and hashes
	motorcortex_types = motorcortex.MessageTypes()


	# Open request connection
	rospack = rospkg.RosPack()
	pkg_path = rospack.get_path('ros_udp_bridge_pkg')
	req_, sub_ = motorcortex.connect("ws://192.168.42.1:5558:5557", motorcortex.MessageTypes(), motorcortex.ParameterTree(),
	                               certificate=pkg_path+"/config/motorcortex.crt", timeout_ms=1000,
	                               login="root", password="vectioneer")
	req = req_

	rospy.init_node('motorcortex_proxy')
	pub_blob = rospy.Publisher('blob', Vector3)
	blob = Vector3()
	subscription2 = sub_.subscribe(["root/Processing/BlobDetectorNew/blobBuffer"], "blob", 1)
	subscription2.get()
	subscription2.notify(onBlob)
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			break
