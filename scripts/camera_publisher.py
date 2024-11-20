#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os

def load_camera_info(yaml_file):
    with open(yaml_file, "r") as file:
        calib_data = yaml.safe_load(file)
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "camera_frame"
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    return camera_info_msg

def publish_camera_image():
    rospy.init_node('camera_publisher', anonymous=True)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        return

    cam_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'cam.yaml')
    camera_info_msg = load_camera_info(cam_file)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            break

        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        camera_info_msg.header.stamp = rospy.Time.now()

        image_pub.publish(image_msg)
        camera_info_pub.publish(camera_info_msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        publish_camera_image()
    except rospy.ROSInterruptException:
        pass
