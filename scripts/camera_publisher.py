#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

def get_camera_info():
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "camera_frame"
    camera_info_msg.width = 1280
    camera_info_msg.height = 720
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

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            break

        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        camera_info_msg = get_camera_info()
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
