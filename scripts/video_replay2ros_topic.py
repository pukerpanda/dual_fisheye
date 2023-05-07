#! /usr/bin/python3

import os
import argparse

import rospy

from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge, CvBridgeError
import cv2

ROOT = os.path.dirname(os.path.abspath(__file__))+'/'

FRAME_ID = "head_camera"  # Name of the camera frame.

TOPIC = 'camera/image_raw/compressed'


def parse_command_line_arguments():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Publish color images from a video file to ROS topic.")
    parser.add_argument("-i", "--input", required=False, type=str,                        
                        help="An input video of images for publishing.")
    parser.add_argument("-t", "--topic_name", required=False, type=str,
                        default=TOPIC,
                        help="ROS topic name to publish image.")
    parser.add_argument("-r", "--publish_rate", required=False, type=float,
                        default=0.0,
                        help="How many images to publish per second.")

    args = parser.parse_args(rospy.myargv()[1:])                    
    return args

def main(args):
    '''Creates a bag file with a video file'''
    videopath = args.input    
    img_publisher = rospy.Publisher(args.topic_name, CompressedImage, queue_size=10)

    cap = cv2.VideoCapture(videopath)
    cb = CvBridge()
    prop_fps = cap.get(cv2.CAP_PROP_FPS)
    if prop_fps != prop_fps or prop_fps <= 1e-2:
        print("Warning: can't get FPS. Assuming 24.")
        prop_fps = 24

    if args.publish_rate != 0.0:
        prop_fps = args.publish_rate
    
    loop_rate = rospy.Rate(prop_fps)    

    frame_id = 0
    while not rospy.is_shutdown():

        ret, frame = cap.read()
        if not ret:
            rospy.loginfo("EOS")
            break

        frame_id += 1

        try:
            image = cb.cv2_to_compressed_imgmsg(frame)
        except CvBridgeError as e:
            rospy.logerr(e)
            continue

        image.header.stamp = rospy.Time.now()
        image.header.frame_id = FRAME_ID
        img_publisher.publish(image)

        loop_rate.sleep()

    cap.release()
    


if __name__ == '__main__':
    node_name = "publish_images"
    rospy.init_node(node_name)

    args = parse_command_line_arguments()   
#    args.input = ROOT + '../data/PIC_20230504_080209.mp4'                            
    try:
        main(args)
    except rospy.ROSInterruptException: pass

    rospy.logwarn("Node `{}` stops.".format(node_name))

