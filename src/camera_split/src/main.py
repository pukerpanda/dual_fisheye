#! /usr/bin/python3

import rospy

from sensor_msgs.msg import CompressedImage

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import omnicv

FRAME_ID = "head_camera"  # Name of the camera frame.

TOPIC_IN = '/camera/image_raw/compressed' # replace with transport option '_image_transport:=compressed'
TOPIC_OUT = 'omnicv/%s/camera/compressed'
FACES = {
    'FRONT': (0,1),
    'RIGHT': (1,2),
    'BACK': (2,3),
    'LEFT': (3,4),
    'UP': (4,5),
    'DOWN': (5,6),
    'CUBE': (0,6)
}

class ImageConverter:
    '''Split equirectangular image into cubemap and publish one face'''

    def __init__(self):
        self.camera = rospy.get_param('~camera', default='FRONT')
        self.side = rospy.get_param('~side', default='256')
        self.side = int(self.side)

        if self.camera.upper() not in list(FACES.keys()):
            rospy.logerr(f'Bad camera: {self.camera}; need on of {str(list(FACES.keys()))}')
            raise BaseException('ROS parameter error')

        self.face_index = FACES[self.camera.upper()]
        self.image_pub = rospy.Publisher(TOPIC_OUT % self.camera.lower(), CompressedImage, queue_size=10)
        self.image_sub = rospy.Subscriber(TOPIC_IN, CompressedImage, self.callback) 
        self.brige = CvBridge()
        self.mapper = omnicv.fisheyeImgConv()

    def callback(self, data):
        try:
            cv_image = self.brige.compressed_imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        cubemap = self.mapper.equirect2cubemap(cv_image, side=self.side, dice=False)
        cv_image_out = cubemap[:, self.face_index[0] * self.side: self.face_index[1] * self.side, :] 
        #cv_image_out = cubemap
        self.image_pub.publish(self.brige.cv2_to_compressed_imgmsg(cv_image_out))


if __name__ == '__main__':
    node_name = "camera_split"
    node_name += "_" + str(np.random.randint(low=0, high=99999999999))
    rospy.init_node(node_name)
    ic = ImageConverter()
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass

    rospy.logwarn("Node `{}` stops.".format(node_name))


