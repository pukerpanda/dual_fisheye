#! /usr/bin/python3

import math
import os
import sys
import argparse


from cv_bridge import CvBridge, CvBridgeError
import cv2

import omnicv

ROOT = os.path.dirname(os.path.abspath(__file__))+'/'

FACES = {
    'FRONT': (0,1),
    'RIGHT': (1,2),
    'BACK': (2,3),
    'LEFT': (3,4),
    'UP': (4,5),
    'DOWN': (5,6),
    'CUBE': (0,6)
}

def parse_command_line_arguments():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Publish color images from a video file to ROS topic.")
    parser.add_argument("-i", "--input", required=False, type=str,                        
                        help="An input video of images for publishing.")
    parser.add_argument("-o", "--output", required=False, type=str,
                        default='images',
                        help="Output images dir.")
    parser.add_argument("-n", "--num-frames-target", required=False, type=float,
                        default=100,
                        help="How many images to use.")
    parser.add_argument("-s", "--side", required=False, type=float,
                        default=1024,
                        help="Cube side.")

    args = parser.parse_args(sys.argv[1:])                    
    return args

def main(args):
    '''Split video file into images'''
    videopath = args.input    
    directory = args.output
    
    if not os.path.exists(directory):
        os.makedirs(directory)

    cap = cv2.VideoCapture(videopath)
    cb = CvBridge()
    prop_fps = cap.get(cv2.CAP_PROP_FPS)
    prop_num_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    stride = math.ceil(prop_num_frames / args.num_frames_target)

    mapper = omnicv.fisheyeImgConv()
    
    frame_id = 0
    image_seq = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("EOS")
            break

        frame_id += 1

        if (frame_id + 1) % stride:
            continue
        
        cubemap = mapper.equirect2cubemap(frame, side=args.side, dice=False)

        for dir in ['FRONT', 'RIGHT', 'BACK', 'LEFT']:
            face_index = FACES[dir]
            side_image = cubemap[:, face_index[0] * args.side: face_index[1] * args.side, :] 
            cv2.imwrite(f'{directory}/frame_{"%04d"%image_seq}.jpeg', side_image)
            
            image_seq += 1


if __name__ == '__main__':
    node_name = "video_spliter"

    args = parse_command_line_arguments()   
    try:
        main(args)
    except KeyboardInterrupt: pass

    print("Node `{}` stops.".format(node_name))
        
