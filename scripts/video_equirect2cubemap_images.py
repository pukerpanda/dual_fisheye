#! /usr/bin/python3

import math
import os
import sys
import argparse
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
import cv2

import omnicv
from py360convert import e2c

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
    parser.add_argument("-s", "--side", required=False, type=int,
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
    
    if prop_num_frames / 2 <  args.num_frames_target:
        print("Warning: insuficient frames for double time stride")

    stride = math.floor(prop_num_frames / args.num_frames_target)    # double time 
    print(f"Video frames {prop_num_frames}, stride: {stride}, target frames: {args.num_frames_target}")
    mapper = omnicv.fisheyeImgConv()
    
    frame_id = 0
    image_seq = 0
    while True:

        ret1, frame1 = cap.read()
        ret2, frame2 = cap.read()
        if not (ret1 and ret2):
            print("EOS")
            break

        frame_id += 1
        if (frame_id) % stride:
            continue
        print('>> frame/seq', frame_id, image_seq)
        
        #cubemap1 = mapper.equirect2cubemap(frame1, side=args.side, dice=False)
        cubemap1 = e2c(frame1, face_w=args.side, cube_format='horizon') 
        frame2 = np.roll(frame1, int(args.side / 2), axis=1)
        cubemap2 = e2c(frame2, face_w=args.side, cube_format='horizon') 

        # cv2.imwrite(f'{directory}/frame_{"%04d"%image_seq}1.jpg', cubemap1)
        # image_seq += 1
        # cv2.imwrite(f'{directory}/frame_{"%04d"%image_seq}2.jpg', cubemap2)
        # image_seq += 1
        # continue

        for dir in ['FRONT', 'LEFT', 'BACK', 'RIGHT']:
            face_index = FACES[dir]
            
            # frame 1
            side_image1 = cubemap1[:, face_index[0] * args.side: face_index[1] * args.side, :] 
            if dir in ['RIGHT', 'BACK']:
                side_image1 = np.flip(side_image1, axis=1)
            # cv2.imwrite(f'{directory}/frame_{"%04d"%image_seq}.1_{dir}.jpg', side_image1)
            cv2.imwrite(f'{directory}/frame_{"%04d"%image_seq}.jpg', side_image1)
            image_seq += 1

            # frame 2 
            side_image2 = cubemap2[:, face_index[0] * args.side: face_index[1] * args.side, :] 
            if dir in ['RIGHT', 'BACK']:
                side_image2 = np.flip(side_image2, axis=1)
            cv2.imwrite(f'{directory}/frame_{"%04d"%image_seq}.jpg', side_image2)
            image_seq += 1

if __name__ == '__main__':
    node_name = "video_spliter"

    args = parse_command_line_arguments()   
    try:
        main(args)
    except KeyboardInterrupt: pass

    print("Node `{}` stops.".format(node_name))
        
