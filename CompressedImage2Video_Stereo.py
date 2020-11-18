#!/usr/bin/env python2

from __future__ import division
import rosbag, rospy, numpy as np
import sys, os, cv2, glob
from itertools import izip, repeat
import argparse
import pickle 
from CompressedImage2Video import write_frames, get_info, calc_n_frames, imshow, noshow

# try to find cv_bridge:
try:
    from cv_bridge import CvBridge
except ImportError:
    # assume we are on an older ROS version, and try loading the dummy manifest
    # to see if that fixes the import error
    try:
        import roslib; roslib.load_manifest("bag2video")
        from cv_bridge import CvBridge
    except:
        print ("Could not find ROS package: cv_bridge")
        print ("If ROS version is pre-Groovy, try putting this package in ROS_PACKAGE_PATH")
        sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract and encode video from bag files.')
    parser.add_argument('--outfile', '-o', action='store', default=None,
                        help='Destination of the video file. Defaults to the location of the input file.')
    parser.add_argument('--precision', '-p', action='store', default=1, type=int,
                        help='Precision of variable framerate interpolation. Higher numbers\
                        match the actual framerater better, but result in larger files and slower conversion times.')
    parser.add_argument('--fps', '-fps', action='store', default=15, type=int,
                        help='Frame per second of the video. default 15.')
    parser.add_argument('--viz', '-v', action='store_true', help='Display frames in a GUI window.')
    parser.add_argument('--start', '-s', action='store', default=rospy.Time(0), type=rospy.Time,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--end', '-e', action='store', default=rospy.Time(sys.maxint), type=rospy.Time,
                        help='Rostime representing where to stop in the bag.')
    parser.add_argument('--encoding', choices=('rgb8', 'bgr8', 'mono8'), default='bgr8',
                        help='Encoding of the deserialized image.')

    parser.add_argument('topic_left')
    parser.add_argument('topic_right')
    parser.add_argument('bagfile')

    args = parser.parse_args()

    if not args.viz:
        imshow = noshow

    for bagfile in glob.glob(args.bagfile):
        print (bagfile)

        bag = rosbag.Bag(bagfile, 'r')
        print ('Calculating video properties')

        rate, minrate, maxrate, size, times = get_info(bag, args.topic_left, start_time=args.start, stop_time=args.end)
        print('Topic recorded at rate: ', rate)
        nframes = calc_n_frames(times, args.precision)

        # write left first
        print('Write left video')
        outfile = args.outfile.replace('.avi','_left.avi')

        if rate==minrate==maxrate==0.0:
            print('Video not timestamped, use default fps')
            writer = cv2.VideoWriter(outfile, cv2.VideoWriter_fourcc(*'DIVX'),args.fps,size)#, , size)
        else:
            writer = cv2.VideoWriter(outfile, cv2.VideoWriter_fourcc(*'DIVX'),np.ceil(maxrate*args.precision),size)#, , size)
        write_frames(bag, writer, len(times), topic=args.topic_left, nframes=nframes, start_time=args.start, stop_time=args.end, encoding=args.encoding)

        writer.release()
        
        # then write right
        print('Write right video')
        outfile = args.outfile.replace('.avi','_right.avi')

        if rate==minrate==maxrate==0.0:
            print('Video not timestamped, use default fps')
            writer = cv2.VideoWriter(outfile, cv2.VideoWriter_fourcc(*'DIVX'),args.fps,size)#, , size)
        else:
            writer = cv2.VideoWriter(outfile, cv2.VideoWriter_fourcc(*'DIVX'),np.ceil(maxrate*args.precision),size)#, , size)
        write_frames(bag, writer, len(times), topic=args.topic_right, nframes=nframes, start_time=args.start, stop_time=args.end, encoding=args.encoding)

        writer.release()
        print ('\n')
