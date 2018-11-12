#!/usr/bin/env python
from __future__ import print_function

import rospy
import rospkg

import sys
import os
import argparse

import cv2

# Global parameters
x = 220 # 160 # 200
y = 300 # 280 # 200
width = 400 # 400 # 320
height = 220 # 240 # 320

# Function for processing a video strean frame by frame
def processVideo(path, fname):

    # Open the video stream
    cap = cv2.VideoCapture(path + '/videos/' + fname)
    if not cap.isOpened():
        print('[Video] Could not open stream: ' + path + '/videos/' + fname)
        return

    # Save frame counter
    cnt = 1

    # Process the stream
    paused = False
    step = False
    frame = None
    while True:

        # Capture a frame
        if not paused or step:
            ret, frame = cap.read()
            if not ret:
                break
                
            step = False

            # Draw sub-region
            cv2.rectangle( frame, (x,y), (x+width-1,y+height-1), ( 32, 84, 233), 2)
            
        # Grab keyboard input
        key = cv2.waitKey(1) & 0xff
        
        # 'Pause' and 'unpause' the video
        if key == ord('P') or key == ord('p'):
            paused = not paused        
 
        # 'Save' the frame
        if paused and (key == ord('S') or key == ord('s')):
            print("HERE!")

            # Create folder (if it does not exist)
            if not os.path.exists(path + '/images'):
                os.makedirs(path + '/images')
                print('[Video] Created folder: ' + path + '/images')

            # Save the frame
            if frame is not None:            
                output = fname.split('.')[0] + '_' + str(cnt) + '.jpg'
                output = path + '/images/' + output
                print('[Video] Saving frame to: ' + output)
                cropped = frame[y:y+height, x:x+width]
                cv2.imwrite( output, cropped); 
                cnt += 1

        # 'Step' to the next frame
        if key == 32:
            step = True        
        
        # 'Quit'
        if key == 27 or key == ord('Q') or key == ord('q'): 
            break            
        
        # Display the result
        if frame is not None:            
            cv2.imshow('Anchor video frame...',frame)
    
    # Clean up...
    cap.release()
    cv2.destroyAllWindows()



# Main fn
def main():

    # Parse arguments    
    parser = argparse.ArgumentParser(description="Anchor video processing.")
    parser.add_argument('-n', '--name', type=str, required=True)
    args = parser.parse_args()

    # Get the file path for this package
    rospack = rospkg.RosPack()
    path = rospack.get_path('display')

    # Call the video processing function
    processVideo(path, args.name)    

if __name__ == '__main__':
    main()
