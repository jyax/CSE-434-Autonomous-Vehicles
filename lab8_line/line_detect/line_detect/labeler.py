#!/usr/bin/env python
'''
Simple Image Labeler

Enables user to specify one or more targets with closed polygons, and converts this to a mask image

Usage:  python labeler.py image_name

Note: output mask is saved as <image_name>_mask.png, and is a 3-channel image.  To use it
you will likely need to select just one channel.

Daniel Morris, Jan 2020
'''
import numpy as np
import os
import cv2
import argparse

class PolygonMaker(object):
    def __init__(self, window_name):
        self.window_name = window_name  # Name for our window
        self.done_poylgon = False  # Flag signalling we're done current polygon
        self.done_all = False  # Flag signalling we're done all polygons
        self.current = (0, 0)  # Current position, so we can draw the line-in-progress
        self.points = []  # List of points defining our polygon

    def on_mouse(self, event, x, y, buttons, user_param):
        # Mouse callback that gets called for every mouse event (i.e. moving, clicking, etc.)
        if self.done_polygon:  # Nothing more to do
            return
        if event == cv2.EVENT_MOUSEMOVE:
            # We want to be able to draw the line-in-progress, so update current mouse position
            self.current = (x, y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            # Left click means adding a point at current position to the list of points
            self.points.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN:
            # Right click means we're done
            self.done_polygon = True

    def run(self, img, outName):
        # Let's create our working window and set a mouse callback to handle events
        cv2.namedWindow(self.window_name, cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)
        cv2.setMouseCallback(self.window_name, self.on_mouse)
        print('Resize window as needed')
        print('Use left mouse button to create a polygon around target region, and right button to close it')
        print('Add more regions as needed')

        mask = np.zeros_like(img)
        green = mask.copy()
        green[:,:,1] = 128

        while not self.done_all:
            self.done_polygon = False
            while not self.done_polygon:
                # This is our drawing loop, we just continuously draw new images
                # and show them in the named window
                canvas = img.copy() * (1-mask) + (img.copy()//2 + green) * mask
                if len(self.points) > 0:
                    # Draw all the current polygon segments
                    cv2.polylines(canvas, np.array([self.points]), False, (255,255,255), 1)
                    # And  also show what the current segment would look like
                    cv2.line(canvas, self.points[-1], self.current, (127,127,127))
                # Update the window
                cv2.imshow(self.window_name, canvas)
                # And wait 50ms before next iteration (this will pump window messages meanwhile)
                if (cv2.waitKey(50) & 0xff) == 27:  # ESC hit
                    self.done = True

            # Create mask from polygon
            cv2.fillPoly(mask, np.array([self.points]), (1,1,1))

            #Show mask overlayed on image for user to validate:
            canvas = img.copy() * (1-mask) + (img.copy()//2 + green) * mask
            # canvas = img.copy() * mask + (1-mask) * img.copy()//2
            cv2.imshow(self.window_name, canvas)

            print("Over image, press 's' to save mask, or 'q' to quit without saving, or any key to add another polygon")
            key = (cv2.waitKey() & 0xff)
            if key == ord('q'):
                self.done_all = True
                print('Quitting without saving')
            elif key ==  ord('s'):
                self.done_all = True
                print('Saving:',outName)
                cv2.imwrite( outName, mask * 255 )
            else:
                self.points = []

        cv2.destroyWindow(self.window_name)

# ============================================================================

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Mask builder')
    parser.add_argument('imgName', type=str, metavar='PATH', help='Full path of image name')
    args = parser.parse_args()

    if os.path.isfile(args.imgName):
        print("Reading in image:", args.imgName)
        img = cv2.imread(args.imgName)
        root, _ = os.path.splitext( args.imgName)    
        outName = root + '_mask.png'
        pd = PolygonMaker("Labeler")  # Initialize labeler
        pd.run(img, outName)          # Run
    else:
        print("Error: unable to find:", args.imgName)

