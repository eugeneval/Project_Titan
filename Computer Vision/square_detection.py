# import numpy as np
import argparse
import cv2
import modules.detect as detect

# ARGUMENTS
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to image")
ap.add_argument("-v", "--video", help="path to video (0 to use camera)")
ap.add_argument("-s", "--size", help="factor to scale image by")
args = vars(ap.parse_args())


if not args["size"]:
    scaleFactor = 1
else:
    scaleFactor = float(args["size"])

if not args["image"]:
    image = None
    if not args["video"]:
        print "You must select an image or a video!"
        exit()
    elif args["video"] == "0":
        video = 0
    else:
        video = args["video"]
else:
    image = args["image"]
    video = None

if image != None:
    img = cv2.imread(image, -1)
    squares = detect.squares(img)
    cv2.drawContours(img, squares, -1, (0, 255, 0), 3)

    img = cv2.resize(img, (0,0), fx=scaleFactor, fy=scaleFactor)
    cv2.imshow('Target', img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

elif video != None:
    camera = cv2.VideoCapture(video)

    while True:
        isFrame, img = camera.read()
        if not isFrame:
            break

        squares = detect.squares(img)
        cv2.drawContours(img, squares, -1, (0, 255, 0), 3)
        img = cv2.resize(img, (0,0), fx=scaleFactor, fy=scaleFactor)
        cv2.imshow('Target', img)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()
