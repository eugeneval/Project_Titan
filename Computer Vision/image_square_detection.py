# import numpy as np
import argparse
import cv2
import modules.detect as detect

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to image")
ap.add_argument("-s", "--size", help="factor to scale image by")
args = vars(ap.parse_args())
scaleFactor = float(args["size"])

image = cv2.imread(args["image"], -1)
squares = detect.findSquares(image)
cv2.drawContours(image, squares, -1, (0, 255, 0), 3)

image = cv2.resize(image, (0,0), fx=scaleFactor, fy=scaleFactor)
cv2.imshow('Target', image)

cv2.waitKey(0)
cv2.destroyAllWindows()
