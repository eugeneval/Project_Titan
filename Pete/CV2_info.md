# Functions



## General

**cv2.waitKey(howlong)**
Will wait for a keyboard event for the specified time in ms (forever if 0)

**cv2.destroyAllWindows()**

**cv2.destroyWindow(window name)**



## Images

**cv2.imread('file path', colour)**
*colour:* 1 for colour, 0 for grayscale, -1 for as is

**cv2.imshow('window name', img)**



## Video

**cv2.VideoCapture(from where)**
*from where:* either a file, or a camera (0 is the first connected camera, 1 is the second etc)

**read()**
Use to get a single frame (and then manipulate it as an image)



## Drawing

Common arguments:

* *img*
* *colour:* RGB or grayscale
* *thickness* default is 1. -1 fills shape
* *lineType* default is 8-connected

**cv2.line(img, start XY, end XY, colour, thickness)**

**cv2.rectangle(img, start XY, end XY, colour, thickness)**

**cv2.circle(img, centre XY, radius, colour, thickness)**

**cv2.ellipse(img, centre XY, (major axis length, minor axis length), angle, startAngle, endAngle, colour, thickness)**

**cv2.polylines(img, [pts], connected, colour)**
*[pts]:* an array of XY coordinates.
*connected* true to connect in order, false to connect all points to all other points.

**cv2.putText(img, text, pos XY, font, font size, colour, thickness)**
