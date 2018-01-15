import cv2
from PIL import Image
import pytesseract
import os
import itertools

def squares(img):
    # Edge detection
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    img = cv2.Canny(img, 100, 200)

    # Find contours
    contImg, contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    squares = []
    for c in contours:
        contour_length = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * contour_length, True)

        if 4 <= len(approx) <= 6:
            (x, y, w, h) = cv2.boundingRect(approx)
            aspectRatio = w / float(h)

            area = cv2.contourArea(c)
            hullArea = cv2.contourArea(cv2.convexHull(c))
            solidity = area / float(hullArea)

            checkDimensions = w > 25 and h > 25
            checkSolidity = solidity > 0.9
            checkAspectRatio = 0.5 <= aspectRatio <= 1.5

            if checkDimensions and checkSolidity and checkAspectRatio:
                squares.append(c)

    print "Number of contours: %s" % len(contours)
    print "Total number of squares: %s" % len(squares)
    return squares

def targets(img):
    # Edge detection
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    img = cv2.Canny(img, 100, 200)

    # Find contours
    contImg, contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # Find squares
    squares = []
    for c in contours:
        contour_length = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * contour_length, True)

        if 4 <= len(approx) <= 6:
            (x, y, w, h) = cv2.boundingRect(approx)
            aspectRatio = w / float(h)

            area = cv2.contourArea(c)
            hullArea = cv2.contourArea(cv2.convexHull(c))
            solidity = area / float(hullArea)

            checkDimensions = w > 25 and h > 25
            checkSolidity = solidity > 0.95
            checkAspectRatio = 0.5 <= aspectRatio <= 1.5

            if checkDimensions and checkSolidity and checkAspectRatio:
                s = Square(c, x, y, w, h)
                if s not in squares:
                    squares.append(s)
                    # print "Added %s" %s

    # Reject similar squares
    for s1, s2 in itertools.combinations(squares, 2):
        if s1.similar(s2) and s2 in squares:
            squares.remove(s2)
            # print "Removed %s" %s2

    # Find targets
    targets = []
    for s1, s2 in itertools.combinations(squares, 2):
        if s1.concentric(s2):
            t = Target(s1, s2)
            if t not in targets:
                targets.append(t)
                # print "Found %s" %t

    print "Total number of squares: %s" %(len(squares))
    print "Total number of targets: %s" %(len(targets))
    return targets

class Square:

        def __init__(self, c, x, y, w, h):
            self.contour = c
            self.x = x
            self.y = y
            self.w = w
            self.h = h

            M = cv2.moments(c)
            self.cX = int(M['m10']/M['m00'])
            self.cY = int(M['m01']/M['m00'])

        def __str__(self):
            return "Square: (x,y,w,h) = (%s,%s,%s,%s)" % (self.x, self.y, self.h, self.w)

        def similar(self, other, tolerance=0.05):
            tolerance = tolerance * ((self.w + self.h)/2)
            if - tolerance < self.x - other.x < tolerance and - tolerance < self.y - other.y < tolerance and - tolerance < self.w - other.w < tolerance and - tolerance < self.h - other.h < tolerance:
                return True
            else: return False

        def concentric(self, other, accuracy=0.05):
            accuracy = accuracy * ((self.w + self.h)/2)
            if - accuracy < (self.cX - other.cX) < accuracy and - accuracy < (self.cY - other.cY) < accuracy:
                return True
            else: return False

class Target:

    def __init__(self, s1, s2):
        self.s1 = s1
        self.s2 = s2
        self.contour = [s1.contour, s2.contour]

        self.cX = (s1.cX + s2.cX)/2
        self.cY = (s1.cY + s2.cY)/2
        self.h = s1.h
        self.w = s1.w

    def __str__(self):
        return "Target: (x,y) = (%s,%s)" %(self.cX, self.cY)

    def draw(self, frame):
        cv2.drawContours(frame, self.contour, -1, (0, 255, 0), 3)

        (startX, endX) = (int(self.cX - (self.w * 0.15)), int(self.cX + (self.w * 0.15)))
        (startY, endY) = (int(self.cY - (self.h * 0.15)), int(self.cY + (self.h * 0.15)))
        cv2.line(frame, (startX, self.cY), (endX, self.cY), (0, 255, 0), 3)
        cv2.line(frame, (self.cX, startY), (self.cX, endY), (0, 255, 0), 3)


def text(img, preprocess):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if preprocess == "thresh":
    	img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    elif preprocess == "blur":
    	img = cv2.medianBlur(img, 3)

    filename = "{}.png".format(os.getpid())
    cv2.imwrite(filename, img)

    text = pytesseract.image_to_string(Image.open(filename))
    os.remove(filename)
    return(text)
