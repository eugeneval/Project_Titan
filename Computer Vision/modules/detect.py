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

def targets(img, readText=False):
    # Edge detection
    modimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    modimg = cv2.GaussianBlur(img, (5, 5), 0)
    modimg = cv2.Canny(img, 100, 200)

    # Find contours
    contImg, contours, _ = cv2.findContours(modimg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

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

    # Read text inside targets
    if readText:
        for t in targets:
            t.readText(img)

    print "Total number of squares: %s" %(len(squares))
    print "Total number of targets: %s" %(len(targets))

    return targets

class Square:
    id = 0

    def __init__(self, c, x, y, w, h):
        self.id = Square.id
        Square.id += 1

        self.contour = c
        self.x = x
        self.y = y
        self.w = w
        self.h = h

        M = cv2.moments(c)
        self.cX = int(M['m10']/M['m00'])
        self.cY = int(M['m01']/M['m00'])

    def __str__(self):
        return "Square %s: (x,y,w,h) = (%s,%s,%s,%s)" % (self.id, self.x, self.y, self.h, self.w)

    def __gt__(self, other):
        return self.w + self.h > other.w + other.h

    def __lt__(self, other):
        return self.w + self.h < other.w + other.h

    def similar(self, other, tolerance=0.05):
        """Used for filtering out squares that are similar to each other."""
        tolerance = tolerance * ((self.w + self.h)/2)
        if - tolerance < self.x - other.x < tolerance and - tolerance < self.y - other.y < tolerance and - tolerance < self.w - other.w < tolerance and - tolerance < self.h - other.h < tolerance:
            return True
        else: return False

    def concentric(self, other, accuracy=0.05):
        """Checks if two squares are concentric.s"""
        accuracy = accuracy * ((self.w + self.h)/2)
        if - accuracy < (self.cX - other.cX) < accuracy and - accuracy < (self.cY - other.cY) < accuracy:
            return True
        else: return False

    def readText(self, img):
        """WARNING: this is a slow operation, taking ~0.3 seconds per square. Do not use if it is not required."""
        roi = img[self.y:self.y+self.h, self.x:self.x+self.w]
        self.text = text(roi)

class Target:
    id = 0

    def __init__(self, s1, s2):
        self.id = Target.id
        Target.id += 1

        if s1 > s2:
            self.outer = s1
            self.inner = s2
        else:
            self.outer = s2
            self.inner = s1
        self.contour = [s1.contour, s2.contour]

        self.cX = (s1.cX + s2.cX)/2
        self.cY = (s1.cY + s2.cY)/2

    def __str__(self):
        if hasattr(self, 'text'):
            return "Target %s: (x,y) = (%s,%s) Text: %s" %(self.id, self.cX, self.cY, self.text)
        else:
            return "Target %s: (x,y) = (%s,%s)" %(self.id, self.cX, self.cY)

    def draw(self, frame):
        cv2.drawContours(frame, self.contour, -1, (0, 255, 0), 3)

        (startX, endX) = (int(self.cX - (self.inner.w * 0.15)), int(self.cX + (self.inner.w * 0.15)))
        (startY, endY) = (int(self.cY - (self.inner.h * 0.15)), int(self.cY + (self.inner.h * 0.15)))
        cv2.line(frame, (startX, self.cY), (endX, self.cY), (0, 255, 0), 3)
        cv2.line(frame, (self.cX, startY), (self.cX, endY), (0, 255, 0), 3)

    def readText(self, img):
        self.text = self.inner.readText(img)


def text(img, preprocess="thresh"):
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
