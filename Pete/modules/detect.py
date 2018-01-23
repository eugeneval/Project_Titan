import cv2
from PIL import Image
import pytesseract
import os
import itertools
from square import Square
from target import Target

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
                s = Square(c, x, y, w, h)
                if s not in squares:
                    squares.append(s)

    # Reject similar squares
    for s1, s2 in itertools.combinations(squares, 2):
        if s1.similar(s2) and s2 in squares:
            squares.remove(s2)

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
    if readText == True:
        for t in targets:
            t.readText(img)

    # print "Total number of squares: %s" %(len(squares))
    # print "Total number of targets: %s" %(len(targets))

    return targets

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
