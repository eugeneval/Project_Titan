import cv2

def squares(img):
    # Edge detection
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    img = cv2.Canny(img, 50, 150)

    # Find contours
    contImg, contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
            checkAspectRatio = 0.8 <= aspectRatio <= 1.2

            if checkDimensions and checkSolidity and checkAspectRatio:
                squares.append(c)

    return squares
