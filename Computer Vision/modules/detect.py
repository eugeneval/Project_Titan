import cv2

def findSquares(img):
    # Edge detection
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    img = cv2.Canny(img, 50, 150)

    # Find contours
    contImg, contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    squares = []
    for c in contours:
        # approximate the contour
        contour_length = cv2.arcLength(c, True)
        c = cv2.approxPolyDP(c, 0.02 * contour_length, True)

        if 4 <= len(c) <= 6 and cv2.contourArea(c) > 1000:
            squares.append(c)

    return squares
