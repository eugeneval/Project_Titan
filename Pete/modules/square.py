import cv2, json
import detect

class Square:
    id = 0

    def __init__(self, c, x, y, w, h):
        self.id = Square.id
        Square.id += 1

        self.x = x
        self.y = y
        self.w = w
        self.h = h

        self.contour = [c]
        M = cv2.moments(c)
        self.cX = int(M['m10']/M['m00'])
        self.cY = int(M['m01']/M['m00'])

    def __str__(self):
        return "Square %s: (x,y,w,h) = (%s,%s,%s,%s)" % (self.id, self.x, self.y, self.h, self.w)

    def __gt__(self, other):
        return self.w + self.h > other.w + other.h

    def __lt__(self, other):
        return self.w + self.h < other.w + other.h

    def to_json(self):
        data = {'id': self.id, 'x': self.x, 'y': self.y, 'w': self.w, 'h': self.h, 'cX': self.cX, 'cY': self.cY}
        return json.dumps(data)

    def draw(self, frame):
        cv2.drawContours(frame, self.contour, -1, (0, 255, 0), 3)

    def similar(self, other, tolerance=0.05):
        """Used for filtering out squares that are similar to each other."""
        tolerance = tolerance * ((self.w + self.h)/2)
        if - tolerance < self.x - other.x < tolerance and - tolerance < self.y - other.y < tolerance and - tolerance < self.w - other.w < tolerance and - tolerance < self.h - other.h < tolerance:
            return True
        else: return False

    def concentric(self, other, accuracy=0.05):
        """Checks if two squares are concentric."""
        accuracy = accuracy * ((self.w + self.h)/2)
        if - accuracy < (self.cX - other.cX) < accuracy and - accuracy < (self.cY - other.cY) < accuracy:
            return True
        else: return False

    def readText(self, img):
        """WARNING: this is a slow operation, taking ~0.3 seconds per square, due to file creation/deletion. Do not use if it is not required."""
        roi = img[self.y:self.y+self.h, self.x:self.x+self.w]
        self.text = detect.text(roi)

class Square2(Square):
    def __init__(self, id, x, y, w, h, cX, cY):
        self.id = id
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.cX = cX
        self.cY = cY

    def draw(self, img):
        cv2.rectangle(img, (self.x, self.y), (self.x+self.w, self.y+self.h), (0, 255, 0), 3)

def square_from_json(data):
    data = json.loads(data)
    return Square2(data['id'], data['x'], data['y'], data['w'], data['h'], data['cX'], data['cY'])
