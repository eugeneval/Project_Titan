import cv2
from square import Square2

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

    def to_json(self):
        return {'id': self.id, 'id1': self.s1.id, 'x1': self.s1.x, 'y1': self.s1.y, 'w1': self.s1.w, 'h1': self.s1.h, 'cX1': self.s1.cX, 'cY1': self.s1.cY, 'id2': self.s2.id, 'x2': self.s2.x, 'y2': self.s2.y, 'w2': self.s2.w, 'h2': self.s2.h, 'cX2': self.s2.cX, 'cY2': self.s2.cY}

    def draw(self, frame):
        cv2.drawContours(frame, self.contour, -1, (0, 255, 0), 3)

        (startX, endX) = (int(self.cX - (self.inner.w * 0.15)), int(self.cX + (self.inner.w * 0.15)))
        (startY, endY) = (int(self.cY - (self.inner.h * 0.15)), int(self.cY + (self.inner.h * 0.15)))
        cv2.line(frame, (startX, self.cY), (endX, self.cY), (0, 255, 0), 3)
        cv2.line(frame, (self.cX, startY), (self.cX, endY), (0, 255, 0), 3)

    def readText(self, img):
        self.text = self.inner.readText(img)

class Target2(Target):
    def __init__(self, id, s1, s2):
        self.id = id
        self.s1 = s1
        self.s2 = s2

        self.cX = (s1.cX + s2.cX)/2
        self.cY = (s1.cY + s2.cY)/2

def target_from_json(json):
    s1 = Square2(json['id1'], json['x1'], json['y1'], json['w1'], json['h1'], json['cX1'], json['cY1'])
    s2 = Square2(json['id2'], json['x2'], json['y2'], json['w2'], json['h2'], json['cX2'], json['cY2'])
    return Target2(json['id'], s1, s2)
