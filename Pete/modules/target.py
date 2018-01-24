import cv2, json
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
        data =  {'id': self.id, 'id1': self.inner.id, 'x1': self.inner.x, 'y1': self.inner.y, 'w1': self.inner.w, 'h1': self.inner.h, 'cX1': self.inner.cX, 'cY1': self.inner.cY, 'id2': self.outer.id, 'x2': self.outer.x, 'y2': self.outer.y, 'w2': self.outer.w, 'h2': self.outer.h, 'cX2': self.outer.cX, 'cY2': self.outer.cY}
        return json.dumps(data)

    def draw(self, frame):
        self.inner.draw(frame)
        self.outer.draw(frame)

        (startX, endX) = (int(self.cX - (self.inner.w * 0.15)), int(self.cX + (self.inner.w * 0.15)))
        (startY, endY) = (int(self.cY - (self.inner.h * 0.15)), int(self.cY + (self.inner.h * 0.15)))
        cv2.line(frame, (startX, self.cY), (endX, self.cY), (0, 255, 0), 3)
        cv2.line(frame, (self.cX, startY), (self.cX, endY), (0, 255, 0), 3)

    def readText(self, img):
        self.text = self.inner.readText(img)

    def calculate_offset(self, x, y):
        self.offX = self.cX - x
        self.offY = self.cY - y
        return self.offX, self.offY

class Target2(Target):
    def __init__(self, id, inner, outer):
        self.id = id
        self.inner = inner
        self.outer = outer

        self.cX = (inner.cX + outer.cX)/2
        self.cY = (inner.cY + outer.cY)/2


def target_from_json(data):
    data = json.loads(data)
    inner = Square2(data['id1'], data['x1'], data['y1'], data['w1'], data['h1'], data['cX1'], data['cY1'])
    outer = Square2(data['id2'], data['x2'], data['y2'], data['w2'], data['h2'], data['cX2'], data['cY2'])
    return Target2(data['id'], inner, outer)
