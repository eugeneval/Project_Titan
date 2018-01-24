import cv2, threading, time
import modules.detect as detect
from dronekit import connect
from modules.vehicle import Vehicle as MyVehicle

scaleFactor = 0.6
offset = (0, 0)
offset_lock = threading.Lock()

def findCenter(img):
    (h,w,c) = img.shape
    (x,y) = (w/2,h/2)
    print "Center: (%s,%s)" %(x,y)
    return (x,y)

def process(img, (x,y)):
    # startTime = timeit.default_timer()
    targets = detect.targets(img)
    if not targets: return img, 0, 0
    t = targets[0]
    t.draw(img)
    print "Offset from center: (%s,%s)" % (t.cX - x, t.cY - y)

    # endTime = timeit.default_timer()
    # print "Elapsed time: %s" %(endTime-startTime)
    return img, t.cX - x, t.cY - y

class Fly(threading.Thread):
    def __init__(self, connection_string):
        threading.Thread.__init__(self)

        print("Connecting to vehicle on: %s" % (connection_string,))
        self.vehicle = connect(connection_string, wait_ready=True, vehicle_class=MyVehicle)
        time.sleep(1)
        self.vehicle.wait_ready('autopilot_version')

    def run(self, command, x=0, y=0, h=0):
        if command == "TAKEOFF":
            self.vehicle.arm_and_takeoff(h)
        elif command == "GOTO":
            self.vehicle.goto_relative(x, y, h, wait=False, text=False)
        elif command == "LAND":
            self.vehicle.returnToLand()

camera = cv2.VideoCapture(0)

isFrame, img = camera.read()
center = findCenter(img)

drone = Fly('127.0.0.1:14540')
drone.run('TAKEOFF', h=10)

while True:
    # start = timeit.default_timer()
    isFrame, img = camera.read()
    if not isFrame:
        break

    offX, offY = 0, 0
    img, offX, offY = process(img, center)
    img = cv2.resize(img, (0,0), fx=scaleFactor, fy=scaleFactor)
    cv2.imshow('Target', img)
    drone.run('GOTO', offX/10, offY/10, 0)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    # end = timeit.default_timer()
    # print "Time for this frame: %s" % (end-start)

camera.release()
cv2.destroyAllWindows()
drone.run('LAND')
