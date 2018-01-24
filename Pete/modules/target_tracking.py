import cv2
import modules.detect as detect

scaleFactor = 0.6

def track():
    camera = cv2.VideoCapture(0)
    isFrame, img = camera.read()
    (x, y) = findCenter(img)
    while True:
        # start = timeit.default_timer()
        isFrame, img = camera.read()
        if not isFrame:
            break

        targets = detect.targets(img)
        target = targets[0] if targets else None
        if target:
            target.calculate_offset(x, y)
            target.draw(img)
            print target
        cv2.imshow('Target', img)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        # end = timeit.default_timer()
        # print "Time for this frame: %s" % (end-start)

    camera.release()
    cv2.destroyAllWindows()

def findCenter(img):
    (h,w,c) = img.shape
    (x,y) = (w/2,h/2)
    print "Center: (%s,%s)" %(x,y)
    return (x,y)
