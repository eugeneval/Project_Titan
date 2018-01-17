import argparse
import cv2
import modules.detect as detect
import timeit

# ARGUMENTS
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str,help="path to image")
ap.add_argument("-v", "--video", type=str, help="path to video (0 to use camera)")
ap.add_argument("-s", "--size", type=float, help="factor to scale image by")
ap.add_argument("-o", "--ocr", action='store_true', help="whether OCR should be run (only works on images)")
ap.add_argument("-p", "--preprocess", type=str, default="thresh", help="type of preprocessing to be done for OCR")
args = vars(ap.parse_args())


if not args["size"]:
    scaleFactor = 1
else:
    scaleFactor = args["size"]

if not args["image"]:
    image = None
    if not args["video"]:
        print "You must select an image or a video!"
        exit()
    elif args["video"] == "0":
        video = 0
    else:
        video = args["video"]
else:
    image = args["image"]
    video = None

def process(img, (x,y), readText):
    startTime = timeit.default_timer()
    targets = detect.targets(img, readText)
    for t in targets:
        t.draw(img)
        print t
        print "Offset from center: (%s,%s)" % (t.cX - x, t.cY - y)

    img = cv2.resize(img, (0,0), fx=scaleFactor, fy=scaleFactor)
    endTime = timeit.default_timer()
    print "Elapsed time: %s" %(endTime-startTime)
    return img

def findCenter(img):
    (h,w,c) = img.shape
    (x,y) = (w/2,h/2)
    print "Center: (%s,%s)" %(x,y)
    return (x,y)

# IMAGE PROCESSING
if image != None:
    img = cv2.imread(image, -1)

    # if args["ocr"] == True:
    #     text = detect.text(img, args["preprocess"])
    #     print text

    center = findCenter(img)
    img = process(img, center, args["ocr"])
    cv2.imshow('Target', img)

    cv2.waitKey(0)

# VIDEO PROCESSING
elif video != None:
    camera = cv2.VideoCapture(video)

    isFrame, img = camera.read()
    center = findCenter(img)
    while True:
        isFrame, img = camera.read()
        if not isFrame:
            break

        img = process(img, center, args["ocr"])
        cv2.imshow('Target', img)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    camera.release()

cv2.destroyAllWindows()
