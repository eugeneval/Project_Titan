import modules.virtual_serial_port as port
import modules.detect as detect
import modules.target as target
import cv2, json, serial

drone = serial.Serial("/dev/ttys001")
base = serial.Serial("/dev/ttys002")


img = cv2.imread("Images/IMechE_Target/mid.png", -1)
targets = detect.targets(img)
for t in targets:
    toSend = json.dumps(t.to_json())
    drone.write(toSend+"\r\n")
    data = base.readline()
    data = json.loads(data)
    tb = target.target_from_json(data)
    print tb
    tb.draw(img)

cv2.imshow('Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
