import modules.virtual_serial_port as port
import modules.detect as detect
import modules.square as square
import cv2, json, serial

drone = serial.Serial("/dev/ttys001")
base = serial.Serial("/dev/ttys002")


img = cv2.imread("Images/IMechE_Target/mid.png", -1)
squares = detect.squares(img)
for s in squares:
    toSend = json.dumps(s.to_json())
    drone.write(toSend+"\r\n")
    data = base.readline()
    data = json.loads(data)
    sb = square.square_from_json(data)
    print sb
    sb.draw(img)

cv2.imshow('Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# drone.write("Hello World!\n")
# print base.readline()
