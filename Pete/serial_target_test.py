import modules.virtual_serial_port as port
import modules.detect as detect
from modules.detect import Square
import cv2, json, serial

drone = serial.Serial("/dev/ttys001")
base = serial.Serial("/dev/ttys002")


img = cv2.imread("Images/IMechE_Target/mid.png", -1)
squares = detect.squares(img)
for s in squares:
    toSend = json.dumps(s.to_json())
    print toSend
    drone.write(toSend+"\r\n")
    data = base.readline()
    print data

# drone.write("Hello World!\n")
# print base.readline()
