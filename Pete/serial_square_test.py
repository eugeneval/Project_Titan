import modules.virtual_serial_port as port
import modules.detect as detect
import modules.square as square
import cv2, serial

drone = serial.Serial("/dev/ttys001")
base = serial.Serial("/dev/ttys002")


img = cv2.imread("Images/IMechE_Target/mid.png", -1)
squares = detect.squares(img)
for s in squares:
    drone.write(s.to_json()+"\r\n")
    data = base.readline()
    sb = square.square_from_json(data)
    print sb
    sb.draw(img)

cv2.imshow('Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# drone.write("Hello World!\n")
# print base.readline()
