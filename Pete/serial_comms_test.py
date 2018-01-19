import cv2, pickle
import modules.virtual_serial_port as port
import modules.detect as detect

m,s = port.open()
img = cv2.imread("Images/IMechE_Target/mid.png", -1)

targets = detect.targets(img)
for t in targets:
    t.draw(img)
    data = pickle.dump(t, s)
    received = pickle.load(m)
    print received
