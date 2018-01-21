import serial
import modules.virtual_serial_port as port

ser1, ser2 = port.open()

ser1.write('Hello World!\r\n')
print ser2.readline()

ser1.close()
ser2.close()
