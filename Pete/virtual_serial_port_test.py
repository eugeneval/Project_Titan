import modules.virtual_serial_port as port

m, s = port.open()
s.write("Hello World")
print m.read()
m.write("Goodbye")
print s.read()
