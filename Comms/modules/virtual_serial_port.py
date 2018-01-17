import os, pty, serial

def createVirtualPort():
    master, slave = pty.openpty()
    m = MasterPort(master)
    s = SlavePort(slave)
    return m, s


class MasterPort:
    def __init__(self, pty):
        self.pty = pty

    def write(self, str):
        str = str + "\r\n"
        os.write(self.pty, str)

    def read(self):
        return os.read(self.pty,1000)

class SlavePort:
    def __init__(self, pty):
        self.pty = pty
        self.name = os.ttyname(pty)
        self.port = serial.Serial(self.name)

    def write(self, str):
        self.port.write(str)

    def read(self):
        return self.port.readlaine()
