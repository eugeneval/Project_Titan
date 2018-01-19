import os, pty, serial

def open():
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

    def read(self, length=1000):
        return os.read(self.pty,length)

class SlavePort:
    def __init__(self, pty):
        self.pty = pty
        self.name = os.ttyname(pty)
        self.port = serial.Serial(self.name)

    def write(self, str):
        self.port.write(str)

    def readline(self):
        return self.port.readline()
