import serial

def open():
    """
    Run the following command in the command line before opening the ports:
    'socat -d -d pty,raw,echo=0 pty,raw,echo=0'
    Make sure to kill socat when you are done
    """
    ser1 = serial.Serial('/dev/ttys001')
    ser2 = serial.Serial('/dev/ttys002')
    return ser1, ser2
