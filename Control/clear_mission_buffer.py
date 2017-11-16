print("Starting")
from dronekit import connect
import time

# Connect
connection_string = '127.0.0.1:14540'
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)
time.sleep(1)
vehicle.wait_ready('autopilot_version')

# Upload empty mission
cmds = vehicle.commands
cmds.clear()
cmds.upload()
print("Clearing")
time.sleep(3)

# Confirm it is empty
cmds.download()
print("Comfirming clear")
print("Number of commands should be 0: %s" % vehicle.commands.count)
time.sleep(1)

# Close and exit
vehicle.close()
print("Completed")
