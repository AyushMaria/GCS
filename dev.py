import evdev
from select import select

# devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
devices = map(evdev.InputDevice, ('/dev/input/event10', '/dev/input/event11'))
devices = {dev.fd: dev for dev in devices}

for dev in devices.values(): print(dev)

while True:
	r, w, x = select(devices, [], [])
	for fd in r:
		for event in devices[fd].read():
			print(event)
