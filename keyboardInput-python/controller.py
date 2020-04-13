import serial.tools.list_ports as port_list
import serial

import ctypes
import time

"""
Code from sentdex PyGTA https://github.com/Sentdex/pygta5/blob/master/vjoy-testing/directkeys.py
"""

### Simulate input part ###

# Send input to game with ctypes

SendInput = ctypes.windll.user32.SendInput

# Hex var of input

W = 0x11
D = 0x20

# C struct redefinitions 
PUL = ctypes.POINTER(ctypes.c_ulong)
class KeyBdInput(ctypes.Structure):
    _fields_ = [("wVk", ctypes.c_ushort),
                ("wScan", ctypes.c_ushort),
                ("dwFlags", ctypes.c_ulong),
                ("time", ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class HardwareInput(ctypes.Structure):
    _fields_ = [("uMsg", ctypes.c_ulong),
                ("wParamL", ctypes.c_short),
                ("wParamH", ctypes.c_ushort)]

class MouseInput(ctypes.Structure):
    _fields_ = [("dx", ctypes.c_long),
                ("dy", ctypes.c_long),
                ("mouseData", ctypes.c_ulong),
                ("dwFlags", ctypes.c_ulong),
                ("time",ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class Input_I(ctypes.Union):
    _fields_ = [("ki", KeyBdInput),
                 ("mi", MouseInput),
                 ("hi", HardwareInput)]

class Input(ctypes.Structure):
    _fields_ = [("type", ctypes.c_ulong),
                ("ii", Input_I)]

# Actuals Functions

def PressKey(hexKeyCode):
    extra = ctypes.c_ulong(0)
    ii_ = Input_I()
    ii_.ki = KeyBdInput( 0, hexKeyCode, 0x0008, 0, ctypes.pointer(extra) )
    x = Input( ctypes.c_ulong(1), ii_ )
    ctypes.windll.user32.SendInput(1, ctypes.pointer(x), ctypes.sizeof(x))

def ReleaseKey(hexKeyCode):
    extra = ctypes.c_ulong(0)
    ii_ = Input_I()
    ii_.ki = KeyBdInput( 0, hexKeyCode, 0x0008 | 0x0002, 0, ctypes.pointer(extra) )
    x = Input( ctypes.c_ulong(1), ii_ )
    ctypes.windll.user32.SendInput(1, ctypes.pointer(x), ctypes.sizeof(x))


### Microcontroller part ###

# Port listening

assert len(port_list.comports()) > 0, "No serial ports available"

port = port_list.comports()[0].device # grab the first serial port
print('Classify on microcontroller via', port)

device = serial.Serial(port=port, baudrate=115200, timeout=0) 

# Listening loop

while(True):
    resp = device.readline()
    resp_text = resp.decode('ascii').strip().split(';')
    if (resp_text[0] != ''):
        userActivity = resp_text[0][-1]
        print(userActivity)

        # Condition
        if (userActivity == '2'):
            print("Detect user running")
            PressKey(W)

            time.sleep(0.5) # Required for game
            
            ReleaseKey(W)

            #time.sleep(1)

    
