'''
Code Version 1:- Read PWM (3.3V Logic) From T12 RX or FC (3.3v Logic) with python
-------- Components Used --------------
Stepper  = Nema 17 JK42HS60-1684
Gear Box = NO
Driver   = TB6600 
Current Set = 1.5Amp
Micro Step  = 3200
----------------------------------------
-------- Connections -------------------
Pico 2w           Driver 
GPIO 2     ->     pul-
GPIO 3     ->     dir-
GPIO 4     ->     ena-
3.3V       ->     (Short Dir+,Pul+,Ena+)
----------------------------------------

Created On :- 13-feb-2026
Created by :- K Madhu Mohan Chary
For Code Enquiry :- madhu@marutdrones.com

'''

import time
from machine import Pin

# Define the GPIO pins connected to the TB6600
DIR_PIN = Pin(17, Pin.OUT)
STEP_PIN = Pin(16, Pin.OUT)
ENA_PIN = Pin(15, Pin.OUT)
ENA_PIN.value(1) # 0 to Disable and 1 to Enable 

# Constants for direction
CW = 1 # Clockwise
CCW = 0 # Counter-clockwise

steps_per_revolution = 200 # motor and microstepping 
delay = 0.001 # delay for changing speed , decrease the value to increase the speed

def move_stepper(direction, steps):
    DIR_PIN.value(direction) # Set direction
    for _ in range(steps):
        STEP_PIN.value(1) # Step pulse high
        time.sleep(delay)
        STEP_PIN.value(0) # Step pulse low
        time.sleep(delay)

# Move 1 revolution clockwise
move_stepper(CW, steps_per_revolution)
time.sleep(1) # Wait for 1 second

# Move 1 revolution counter-clockwise
move_stepper(CCW, steps_per_revolution)
