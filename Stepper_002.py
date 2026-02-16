'''
Code Version 2:- Read PWM (3.3V Logic) From T12 RX or FC (3.3v Logic) With State Machine
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



from machine import Pin
import rp2
import time

# PIN DEFINITIONS
STEP_PIN = 2
DIR_PIN  = 3
ENA_PIN  = 4

# GPIO SETUP
step_pin = Pin(STEP_PIN, Pin.OUT)
dir_pin  = Pin(DIR_PIN, Pin.OUT)
ena_pin  = Pin(ENA_PIN, Pin.OUT)

# Common Anode -> HIGH = Enable
ena_pin.value(1)     # Enable Driver


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def simple_step():

    set(pins, 0)
    nop() [31] # Should be less thean 32
    

    set(pins, 1)
    nop() [31]


# STATE MACHINE
sm0 = rp2.StateMachine(0, simple_step, freq=80000, set_base=step_pin)# PIO clock (2 MHz)
    
sm0.active(1)


'''
nop()[num] ; num in range 1 to 30

REQUIREMENTS
1) Minimum Required micro seconds = 10us
2) Maximum Required micro seconds = 10000us
3) Find the optimal frequency?

----Formula-----------
freq(Hz) = cycle / time(s)         ; 1 cycle = 1 instruction
time(s)  = 1 instruction / freq(Hz) ;
time(us) = (1 instruction / freq(Hz)) * 1000_000

minimum delay in us:-
As we need mimium two instruction "set(pins,0 or 1)" and "nop()[num]" num in range 0 to 31
minimum delay us = (2 instructions / freq(Hz))*1000_000

maximum delay in us:-
Maximum instructions can be 33 in total "set(pins,0 or 1)" + "nop()[31])"  ; 1 + 32 = 33
Maximum delay us = (33 instructions / freq(Hz))*1000_000

------------
freq(Hz)     minimum delay (us)     maximum delay (us)
8000             250                    4125
10000            200                    3300
15000            133.3                  2200
20000            100                    1650
25000            80                     1320 (optimal)
30000            66.6                   1100


80000            25                     412.5

'''