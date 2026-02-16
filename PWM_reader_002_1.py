'''
Code Version 1:- Read PWM (3.3V Logic) From T12 RX or FC (3.3v Logic)

------------------ Component Used -------------------------
1) raspberry pi pico 2w
2) T12 Tx and Rx
3) Bread Board and some jumper wires
------------------------------------------------------------
------------------ Connections -----------------------------
T12 Rx             Pi Pico 2W
Ch9                GPIO 17 (22 Physical Pin)
GND                GND

Note :- Power On Rx with 5v
T12 Rx             Power Supply
5V                 5v
GND                GND
------------------------------------------------------------

Created On :- 16-feb-2026
Created by :- K Madhu Mohan Chary
For Code Enquiry :- madhu@marutdrones.com

'''
import rp2
from machine import Pin
import time

FREQ = 125_000_000 #Standard Do not change (easy math and be able to use machine.freq() @125MHz)

#------------------- State Machine Code (PIO Assembly Code) ---------------------------
@rp2.asm_pio() #Decorator
def pwm_measure():
    
    wrap_target()
    wait(0, pin, 0)         # Wait for current pulse to end (if any)
    set(x, 0)
    wait(1, pin, 0)         # Wait for new pulse to start (rising edge)
    
    label("loop")
    jmp(x_dec, "next")       
    label("next")
    jmp(pin, "loop")        # Count while pin is high
    
    mov(isr, x)             
    push(noblock)           # Use noblock to prevent the PIO from stalling
    wrap()
#-------------------------------------------------------------------------
    
#----------------- Main Code ---------------------------------------------
# GPIO 17 (Physical Pin 22)
pwm_pin = Pin(17, Pin.IN, Pin.PULL_DOWN)

sm0 = rp2.StateMachine(0, pwm_measure, freq=FREQ, in_base=pwm_pin, jmp_pin=pwm_pin)
sm0.active(1) #Activate State Machine one

# read raw pulse width and convert to actual pulse width 
def get_pulse_width_us():
    # Flush old data if the FIFO is backed up
    while sm0.rx_fifo() > 1: # 1 Ensures to keep latest value and remaining 2nd,3rd and 4th values in fifo are cleared
        sm0.get()
        
    raw = sm0.get() #Only 1st value is available which is the latest pwm value
    counts = (0xFFFFFFFF - raw) & 0xFFFFFFFF
    # (counts * 2 instructions per loop) / (Clock in MHz)
    return (counts * 2) / (FREQ / 1_000_000)

# main loop 
print("Monitoring PWM")
while True:
    if sm0.rx_fifo(): #if fifo has some values
        width = get_pulse_width_us()
        # Filter out impossible noise (e.g. pulses < 10us)
        if width > 10:
            print(f"PWM : {width:.0f}")
    time.sleep(0.1) #100ms 
#------------------------------------------------------------------------------------------
