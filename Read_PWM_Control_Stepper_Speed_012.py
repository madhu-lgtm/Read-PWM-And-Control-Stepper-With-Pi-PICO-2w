'''
Version 12:- Failed to update ver11
Version 11:- In ver10 Failed to implement steps 
Version 10:- Added Second Stepper Motor
Version 9:- Read Second Source of PWM for Pipe Bending and opening
Version 8_2:- Updated max RPM to 80 
Version 8_1:- Changed Stepper Motor From Nema 17 42HS60-1684 to Nema 17 42HS48-1684 With Gear Box 1:3.71
                and Change PWM Values Of Cube FC (Updated Connections For Fc)
Version 8:- Implement cutoff pwm and disable the driver when read pwm widht is under it,
            also only call "sm1.put()" if found significant change in no.of cycles(reduces fluctuation in stepper speed)
Version 7:- Version 6 got failed to output variable pulse widths for stepper driver, updates "ASM Code For Stepper" For Stepper with  
version 6:- Now make speed dynamic not fixed delay count any more
version 5:- Now Implement the another state machine for the stapper motor @1MHz with fixed delay (i.e, fixed speed)
Version 4:- Make a function to calculate step delay cycles(no.of Instruction for delay) for stepper motor  
Version 3:- Make a function to convert safe_target_rpm to steps required per second (for stepper motor with gearbox)
Version 2:-  Make a seperate fuction in python for pwm mesurement and map the read pwm values with rpm 
Version 1 :- Measure the counts(Cycles) for the +width of a PWM Signal using a State Machine @125MHz
             and Compute the corresponding PWM value in Main Python Loop  

--------------- Problem Statement / Requirements -----------------
1) Need To eliminate Usage of Multiple Micro Controllers i.e, One Arduino Nano For Reading PWM From FC
   One More Nano For Generating Pulses For Stepper Driver 
2) Need a System Which can Read PWM values accurately and Can Generate the Pulses in Real time For sending it to Stepper Driver
3) The Micro Controller (PI PICO 2w) Should Compute the Output Shaft RPM (Seed Picker Stepper For DSR) Based on the PWM Read
   i.e, Speed Control Of Stepper based on FC PWM.
------------------------------------------------------------------
--------------- Algoritham ---------------------------------------
1) Read PWM From Drone Flight Controller (Required Non Blocking Code) - Use One of PIO State Machine For Reading PWM
2) Compute the RPM Based on PWM - Write Helper Functions For Calculations
3) Generate The Pulses Required For the Stepper Driver based on the Required RPM calculated above
------------------------------------------------------------------

--------------- Components ---------------------------------------
1) Controller    = Pi Pico 2w
2) PWM Generator = Servo Tester
3) Stepper       = Nema 17 42HS48-1684 (GB 1:3.71); Purchase Link :- https://robokits.co.in/motors/stepper-motor/stepper-motor-with-gearbox/nema17-planetary-geared-stepper-motor-14kgcm
4) Gear Box      = 1:3.71
5) Driver        = TB6600 
6) Current Set   = 1.5Amp (S1 = 1, S2 = 1, S3 = 0)
7) Micro Step    = 3200  (S3 = 0, S4 = 0, S6 = 1)
8) BEC(12v to 5v)= Ready To Sky 5v Out BEC (12V Input)
9) Buck(12V 8Amp)= Buck Convertor (17-55v input and 12V 8Amp output)
10)FC            = Cube Orange FC 

------------------------------------------------------------------

-------------- Connections ---------------------------------------
Select the PWM Source (Servo tester Or Cube Orange Flight Controller Or T12 Rx)
Note :- Voltage Divider is Only Needed For Servo Tester(As it Gives 5v Logic PWM)

************************ if Used Servo tester **********************
Servo Tester                             Pi Pico 2w
Signal Out -> Voltage Divider ->         GPIO17 (Physical Pin 22)
GND                           ->         GND
********************************************************************
****************** if Used Cube Orange FC **************************
Cube FC                      Pi Pico 2w
Aux1 Sig FC         ->       GPIO17 (Phy Pin 22)
GND                 ->       GND    (Phy Pin 23)
********************************************************************

Pi Pico 2w                   Driver 
GPIO 2 (Phy Pin4)     ->      pul-
GPIO 3 (Phy Pin5)     ->      dir-
GPIO 4 (Phy Pin6)     ->      ena-
3.3V   (Phy Pin36)    ->      (Short Dir+,Pul+,Ena+)

Pi Pico 2w                   5V Bec
VSYS (Phy Pin39)     ->       5v Out
GND  (Phy Pin38)     ->        Gnd




Note1 :- Never Give 5v PWM logic Voltage to Pi Pico 2w GPIO pins Directly, as pi pico 2w input logic voltage is only 3.3v
Note2 :- Power ON the Servo Tester With Seperate 5v as Pi Pico 2w does Not Output 5V
Note3 :- Use 10k and 20k Resistor for Voltage Divider (10k should be on 5v logic PWM input Side and 20k Should be on GND Side)


->Voltage Divider Connections
5V Logic PWM From Servo Tester ----> 10K Resistor -------> 20K Resistor ------> Common GND (Between Servo Tester and PI PICO 2W)
                                                    |
                                                    |
                                                   \|/
                                                  GPIO17 of Pi PICO 2w
-------------------------------------------------------------------
----------------------------- TESTING ON CUBE FC ON BENCH AUX1 PARAMETERS---------------------------------
FC Firmware :- Arducopter V4.6.0
FC :- Cube Orange Only

SERVO9_FUNCTION = 134 for Min PWM , 135 for mid PWM, 136 for Max PWM
SERVO9_MAX      = 1951
SERVO9_TRIM     = 1501
SERVO9_MIN      = 1051

Note :- Above Params are Only For Testing On Bench With FC , Hench Not For Flight test
-------------------------------------------------------------------------------------------------

Code Developed On :- 20-Feb-2026
Code Updated On :- 03-Mar-2026
Code Developed By :- K Madhu Mohan Chary
Contact Details   :- madhu@marutdrones.com

'''
'''
Version 13:- Fixed PIO empty-label crash and corrected ENA pin logic
'''
from machine import Pin
import rp2
import time

PWM_READ_FREQ = 125_000_000 # 125MHz
STEPPER_FREQ = 1_000_000    # 1MHz

# --- Pins ---
pwm_pin = 17            # Main Stepper PWM
pwm_pin_pipe = 21       # Pipe Folding PWM

step_pin = Pin(2, Pin.OUT)
dir_pin  = Pin(3, Pin.OUT)
ena_pin  = Pin(4, Pin.OUT)

step_pin_pipe = Pin(5, Pin.OUT)
dir_pin_pipe = Pin(6, Pin.OUT)
ena_pin_pipe = Pin(7, Pin.OUT)

# --- Initial Pin States ---
dir_pin.value(1) # 1-> cw, 0->ccw
ena_pin.value(0) # 0-> disable driver, 1-> enable driver

dir_pin_pipe.value(1)
ena_pin_pipe.value(1) # 0 -> ENABLE for TB6600 common cathode wiring

# --- Constants & Settings ---
MIN_RPM = 10.0
MAX_RPM = 80 
MAX_PWM = 1951.0 
CUT_OFF_PWM = 1100.0 
MIN_PWM = CUT_OFF_PWM 
MICRO_STEPS = 3200 
GEAR_RATIO = 3.71

# Pipe folding states
STATE_UNKNOWN = 0
STATE_FOLDED = 1
STATE_UNFOLDED = 2
STATE_MID = 3

current_pipe_state = STATE_UNKNOWN
STEPS_TO_MOVE = 50

# Slower delay for testing (2000 cycles at 1MHz = 2ms delay = 4ms per step = 250 steps/sec)
PIPE_STEP_DELAY_CYCLES = 2000 

# ==============================================================================
#                        PIO BLOCK 0 (State Machines 0 & 1)
# ==============================================================================

# ---------------- ASM PWM Reader (Reusable) ----------------
@rp2.asm_pio() 
def count_high_pulse_width_of_pwm():
    wrap_target() 
    wait(0, pin, 0) 
    set(x, 0)      
    wait(1, pin, 0) 
    
    label("loop") 
    jmp(x_dec, "next") 
    
    label("next")
    jmp(pin, "loop") 
    
    mov(isr, x) 
    push(noblock) 
    wrap() 

sm0 = rp2.StateMachine(0, count_high_pulse_width_of_pwm, freq=PWM_READ_FREQ, in_base=Pin(pwm_pin), jmp_pin=Pin(pwm_pin))
sm0.active(1)

# ---------------- ASM Main Stepper (Speed Control) ----------------
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def simple_step():
    pull(block)
    mov(y, osr)
    
    label("loop_start")
    set(pins, 1) [5] 
    mov(x, y)             
    label("delay_high")
    jmp(x_dec, "delay_high")
    
    set(pins, 0)           
    mov(x, y)             
    label("delay_low")
    jmp(x_dec, "delay_low")
    
    set(x, 0)
    pull(noblock)
    mov(x, osr)
    jmp(not_x, "keep_old")
    mov(y, osr)
    jmp("loop_start")
    
    label("keep_old")
    jmp("loop_start")

sm1 = rp2.StateMachine(1, simple_step, freq=STEPPER_FREQ, set_base=step_pin)
sm1.active(1)


# ==============================================================================
#                        PIO BLOCK 1 (State Machines 4 & 5)
# ==============================================================================

# Read Pipe Folding PWM
sm4 = rp2.StateMachine(4, count_high_pulse_width_of_pwm, freq=PWM_READ_FREQ, in_base=Pin(pwm_pin_pipe), jmp_pin=Pin(pwm_pin_pipe))
sm4.active(1)

# ---------------- ASM Position Stepper (Step Counting) FIXED ----------------
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def position_step():
    label("wait_cmd")     # <-- Explicit label to jump back to
    pull(block)           # Wait for speed delay
    mov(y, osr)           # Store delay in Y
    pull(block)           # Wait for step count
    mov(isr, osr)         # Store step count in ISR
    
    label("step_loop")
    mov(x, isr)           # Load step count to X
    jmp(not_x, "wait_cmd")# If steps == 0, go back up and wait for next command!
    
    # Decrement step count and save back to ISR
    jmp(x_dec, "do_step") 
    label("do_step")
    mov(isr, x)

    # HIGH PULSE
    set(pins, 1) [5]
    mov(x, y)             
    label("delay_high")
    jmp(x_dec, "delay_high")
    
    # LOW PULSE
    set(pins, 0)           
    mov(x, y)             
    label("delay_low")
    jmp(x_dec, "delay_low")
    
    jmp("step_loop")      # Repeat until steps reach 0


sm5 = rp2.StateMachine(5, position_step, freq=STEPPER_FREQ, set_base=step_pin_pipe)
sm5.active(1)


# ==============================================================================
#                             PYTHON LOGIC
# ==============================================================================

def get_pulse_width_us():
    if sm0.rx_fifo():
        while sm0.rx_fifo() > 1:
            sm0.get()
        raw = sm0.get()
        counts = (0xFFFFFFFF - raw) & 0xFFFFFFFF 
        return (counts * 2) / 125
    return 0
    
def get_pulse_width_us_pipe_folding():
    if sm4.rx_fifo():
        while sm4.rx_fifo() > 1:
            sm4.get()
        raw = sm4.get()
        counts = (0xFFFFFFFF - raw) & 0xFFFFFFFF 
        return (counts * 2) / 125
    return 0

def get_safe_target_rpm(width):
    slop = (MAX_RPM - MIN_RPM) / (MAX_PWM - MIN_PWM)
    target_rpm = (slop * (width - MIN_PWM)) + MIN_RPM
    return min(MAX_RPM, max(target_rpm, MIN_RPM))

def get_steps_per_second_with_gear_box(safe_target_rpm):
    safe_target_rps = safe_target_rpm / 60
    return safe_target_rps * MICRO_STEPS * GEAR_RATIO

def get_step_delay_cycles(steps_per_second):
    return STEPPER_FREQ / steps_per_second

def get_compensated_step_delay_cycles(step_delay_cycles):
    return (step_delay_cycles / 2) - 8
    
old_cycle_count = 0 

# MAIN LOOP
while True:
    # --- 1. Main Stepper Logic ---
    width = get_pulse_width_us()
    if width and 800 <= width <= 2200:
        if width < CUT_OFF_PWM:
            ena_pin.value(0) 
        else:
            ena_pin.value(1) 
            safe_target_rpm = get_safe_target_rpm(width)
            steps_per_second = get_steps_per_second_with_gear_box(safe_target_rpm)
            step_delay_cycles = get_step_delay_cycles(steps_per_second) 
            compensated_step_delay_cycles = int(get_compensated_step_delay_cycles(step_delay_cycles))
            new_cycle_count = compensated_step_delay_cycles
            
            if abs(old_cycle_count - new_cycle_count) > 2: 
                sm1.put(new_cycle_count)
                old_cycle_count = new_cycle_count

    # --- 2. Pipe Folding Stepper Logic ---
    width_pipe = get_pulse_width_us_pipe_folding()
    
    if width_pipe:
        # FOLD (~1051us) -> 50 Steps CCW
        if 1000 <= width_pipe <= 1100:
            if current_pipe_state != STATE_FOLDED:
                dir_pin_pipe.value(0) # CCW
                sm5.put(PIPE_STEP_DELAY_CYCLES) # Push speed first
                sm5.put(STEPS_TO_MOVE)          # Push step count second
                current_pipe_state = STATE_FOLDED
                print(f"Pipe: Folding (CCW) - PWM: {width_pipe:.1f}")

        # UNFOLD (~1951us) -> 50 Steps CW
        elif 1900 <= width_pipe <= 2000:
            if current_pipe_state != STATE_UNFOLDED:
                dir_pin_pipe.value(1) # CW
                sm5.put(PIPE_STEP_DELAY_CYCLES) # Push speed first
                sm5.put(STEPS_TO_MOVE)          # Push step count second
                current_pipe_state = STATE_UNFOLDED
                print(f"Pipe: Unfolding (CW) - PWM: {width_pipe:.1f}")

        # MID/STOP (~1501us) -> Stop moving
        elif 1450 <= width_pipe <= 1550:
            if current_pipe_state != STATE_MID:
                # To stop, push 0 steps
                sm5.put(PIPE_STEP_DELAY_CYCLES)
                sm5.put(0) 
                current_pipe_state = STATE_MID
                print(f"Pipe: Stopped - PWM: {width_pipe:.1f}")

    time.sleep(0.05)