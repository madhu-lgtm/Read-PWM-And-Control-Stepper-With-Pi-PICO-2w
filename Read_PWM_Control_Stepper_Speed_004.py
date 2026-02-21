'''
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
1) Pi Pico 2w
2) Servo Tester 
------------------------------------------------------------------

-------------- Connections ---------------------------------------
Servo Tester                             Pi Pico 2w
Signal Out -> Voltage Divider ->         GPIO17 (Physical Pin 22)
GND                           ->         GND

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

Code Developed On :- 20-Feb-2026
Code Developed By :- K Madhu Mohan Chary
Contact Details   :- madhu@marutdrones.com

'''
from machine import Pin
import rp2 #For Working With State Machines
import time

PWM_READ_FREQ = 125_000_000 # Set to 125MHz For More Precision on reading PWM And its a Compact Mode easy for math
STEPPER_FREQ = 1_000_000
pwm_pin = 17 #GPIO17

MIN_RPM = 10.0
MAX_RPM = 100.0

MIN_PWM = 1000.0
MAX_PWM = 2000.0

MICRO_STEPS = 3200 # Pulses per Revolution
GEAR_RATIO = 1.0  #If their is no gear box keep 1.0
                  #If gearbox is used example:- 1:3.71 gear ratio "GEAR_RATIO = 3.71"


@rp2.asm_pio() #This is The Decorator and Function Under This is for State Machine in PIO Assmebly Instructions
def count_high_pulse_width_of_pwm():
    wrap_target() #All Instructions Below target till wrap() acts like a continous loop; But if any instructions above targer are only executed once  
    wait(0,pin,0) # wait(HIGH or LOW, source , pin index) ; source can be directly a GPIO Number or pin 
    set(x,0)      #set X = 0 (Counter , Only decrement it)
    wait(1,pin,0) 
    
    label("loop") #lable is the name given to a particular location in the program , and it does not consume a cycle for execution 
    jmp(x_dec,"next") #This is Conditional Jump Where it decrements the x and jumps to lable loaction "next"
    
    label("next")
    jmp(pin,"loop") #This is also a conditional Jump , it jump backs to Lable "loop" till the "pin" is HIGH (1) , if "pin"becomes LOW(0) below instructions are executed
    
    mov(isr,x) #Copy the decremented x (negative value or raw) to the ISR
    push(noblock) #push the contents in ISR to main python program (recived by sm0.get()) if any;
    #noblock is used because if ther is a vaule in ISR it will push but if their are no contents in ISR it will not wait to until ISR has some contents
    #push(block) or simply push() will wait until there are some contents in ISR and then only push contents to the main python program 
    wrap() #this acts like end of the instruction and calls and executes from wrap_target() again , i.e looping
        

#creating an object "sm0" of state machine we have 12 in total for pi pico 2w
sm0 = rp2.StateMachine(0, count_high_pulse_width_of_pwm, freq=PWM_READ_FREQ, in_base = pwm_pin, jmp_pin = pwm_pin)

sm0.active(1) # "1" for activating the state machine and "0" for deactivating the state machine

print("Reading PWM Values")


def get_pulse_width_us()->float:
    if sm0.rx_fifo():#Check if rx fifo has some values (it can hold up to 4 values)
        while sm0.rx_fifo() > 1: #True if rx fifo has more than 1 value; false if it has only 1 
            sm0.get() #Clears 2nd , 3rd and 4th values in rx fifo
        raw = sm0.get() #only returns the 1st value in the rx fifo as remaining are cleared in above while condition
        counts = (0xFFFFFFFF - raw) & 0xFFFFFFFF 
        return (counts * 2)/125 # 2 for no.of instructions(only cycle consumable) and 125 for pwm read frequency in "Mhz" 125_000_000 Hz = 125Mhz

def get_safe_target_rpm(width)->float:
    slop = (MAX_RPM - MIN_RPM)/(MAX_PWM - MIN_PWM)
    target_rpm = (slop*(width - MIN_PWM)) + MIN_RPM
    safe_target_rpm = min(MAX_RPM,max(target_rpm,MIN_RPM))
    
    return safe_target_rpm

def get_steps_per_second_with_gear_box(safe_target_rpm) -> float:

    safe_target_rps = safe_target_rpm / 60
    return safe_target_rps * MICRO_STEPS * GEAR_RATIO

def get_step_delay_cycles(steps_per_second):
    '''
    step_delay_cycles is the ratio of frequency at which the stepper state machine is operating to...
    ...the frequency(i.e, SPS steps per second) computed for particular target RPM.
    
    First set the frequency of the stepper motor (other state machine)
    STEPPER_FREQ = 1_000_000
    
    step_delay_cycles =  STEPPER_FREQ / steps_per_second
    
    Note :- step_delay_cycles are the number of instructions or cycles to be executed by the stepper state machine to get...
    ... that particular delay for a corresponding RPM
    
    '''
    return STEPPER_FREQ / steps_per_second
    
    
    
    
    
# MAIN LOOP
while True:
    width = get_pulse_width_us()
    if width and 800 <= width <= 2200: #as 800 and 2200 are min and max pwm limits that any reciver or fc can output
        print(f"PWM Width :{width:.3f} us")
        
        safe_target_rpm = get_safe_target_rpm(width)
        print(f"safe_target_rpm = {safe_target_rpm:.3f} rpm")
        
        steps_per_second = get_steps_per_second_with_gear_box(safe_target_rpm)
        print(f"steps_per_second = {steps_per_second:.3f}")
        
        step_delay_cycles = get_step_delay_cycles(steps_per_second)
        print(f"step_delay_cycles = {step_delay_cycles:.3f}")
        
    time.sleep(0.1)#main loop delay is 100ms


        
        
    


