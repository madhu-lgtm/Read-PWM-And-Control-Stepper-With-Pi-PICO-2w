'''
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
Code Updated On :- 23-Feb-2026
Code Developed By :- K Madhu Mohan Chary
Contact Details   :- madhu@marutdrones.com

'''
from machine import Pin
import rp2 #For Working With State Machines
import time

PWM_READ_FREQ = 125_000_000 # Set to 125MHz For More Precision on reading PWM And its a Compact Mode easy for math
STEPPER_FREQ = 1_000_000
pwm_pin = 17 #GPIO17

MIN_RPM = 10.0 # Tested till 10RPM Dont go Under This 
MAX_RPM = 100.0 # Tested till 400RPM With Gear Box (Don't Go beyond 400RPM)

#MIN_PWM = 1000.0
MAX_PWM = 1951.0 
CUT_OFF_PWM = 1100.0 #Cut OFF PWM For Pixhawk FC = 1100@10%, VKV9 = 1200@0% ,1300@10%, Jiyi = 1000@0% , 1100@10%
MIN_PWM = CUT_OFF_PWM # As i need minimum rpm too


MICRO_STEPS = 3200 # Pulses per Revolution
GEAR_RATIO = 1.0#If their is no gear box keep 1.0
                  #If gearbox is used example:- 1:3.71 gear ratio "GEAR_RATIO = 3.71"

step_pin = Pin(2, Pin.OUT) #GPIO2
dir_pin  = Pin(3, Pin.OUT) #GPIO3
ena_pin  = Pin(4, Pin.OUT) #GPIO4

dir_pin.value(1) #1-> cw, 0->ccw
ena_pin.value(0) #0-> disable driver, 1-> enable driver




#-------------------------------------------- ASM Code For PWM Reading----------------------------------------------
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
#----------------------------------------------------------------------------------------------------------------------------------------------------

#------------------------------------- ASM Code For Stepper -------------------------------------------------------
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW) #initilize the pins to OUT and with LOW Logic
def simple_step():
    pull(block) # wait until we have some data in OSR and pull
    mov(y,osr)
    
    label("loop_start")
    
    # HIGH PULSE
    set(pins, 1)  [5] # 5 for only High because we have used 5 extra instructions in Low Pulse  set(x,0); pull(noblock); mov(x,osr); jmp(not_x,"keep_old"); mov(y,osr)
    mov(x, y)# Move the contents (i.e, No.of cycles) in OSR to Y register              
    label("delay_high")
    jmp(x_dec, "delay_high")
    
    # LOW PULSE
    set(pins, 0)           
    mov(x, y)              
    label("delay_low")
    jmp(x_dec, "delay_low")
    
    #Filter to get update the y register with non zero value
    set(x,0)
    pull(noblock)
    mov(x,osr)
    jmp(not_x,"keep_old")
    mov(y,osr)
    jmp("loop_start")
    
    label("keep_old")
    jmp("loop_start")
    
#initlize the state machine 
sm1 = rp2.StateMachine(1, simple_step, freq=STEPPER_FREQ, set_base=step_pin)

sm1.active(1)
#----------------------------------------------------------------------------------------------------------------------------------------------------

#--------------------------------------------------------- Python Sub Function for Main Code --------------------------------------------------------
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
    return STEPPER_FREQ / steps_per_second
    
#--------------------------------------------------------------------------------------------------------------------------------------------    
def get_compensated_step_delay_cycles(step_delay_cycles)->float:
    return (step_delay_cycles/2)-8
    
    
old_cycle_count = 0   
# MAIN LOOP
while True:
    #------------------------------------------ PWM to step delay logic -------------------------------------------
    width = get_pulse_width_us()
    if width and 800 <= width <= 2200: #as 800 and 2200 are min and max pwm limits that any reciver or fc can output
        #print(f"PWM Width :{width:.3f} us")
        
        if(width < CUT_OFF_PWM):
            ena_pin.value(0) #Disable Stepper
        else:
            ena_pin.value(1) #Enable Stepper
            
        
        safe_target_rpm = get_safe_target_rpm(width)
        #print(f"safe_target_rpm = {safe_target_rpm:.3f} rpm")
        
        steps_per_second = get_steps_per_second_with_gear_box(safe_target_rpm)
        #print(f"steps_per_second = {steps_per_second:.3f}")
        
        step_delay_cycles = get_step_delay_cycles(steps_per_second) 
        #print(f"step_delay_cycles = {step_delay_cycles:.3f}")
        #print(f" for DSO step_delay_cycles = {step_delay_cycles/2:.3f}")
        
        compensated_step_delay_cycles = int(get_compensated_step_delay_cycles(step_delay_cycles))
        #print(f"compensated_step_delay_cycles = {compensated_step_delay_cycles}")
    #----------------------------------------------------------------------------------------------------------------
        new_cycle_count = compensated_step_delay_cycles
        
        if abs(old_cycle_count - new_cycle_count) > 2: #2 is the filter, if difference is more than 2 then only puts the new cout to state machine
            print(f"old_cycle_count = {old_cycle_count}")
            sm1.put(new_cycle_count)
            old_cycle_count = new_cycle_count
            print(f"new_cycle_count = {new_cycle_count}")
            print(f"PWM Width :{width:.3f} us | safe_target_rpm = {safe_target_rpm:.3f} rpm | steps_per_second = {steps_per_second:.3f} | for DSO step_delay_cycles = {step_delay_cycles/2:.3f} |compensated_step_delay_cycles = {compensated_step_delay_cycles}")
            
        
    time.sleep(0.1)#main loop delay is 100ms


        
        
    


