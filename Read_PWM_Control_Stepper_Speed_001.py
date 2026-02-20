'''
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

================================ State Machine Code Logic ====================================================================================

-------------- Clamping The High Pulse Width in PWM Signal by a counter x ---------------------------------------------
 1Fe -> First Falling Edge
 1Re -> First Raising Edge
 2Fe -> Second Falling Edge

                          1Fe            1Re  2Fe
PWM Signal Foam :-    |""""|______________|""""|_____________________|""""|___________________
                      wait  x=0      wait  loop
                      ---->1,2----------->3--->4                  
                                

To Capture the Positive width(3.3v) HIGH in PWM Signal
1) Wait For LOW Signal to Start(After 1Fe) i.e, wait(0,pin,0)
2) As Soon as LOW Signal found, Initilize the Counter (x register) to Zero, Which is used for measuring +ve Width of pulse i.e, set(x,0)
3) Wait For the HIGH Signal to Start (After 1Re) i.e, wait(1,pin,0)
4) Start the Counter Decrementer x-- (x_dec) i.e, lable("loop") -> jmp(x_dec,"next") ,till the Signal Stays HIGH (Before 2Fe) 
   i.e,Counter need to stop When Signal Goes LOW , Until then Loop the decrementer x-- i.e, lable("next") -> jmp(pin,"loop")
   Note:- (Decrementer because as Natively we dont have Counter Incrementer, x++)
5) Move the decremented value x (Contains the counts in negative value) to the ISR i.e, mov(isr,x)
6) Finally Push the value of ISR in State machine to Main Python code i.e, use push(noblock)  
-----------------------------------------------------------------------------------------------------------------------
=========================================================================================================================================================


=========================================================== MAIN Python Code Logic ======================================================================

---------------------- Convert the Negative x counts to positive x counts ---------------------------------------------
1) let negative counts of x be "raw" i.e, raw = -x
2) let postive counts be "counts"

This is the Chip level calculation in Hexa decimal (32bit) for getting positive value of x, just remember it for PICO 2w

*****Formula_1********

counts = (0xFFFFFFFF -raw) & 0xFFFFFFFF ; "&" bit Wise AND

**********************
------------------------------------------------------------------------------------------------------------------------

------- Compute Counts With the Frequency(MHz) and No.of Instructions Used in loop For Getting +Width in MicroSeconds(us)-------
1) Number of Instructions Used in loop = 2 ; i.e, jmp(x_dec,"next") and jmp(pin,"loop")
  Note:- lable("next") and lable("loop") are ignored as they dont have to consume instruction cycles
2) Frequency Used 125MHz or 125_000_000 Hz

*****Formula_2********

width(us) = (counts * no.of Instruction in Loop)/Frequency(MHz)
width = (counts * 2)/125

**********************
----------------------------------------------------------------------------------------------------------------------------------
----------- Clearing the rx fifo for only computing the latest value (i.e, Value Pushed by the ISR by PUSH Instruction)--------------
1) rx_fifo can store 4 values or 4 words (each word is of 32bit length)
2) We only need to consider the 1st pushed value and empty the remaining values or words

a) Check if rx fifo has some values /words----> "if sm0.rx_fifo()"
   i.e, if sm0.rx_fifo()   -> will return the number of words or values present in rx fifo
   suppose sm0.rx_fifo() returns value count in range 1 to 4 -> if condition is TRUE
        or sm0.rx_fifo() returns value zero , meaning rx fifo is empty ...
           ... nothining is pused into it or values are cleared by sm0.get()
b) if above "a)" condition is TRUE, Clear the rx fifo except the first value as below
    while sm0.rx_fifo()>1:
        sm0.get()
   So, by the above while condition rx fifo clears extra old values if it trys to hold more than 1 value (i.e, from 2nd word to 4th word)

c) Now when you read again the rx_fifo after above while loop , you always get a single latest value of the raw count (-ve x value)
    raw = sm0.get()
-----------------------------------------------------------------------------------------------------------------------------------------

--------------- Use While loop for main program for contiously reading the PWM values with some delay-------------------
while True:
    #Clearing the rx fifo for only computing the latest value
    #Convert the Negative x counts to positive x counts
    #Compute Counts With the Frequency(MHz) and No.of Instructions Used in loop For Getting +Width in MicroSeconds(us)
    #print the width
    time.sleep(0.1) #100ms 
======================================================================================================================================================

Code Developed On :- 20-Feb-2026
Code Developed By :- K Madhu Mohan Chary
Contact Details   :- madhu@marutdrones.com

'''
from machine import Pin
import rp2 #For Working With State Machines
import time

PWM_READ_FREQ = 125_000_000 # Set to 125MHz For More Precision on reading PWM And its a Compact Mode easy for math
pwm_pin = 17 #GPIO17

@rp2.asm_pio() #This is The Decorator and Function Under This is for State Machine in PIO Assmebly Instructions
def pwm_measure():
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
sm0 = rp2.StateMachine(0, pwm_measure, freq=PWM_READ_FREQ, in_base = pwm_pin, jmp_pin = pwm_pin)

sm0.active(1) # "1" for activating the state machine and "0" for deactivating the state machine

print("Reading PWM Values")
while True:
    if sm0.rx_fifo():#Check if rx fifo has some values (it can hold up to 4 values)
        while sm0.rx_fifo() > 1: #True if rx fifo has more than 1 value; false if it has only 1 
            sm0.get() #Clears 2nd , 3rd and 4th values in rx fifo
        raw = sm0.get() #only returns the 1st value in the rx fifo as remaining are cleared in above while condition
        counts = (0xFFFFFFFF - raw) & 0xFFFFFFFF
        width = (counts * 2)/125 # 2 for no.of instructions(only cycle consumable) and 125 for pwm read frequency in "Mhz" 125_000_000 Hz = 125Mhz 
        print(f"PWM Width :{width:.3f}us")
    time.sleep(0.1)#main loop delay is 100ms


        
        
    


