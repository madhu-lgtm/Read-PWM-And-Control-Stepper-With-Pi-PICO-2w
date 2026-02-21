## Components Used
- Stepper Motor JK42HS60 -1684
- TB6600 Stepper Driver
- Pi Pico 2w
- T12 Rx and Tx
- Bread Board and some Jumper Wires
- Connection Mentioned in the program
- WARNING! :- Never Connect 5v Logic voltage Input (PWM) to the pi pico 2w , if at all used use voltage divider   
### Generate Pulses From Pi Pico 2w to the TB6600 Stepper Driver
- Use "stepper_001.py" or "stepper_001.py"
- Pulse Generated Cross checked Using DSO
- ![pulse_generation with pico 2w](https://github.com/user-attachments/assets/c627dc39-9500-45c4-bc8c-185bd9433ed3)


### Read PWM From T12 Rx (3.3v) 
- Use "PWM_reader_002_1.py"
- Reading PWM Values From t12 Rx on Ch9
- <img width="1255" height="993" alt="image" src="https://github.com/user-attachments/assets/128309ab-ec41-4d4e-9368-a02825f4fdfe" />

### For a Detailed Work Flow 
- **Version 8:-** Implement cutoff pwm and disable the driver when read pwm widht is under it,
            also only call "sm1.put()" if found significant change in no.of cycles(reduces fluctuation in stepper speed)
  - Refer "Read_PWM_Control_Stepper_Speed_008.py" Circuit in the code
- **Version 7:-** Version 6 got failed to output variable pulse widths for stepper driver, updates "ASM Code For Stepper" For Stepper with
  - Refer "Read_PWM_Control_Stepper_Speed_007.py" Circuit in the code
- **version 6:-** Now make speed dynamic not fixed delay count any more
  - Refer "Read_PWM_Control_Stepper_Speed_006.py" Circuit in the code
- **version 5:-** Now Implement the another state machine for the stapper motor @1MHz with fixed delay (i.e, fixed speed)
  - Refer "Read_PWM_Control_Stepper_Speed_005.py" Circuit in the code
- **Version 4:-** Make a function to calculate step delay cycles(no.of Instruction for delay) for stepper motor
  - Refer "Read_PWM_Control_Stepper_Speed_004.py" Circuit in the code  
- **Version 3:-** Make a function to convert safe_target_rpm to steps required per second (for stepper motor with gearbox)
   - Refer "Read_PWM_Control_Stepper_Speed_003.py" Circuit in the code
- **Version 2:-**  Make a seperate fuction in python for pwm mesurement and map the read pwm values with rpm
  - Refer "Read_PWM_Control_Stepper_Speed_002.py" Circuit in the code 
- **Version 1 :-** Measure the counts(Cycles) for the +width of a PWM Signal using a State Machine @125MHz
             and Compute the corresponding PWM value in Main Python Loop
  - Refer "Read_PWM_Control_Stepper_Speed_001.py" Circuit in the code

