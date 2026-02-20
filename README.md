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
- For Reading PWM From Servo tester Refer "Read_PWM_Control_Stepper_Speed_001.py" Circuit in the code

