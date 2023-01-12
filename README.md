# HaskoFlight
Flight controller based on the Atmega328p and MPU6050

This project doesn't really make sense using hardware older than 10 years. I just wanted to challenge myself to code a good performing flight controller and test out different control methods and see how far I can take it with this limited (but still powerful) hardware. I am using the Atmegas periferials (interrupts, pwm generation, etc.) as much as possible to achieve better performance and lower code size.
## Features
- completely syncronised loop to IMU output including ESC signal output
- hardware generated Oneshot125 ESC protocol
- ESC calibartion
- interrupt based cppm (ppm sum) input from RC reciever
- rate and angle flight modes with in-flight switching
- attitude from MPU6050 digital motion processor (DMP)
- 200hz loop rate (DMP limitation)
- interrupt based I2C handling (thanks @prunkdump)
- PID knob tuning using a spare RC channel
- this is more a hardware thing, but OTA debugging and firmware update through bluetooth
- no external libraries
