#PWM to PPM & SBUS converter
## originated from buzz's 8 channel PWM to PPM converter for RC use

With sbus code from https://www.rcgroups.com/forums/showthread.php?2420473-Arduino-S-BUS-Packet-Generator-Code

# start with:
* an Arduino with a Atmega328 or 328p chip on it. 
* IDE to program the chip with this code.

How:?
* Used Arduino IDE to program this firmware onto the Arduino chip.
* Connect upto 8 RC PWM input signals so that the wires go to:
     red = 5v
     black = GND or 0V pin on arduino
     white = PWM signal pins, these connect to D8,D9,D2,D3,D4,D5,D6,D7

* Connect the PPM output so that the wires go to:
     red = 5v
     black = GND or 0V
     PPM out = D10 

Done! 

TIPS:  
* any channel that you don't connect a PWM input on, will emit a default value ( which is 1500 for all channels excpt throttle, which is 1100.
* disconnecting any channel after booting will cause the system to use the last-known-position of the input, until a good signal returns.
* having pin PC3 (Arduino A3) pulled high on startup produces human readable serial 115200,8n1 output of the channel data instead of sbus
* This is not a "failsafe" unit, if has no failsafe functionality.
* If this code causes your expensive toys to crash, or worse, it's your fault for being a bad person, so you should be nicer, as karma's a bitch. 
* Not my fault.  none of it.   I love you too, now go away.
