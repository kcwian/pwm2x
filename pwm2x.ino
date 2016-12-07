// ------------------------------------------------------------------------------------------------------------------------------------------------------------
//  8 Channel PWM to 1 channel PPM converter for RC receivers, using Arduino
// 
//  THis firmware is based on ArduPPM Version v0.9.87 from 
// http://code.google.com/p/ardupilot-mega/source/browse/Tools/ArduPPM/
// 
// ..and has been hacked code to:
//   only support Atmel328 chips ( as found on Arduino Duemilanove or Arduino Uno )  chips 
//   not support any "error" mode/s, just 8 PWM-IN  channels TO one single PPM OUT
//   not support any LED indicators m just PWM-IN, and PPM-OUT
//  Integrated the one library that is used to the sketch, for easy user experience.
//   made it Arduino IDE compatible, so it uses standard bootloader and Serial uploader, like all realy Arduino/s. 
//  make compile-time option to either "hold last good PPM value" or "hold default value/s" in case of 
//   no actual input signal for each channel.   see FAILHOLD and FAILCENTRE in .h file



// David/Buzz Sept 3rd 2012. 
// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// PREPROCESSOR DIRECTIVES
// ------------------------------------------------------------------------------------------------------------------------------------------------------------

#define CONFIGPIN 17    // Arduino pin 17 (PC3) to read config on startup
#define START_CHANNEL 2 // 0 = normal, 2 when using serial connection to mask out interrupts due to serial comms.
#define SERIAL_SBUS 1   // defined: output sbus data (100kbps,8e2)

#include "Arduino.h"
#include "ppm_encoder.h"
#include <util/delay.h>
#include <avr/io.h>


#define	ERROR_THRESHOLD		2									// Number of servo input errors before alerting
#define ERROR_DETECTION_WINDOW	3000 * LOOP_TIMER_10MS			// Detection window for error detection (default to 30s)
#define	ERROR_CONDITION_DELAY	500 * LOOP_TIMER_10MS			// Servo error condition LED delay (LED blinking duration)

#define PASSTHROUGH_MODE_ENABLED	// Comment this line to remove CH8 radio passthrough mode support (hardware failsafe for Arduplane)
#define PASSTHROUGH_CHANNEL		8 * 2	// Channel for passthrough mode selection
#define PASSTHROUGH_CHANNEL_OFF_US		ONE_US * 1600 - PPM_PRE_PULSE	// Passthrough off threshold
#define PASSTHROUGH_CHANNEL_ON_US		ONE_US * 1800 - PPM_PRE_PULSE	// Passthrough on threshold

#define THROTTLE_CHANNEL		3 * 2	// Throttle Channel
#define THROTTLE_CHANNEL_LED_TOGGLE_US		ONE_US * 1200 - PPM_PRE_PULSE	// Throttle Channel Led toggle threshold
#define LED_LOW_BLINKING_RATE	125 * LOOP_TIMER_10MS // Led blink rate for low throttle position (half period)

// Timers

#define TIMER0_10MS		156			// Timer0 ticks for 10 ms duration
#define TIMER1_10MS		20000		// Timer1 ticks for 10 ms duration
#define TIMER2_100MS		1562	// Timer2 ticks for 100 ms duration
#define LOOP_TIMER_10MS	10			// Loop timer ticks for 10 ms duration

// LED Code

#define	SPACE_SHORT_DURATION	40 * LOOP_TIMER_10MS	// Space after short symbol
#define	SPACE_LONG_DURATION	75 * LOOP_TIMER_10MS		// Space after long symbol
#define	SYMBOL_SHORT_DURATION	20 * LOOP_TIMER_10MS	// Short symbol duration
#define	SYMBOL_LONG_DURATION	100 * LOOP_TIMER_10MS	// Long symbol duration
#define	INTER_CODE_DURATION	150 * LOOP_TIMER_10MS		// Inter code duration

#define INTER_CODE		0		// Symbols value for coding
#define SHORT_SYMBOL	1
#define LONG_SYMBOL		2
#define SHORT_SPACE		3
#define LONG_SPACE		4
#define LOOP			5



// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// PPM ENCODER INIT AND AUXILIARY TASKS
// ------------------------------------------------------------------------------------------------------------------------------------------------------------

	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	// LOCAL VARIABLES
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	bool localinit = true; // We are inside init sequence
	bool mux_passthrough = false; // Mux passthrough mode status Flag : passthrough is off
	uint16_t led_acceleration; // Led acceleration based on throttle stick position
	bool servo_error_condition = false;	//	Servo signal error condition
	
	static uint16_t blink_led_timer = 0; 		// Blink led timer
	
	static uint16_t led_code_timer = 0;	// Blink Code Timer
	static uint8_t led_code_symbol = 0;	// Blink Code current symbol

    static uint8_t serconfig = 0;

	
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	// LOCAL FUNCTIONS
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	// ------------------------------------------------------------------------------
	// Led blinking (non blocking) function
	// ------------------------------------------------------------------------------
		
	uint8_t blink_led ( uint16_t half_period )	// ( half_period max = 65 s )
	{
						
		blink_led_timer++;
		
		if ( blink_led_timer < half_period ) // If half period has not been reached
		{
			return 0; // Exit timer function and return 0
		}
		else	// half period reached - LED Toggle
		{
			PPM_PORT ^= ( 1 << PB0 );	// Toggle status LED
			blink_led_timer = 0;	// Blink led timer reset
						
			return 1;	// half period reached - Exit timer function and return 1
		}
	
	}
	
	// ------------------------------------------------------------------------------
	// Led code (non blocking) function
	// ------------------------------------------------------------------------------
	
	void blink_code_led ( uint8_t code )
	{
		
		const uint8_t coding[2][14] = {
		
		// PPM_PASSTROUGH_MODE
		{ INTER_CODE, LONG_SYMBOL, LONG_SPACE, SHORT_SYMBOL, SHORT_SPACE, SHORT_SYMBOL, LOOP }, 
		
		// JETI_MODE
		{ INTER_CODE, LONG_SYMBOL, LONG_SPACE, SHORT_SYMBOL, SHORT_SPACE, SHORT_SYMBOL, SHORT_SPACE, SHORT_SYMBOL,LOOP }
		
		};
		
		led_code_timer++;		
					
						
			switch ( coding [ code - 2 ] [ led_code_symbol ] )
			{
				case INTER_CODE:
				
				if ( led_code_timer < ( INTER_CODE_DURATION ) ) return;
				else PPM_PORT |= ( 1 << PB0 );		// Enable status LED
				break;
				
				case LONG_SYMBOL:	// Long symbol
				
				if ( led_code_timer < ( SYMBOL_LONG_DURATION ) ) return;
				else PPM_PORT &= ~( 1 << PB0 );	// Disable status LED
				break;
				
				case SHORT_SYMBOL:	// Short symbol
								
				if ( led_code_timer < ( SYMBOL_SHORT_DURATION ) ) return;
				else PPM_PORT &= ~( 1 << PB0 );	// Disable status LED
				break;
				
				case SHORT_SPACE:	// Short space
				
				if ( led_code_timer < ( SPACE_SHORT_DURATION ) ) return;
				else PPM_PORT |= ( 1 << PB0 );		// Enable status LED
				break;
				
				case LONG_SPACE:	// Long space
				
				if ( led_code_timer < ( SPACE_LONG_DURATION ) ) return;
				else PPM_PORT |= ( 1 << PB0 );		// Enable status LED
				break;
				
				case LOOP:	// Loop to code start
				led_code_symbol = 0;
				return;
				break;
				
			}
						
		led_code_timer = 0;	// Code led timer reset
		led_code_symbol++;	// Next symbol
		
		return; // LED code function return
		
	}
	
		
	// ------------------------------------------------------------------------------
	// ppm reading helper - interrupt safe and non blocking function
	// ------------------------------------------------------------------------------
	static inline uint16_t ppm_read( uint8_t channel )
	{

        uint8_t _ppm_channel = ( channel << 1 ) + 1;
		uint16_t ppm_tmp = ppm[ _ppm_channel ];
		while( ppm_tmp != ppm[ _ppm_channel ] ) ppm_tmp = ppm[ _ppm_channel ];

		return ppm_tmp;
	}
uint8_t SBUS_Packet_Data[25];
uint8_t SBUS_Failsafe_Active = 0;
uint8_t SBUS_Lost_Frame = 0;

void SBUS_Build_Packet(void)
{
	uint8_t SBUS_Current_Channel = 0;
	uint8_t SBUS_Current_Channel_Bit = 0;
	uint8_t SBUS_Current_Packet_Bit = 0;
	uint8_t SBUS_Packet_Position = 0;

  for(SBUS_Packet_Position = 0; SBUS_Packet_Position < 25; SBUS_Packet_Position++) 
  {
	SBUS_Packet_Data[SBUS_Packet_Position] = 0x00;  //Zero out packet data
  }
  
  SBUS_Current_Packet_Bit = 0;
  SBUS_Packet_Data[0] = 0x0F;  //Start uint8_t
  SBUS_Packet_Position = 1;
  
  for(SBUS_Current_Channel = START_CHANNEL; SBUS_Current_Channel < SERVO_CHANNELS; SBUS_Current_Channel++)
  {
    for(SBUS_Current_Channel_Bit = 0; SBUS_Current_Channel_Bit < 11; SBUS_Current_Channel_Bit++)
    {
      if(SBUS_Current_Packet_Bit > 7)
      {
        SBUS_Current_Packet_Bit = 0;  //If we just set bit 7 in a previous step, reset the packet bit to 0 and
        SBUS_Packet_Position++;       //Move to the next packet uint8_t
      }
      SBUS_Packet_Data[SBUS_Packet_Position] |= (((ppm_read(SBUS_Current_Channel)>>SBUS_Current_Channel_Bit) & 0x01)<<SBUS_Current_Packet_Bit);  //Downshift the channel data bit, then upshift it to set the packet data uint8_t
      SBUS_Current_Packet_Bit++;
    }
  }
/*
  if(SBUS_Channel_Data[16] > 1023) SBUS_Packet_Data[23] |= (1<<0);  //Any number above 1023 will set the digital servo bit
  if(SBUS_Channel_Data[17] > 1023) SBUS_Packet_Data[23] |= (1<<1);
  if(SBUS_Lost_Frame != 0) SBUS_Packet_Data[23] |= (1<<2);          //Any number above 0 will set the lost frame and failsafe bits
  if(SBUS_Failsafe_Active != 0) SBUS_Packet_Data[23] |= (1<<3);
*/
  SBUS_Packet_Data[24] = 0x00;  //End uint8_t
}

	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	// INITIALISATION CODE
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
    pinMode(CONFIGPIN, INPUT);
  
  	
	// ------------------------------------------------------------------------------	
	// Reset Source checkings
	// ------------------------------------------------------------------------------
	if (MCUSR & 1)	// Power-on Reset
	{
	   MCUSR=0; // Clear MCU Status register
	   // custom code here
	}
	else if (MCUSR & 2)	// External Reset
	{
	   MCUSR=0; // Clear MCU Status register
	   // custom code here
	}
	else if (MCUSR & 4)	// Brown-Out Reset
	{
	   MCUSR=0; // Clear MCU Status register
	   brownout_reset=true;
	}
	else	// Watchdog Reset
	{
	   MCUSR=0; // Clear MCU Status register
	   // custom code here
	}

	
	// ------------------------------------------------------------------------------
	// Servo input and PPM generator init
	// ------------------------------------------------------------------------------
	ppm_encoder_init();

	// ------------------------------------------------------------------------------
	// Outputs init
	// ------------------------------------------------------------------------------
	PPM_DDR |= ( 1 << PB0 );	// Set LED pin (PB0) to output
	PPM_DDR |= ( 1 << PB1 );	// Set MUX pin (PB1) to output
	PPM_DDR |= ( 1 << PPM_OUTPUT_PIN );	// Set PPM pin (PPM_OUTPUT_PIN, OC1B) to output
	
	// ------------------------------------------------------------------------------		
	// Timer0 init (normal mode) used for LED control and custom code
	// ------------------------------------------------------------------------------
	TCCR0A = 0x00;	// Clock source: System Clock
	TCCR0B = 0x05;	// Set 1024x prescaler - Clock value: 15.625 kHz - 16 ms max time
	TCNT0 = 0x00;
	OCR0A = 0x00;		// OC0x outputs: Disconnected
	OCR0B = 0x00;
	TIMSK0 = 0x00;		// Timer 1 interrupt disable
	
	// ------------------------------------------------------------------------------
	// Enable global interrupt
	// ------------------------------------------------------------------------------
	sei();			// Enable Global interrupt flag
	
	// ------------------------------------------------------------------------------
	// Disable radio passthrough (mux chip A/B control)
	// ------------------------------------------------------------------------------
	PPM_PORT |= ( 1 << PB1 );	// Set PIN B1 to disable Radio passthrough (mux)
	

    serconfig = digitalRead(CONFIGPIN);    

    if (serconfig)
    {
        Serial.begin(115200);
    }
    else
    {
        Serial.begin(100000, SERIAL_8E2);
    }
}

void loop() {


	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	// AUXILIARY TASKS
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	PWM_LOOP: // SERVO_PWM_MODE
	while( 1 )
	{
		delay(7);
		
        if (serconfig)
        {
            for (uint8_t i = START_CHANNEL; i < SERVO_CHANNELS; i++)
            {
                Serial.print(ppm_read(i));
                Serial.print(" ");
            }
            Serial.println();
        }
        else
        {
            SBUS_Build_Packet();
	    	Serial.write(SBUS_Packet_Data, 25);
        }

	}	// PWM Loop end

} // main lopo function end
