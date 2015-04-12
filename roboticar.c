/*
* File		: roboticar.c
* Version	: 1.0
* Last changed	: 12/04/2015
* Purpose	: Control steering system of a robotic car running on HCS12 9SG128 MCU
* Author	: Julius Machira Wamuyu
* Copyright	:©2015
* Product	: Robotic car steering control	
*/
#include <hidef.h>           /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "..\common\delay.h"

// Function Prototypes
void init_PWM(void);
void initPorts(void);
void RTI_Init(void);
void clock(char);
#define Driver 0x01
#define auxCountSensor 0x02   
#define Passenger 0x04 
   
#define Driver_On 0x08
#define Passenger_On 0x02
#define Driver_DIR 0x20
#define Passenger_DIR 0x10


unsigned char flag500ms   =  0;  // A  500 ms timer - Toggles LED PortT7
unsigned int  rti500ms  =  0;  // A 500 ms timer - Creates a 
unsigned char signal = 0;

void main(void) 
{
 unsigned char sensor, old_sensor, lapCount = 0, i;
  initPorts( );	                        
 PORTB |= Driver_DIR;                  // Set driver motor direction
 PORTB &= ~Passenger_DIR;              // Set passenger motor direction
 init_PWM();                        //initialize the Pulse width modulation
 RTI_Init();                            // RTI initialization
 PWME = (Driver_On | Passenger_On);    // Turn both motors on. Channel 1 and 3 (PWM) initially
  EnableInterrupts; 
  for(;;) 
 {
   sensor = PORTA & 0x07;              // bit 0 is for driver, bit 1 for passenger and bit 2 for auxCounter
   
   if (old_sensor != sensor) 
   {
     switch(sensor)  
     {  
    case ( 0x00):
        PTT = 0xFF;
                for ( i = 0 ; i < 20; i++){
                       delay2m();
        }
         if(sensor == 0x00)
            lapCount++ ;
           while(lapCount >=5){
          PWME &= ~(Driver_On | Passenger_On);  // Turn both motors off.
            // Every 250 ms, toggle signal (PortT7 LED)
       	PTT = (6 + signal)<<4;
	      clock(9);
	        }       
                break;
   	      
       
       case ( Driver + Passenger):         // Sensor pattern(101)- move straight.
         PWME = (Driver_On | Passenger_On);// Turn both motors on. Channel 1 and 3       
   	     PTT = (10 + signal)<<4;
	       clock(5); 
   	   break;
       case (Driver + auxCountSensor):  // Driver sensor is on.  
   	    PWME = (Driver_On | Passenger_On); // Turn driver motor on, 
   	    PWMDTY23 = 400;                 //reduce the pwm of passenger
   	    //PWME &= ~Passenger_On; 
   	    PTT = (12 + signal)<<4;
	      clock(3); 
	      //PTT = 0xC0;     
   	   break;
   	     
   	   case (Passenger + auxCountSensor):  // Passenger sensor is on.
         PWME = (Driver_On | Passenger_On);// Turn Passenger motor on, Driver motor off.
         PWMDTY01 = 400; 
         //PWME &= ~Driver_On; 
        PTT = (9 + signal)<<4;
	       clock(6);
	          
   	   break; 
   	    case (Driver):                   // Passenger sensor is on.
         PWME &= ~Passenger_On;  	        // Turn Passenger motor off, Driver motor ON.
         PWME |= Driver_On; 
        PTT = (14 + signal)<<4;
	       clock(1);
	       break;  
	       
	      case (Passenger):                   // Passenger sensor is on.
         PWME &= ~Passenger_On;  	        // Turn Passenger motor off, Driver motor ON.
         PWME |= Driver_On; 
         PTT = (10 + signal)<<4;
	       clock(4);
	       break;    
             case (Passenger|auxCountSensor|Driver):  //all sensors are on.(111)
          clock(7);
          PTT = (8 + signal)<<4;
          
          PWME &= ~(Driver_On | Passenger_On);      // Turn both motors off                                   .               
   	   break;
   	   
   	   default:
   	    PWME &= ~(Driver_On | Passenger_On);    // Turn both motors off
   	    PTT = (15 + signal)<<4;
   	    clock(0);
   	   break;
   	 
     }
        old_sensor = sensor;                // update the old_sensor to be used for next detection
   }
 }
} 
void init_PWM()
// initialize the pulse width modulation
{
 PWMCAE = 0x00;                // left aligned
 PWMCLK = 0x00;                // Ch. 0 - Ch. 1 source is clock A, ch.2 & ch 3 use clock B 
 PWMCLKAB = 0x00;              // Use clock A and B respectively
 PWMPOL = 0x0A;                // initial HIGH output on ch. 1 and 3 (Use odd # registers)
 PWMPRCLK = 0x33;              // Clk A pre-scale = 8 (PWM clock = 6.25MHz/8 = 781.25 KHz =>period =                     1.28uS), so is clock B
 PWMCTL = 0x30;                // CON01 = '1' and Con23 = '1': 16-bit PWM counter, period and duty regs.
 PWMPER01 = 600;               // obtain a period width about 600uS to create 1800Hz PWM
 PWMPER23 = 600;               // obtain a period width about 600uS to create 1800Hz PWM
 PWMDTY01 = 500;               // 83% duty cycle
 PWMDTY23 = 500;               // 83% duty cycle
}                                       
void initPorts( )
{ 
 DDRB |= 0x30;                         // Let PB4, PB5 become OUTPUT
 DDRT = 0xF0;                          // PORT T7-T4 output (LEDs)
 PTT = 0xF0;                           // initialize with LEDs off
 DDRA = 0x00;                           // PORTA as inputs
}  
 //led toggle cycle
void clock(char k){
//k selects the blinking led  
if(flag500ms ==1){ 
          if(signal == k){
        
		        signal= 0; 
         }
	    	 else {
	    	  
		      signal = k; 
      }
	   
	    flag500ms =0;  
}
}
//real time interrupt for clock 
#pragma CODE_SEG NON_BANKED
interrupt VectorNumber_Vrti  void isrRTI(void)
{
   rti500ms++;
   if(rti500ms>=500)
   {
      rti500ms = 0;
      
      flag500ms = 1;
      
   }
   CPMUFLG_RTIF = 1;   // clear RTIF
  }
#pragma CODE_SEG DEFAULT

// RTI initialization for a 1ms clock 
  void RTI_Init(void){
    
   CPMURTI = 0x80 | 0x00; // Mode decimal, divider 1000
   CPMUFLG_RTIF = 1;// Clear RTIF
   CPMUINT_RTIE = 1;// Enable RTI
  }
