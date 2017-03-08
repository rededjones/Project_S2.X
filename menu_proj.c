/***************************************************
original code by Tommy Gartlan
edited by Edward Jones
Date last modified:- march 2017
Updated: Update to use xc8 compiler within Mplabx

/****************************************************
		Libraries included
*****************************************************/
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../GitHub/Buttons_Debounce_State_Mch/Buttons_Debounce.X/Buttons_Debounce.h"
#include "../../GitHub/LCD_library/lcdlib_2016.h"

#include <plib/timers.h>
//#include <plib/usart.h>
//#include <plib/can2510.h>

/*************************************************
					Clock
*************************************************/
#define _XTAL_FREQ 19660800


typedef struct 
{
    unsigned char Desired;
    unsigned char Actual;
} Motor;

Motor Motor1;



/************************************************
			Global Variables
*************************************************/
const unsigned char msg_ary[10][16] = {"Desired > ", "Actual  > ", 
                                        "POT Value > ","Press Enter",};

const unsigned char * problem = "Problem";

const unsigned char * startup = "Ready to go";
										  

/************************************************
			Function Prototypes
*************************************************/
void Initial(void);
void Window(unsigned char num);
void delay_s(unsigned char secs);


/************************************************
 Interrupt Function 
*************************************************/
unsigned char count_test =0;
unsigned char TICK_E;
unsigned char sample_time =0;
void __interrupt myIsr(void)
{
    //Timer overflows every 10mS
    // only process timer-triggered interrupts
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        
        Find_Button_Press();       //check the buttons every 10mS
        WriteTimer0(40960); 
        INTCONbits.TMR0IF = 0;  // clear this interrupt condition
        sample_time++;
        if (sample_time > 9)
        {
            TICK_E = 1;
            sample_time =0;
        }
      
        //Heartbeat signal
        count_test++;
        if(count_test == 100){
            PORTCbits.RC7 = ~PORTCbits.RC7;   //check the timer overflow rate
            count_test = 0;                   //Toggle every 1 second (heartbeat))
        }
    }
}


//declare Button
Bit_Mask Button_Press;	

/************************************************
			Macros
*************************************************/
#define MENU_E Button_Press.B0
#define ENTER_E Button_Press.B1
#define UP_E Button_Press.B2
#define DOWN_E Button_Press.B3
#define TURN_ON_ADC ADCON0bits.ADON=1;
#define TURN_OFF_ADC ADCON0bits.ADON=0;
/*****************************************
 			Main Function
******************************************/

void main ( void ) 
{
    unsigned char ADC_Result = 0;
    
    Motor1.Desired=0;
    Motor1.Actual=0;
    typedef  enum {RUN = 0,Update_Desired} states;
    states  my_mch_state = Update_Desired;
    
    Initial();
    lcd_start ();
    lcd_cursor ( 0, 0 ) ;
    lcd_print ( startup ) ;
    
    delay_s(2);
    //Initial LCD Display
    Window(0);
    lcd_cursor ( 12, 0 ) ;
    lcd_display_value(Motor1.Desired);
    lcd_cursor ( 12, 1 ) ;
    lcd_display_value(Motor1.Actual);
    
    while(1)
    {
		//wait for a button Event
		//while(!MENU_E && !ENTER_E && !UP_E && !DOWN_E);  
		while(!Got_Button_E && !TICK_E);
        
		switch(my_mch_state)	
		{
			case RUN: 
				if (MENU_E){
                    my_mch_state = Update_Desired; //state transition
                    Window(1);             //OnEntry action
                    TURN_ON_ADC
                    PIR1bits.ADIF=0; //Clear ADIF bit  
                }
                
				break;
			case Update_Desired: 
				if (MENU_E){
                    my_mch_state = RUN;  //state transition
                    Window(0);              //OnEntry action
                    TURN_OFF_ADC 
                }
				break;
			default: 
				if (MENU_E){
                    my_mch_state = RUN;
                    Window(0);
                }
				break;
		}
			
		switch(my_mch_state)	
		{
			case RUN: 
				lcd_cursor ( 12, 0 ) ;    //state actions
                lcd_display_value(Motor1.Desired);
                lcd_cursor ( 12, 1 ) ;
                lcd_display_value(Motor1.Actual);
                LATC = 0x01;
				
				break;
			case Update_Desired: 
                lcd_cursor ( 12, 0 ) ;
                lcd_display_value(ADC_Result);
                LATC= 0x02;
                PIR1bits.ADIF=0; //Clear ADIF bit
                if (ENTER_E)  
                {        //state actions with guard
                    Motor1.Desired = ADC_Result;
                }
                ADC_Result=ADRESH;
                ADC_Result=ADC_Result>>2;
                if (ADC_Result > 50)
                {
                  ADC_Result=50;  
                }
                
                ADCON0bits.GO_DONE=1;
				//lcd_cursor ( 12, 0 ) ;
                //lcd_display_value(Motor1.Desired);

				break;
			default: 
				lcd_cursor ( 0, 0 ) ;
                lcd_clear();
				lcd_print ( problem );
                LATC = 0x05;
				break;
		}
		
        Button_Press.Full = 0;  //Clear all events since only allowing one button event at a time
        TICK_E=0;                       //which will be dealt with immediately
    
    }
}   


void Initial(void){

	TRISB = 0xFF; //Buttons
	TRISC = 0x00;   //LEDS

	LATC = 0xff;
	delay_s(3);
	LATC = 0x00;
    
    //0n, 16bit, internal clock(Fosc/4), prescale by 2)
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_2);
    WriteTimer0(40960);  //65,536 - 24,576  //overflows every 10mS
    
    ADCON0 =0x00;
    ADCON1 = 0x0E;  //Set An0 to Analog and AN1 to Digital
    ADCON2 = 0x13;
    PIR1bits.ADIF=0; //Clear ADIF bit
    ADCON0bits.ADON=0; //Enable the ADC 
    TRISA = 0x01; //Set Port A as input
    
    ei();
}

void Window(unsigned char num)
{
    lcd_clear();
    lcd_cursor ( 0, 0 ) ;	
	lcd_print ( msg_ary[num*2]);
    lcd_cursor ( 0, 1 ) ;
    lcd_print ( msg_ary[(num*2)+1]);
}


void delay_s(unsigned char secs)
{
    unsigned char i,j;
    for(j=0;j<secs;j++)
    {
        for (i=0;i<25;i++)
                __delay_ms(40);  //max value is 40 since this depends on the _delay() function which has a max number of cycles
        
    }
}

