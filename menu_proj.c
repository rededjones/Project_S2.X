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
#define _XTAL_FREQ 8000000


typedef struct 
{
    unsigned char Desired;
    unsigned char Actual;
    unsigned char Run;
} Motor;

Motor Motor1;

typedef struct
{
    float kp;
    float ki;
    float kpki;
    signed char ek;
    signed char ek1;
    unsigned char uk;
    unsigned char uk1;
}
controller;

controller control1;

/************************************************
			Global Variables
*************************************************/
const unsigned char msg_ary[10][16] = {"Desired > ", "Actual  > ", 
                                        "POT Value > ","Press Enter",};
const unsigned char * problem = "Problem";
const unsigned char * startup = "Ready to go";
unsigned char count = 0;    // count for ccp
unsigned char rev1 = 0;     // Revolution count
unsigned char rev = 0;     // Revolution count
unsigned char Actual = 0;      // Calc value for speed
unsigned int capture;	// captured value 									  

/************************************************
			Function Prototypes
*************************************************/
void Initial(void);
void Window(unsigned char num);
void delay_s(unsigned char secs);
void timer (void);
void controller_funct(void);


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
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) 
    {
        WriteTimer0(45536); 
        INTCONbits.TMR0IF = 0;  // clear this interrupt condition
        Find_Button_Press();       //check the buttons every 10mS
        
        sample_time++;
        if (sample_time > 9)
        {
            TICK_E = 1;
            sample_time =0;
            controller_funct();
            CCPR2L = control1.uk;
             
        }
      
        //Heartbeat signal
        count_test++;
        if(count_test == 100){
            PORTEbits.AN7 = ~PORTEbits.AN7;   //check the timer overflow rate
            count_test = 0; 
            
             //Toggle every 1 second (heartbeat))
        }
    }
    if (PIE1bits.CCP1IE && PIR1bits.CCP1IF)	//Interrupt from ccp code here
    {
        PIR1bits.CCP1IF = 0;
        capture = CCPR1L + (CCPR1H * 256);

        TMR1H = 0;
        TMR1L = 0;
	}
}

//declare Button
Bit_Mask Button_Press;	

/************************************************
			Macros
*************************************************/
#define MENU_E Button_Press.B0
#define ENTER_E Button_Press.B1
#define TURN_ON_ADC ADCON0bits.ADON=1;
#define TURN_OFF_ADC ADCON0bits.ADON=0;
/*****************************************
 			Main Function
******************************************/

void main ( void ) 
{
    unsigned char ADC_Result = 0;
    unsigned short long t1clk = 250000;
    
    typedef  enum {RUN = 0,Update_Desired} states;
    states  my_mch_state = Update_Desired;
    
    Initial();
    timer();  // Configuration of timer bits
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
    
    PIE1bits.CCP1IE = 1;
    
    ei();
    
    while(1)
    {
		//wait for a button Event 
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
                
                
                if (capture !=0)
                {
                    Motor1.Actual = t1clk/capture;
                    capture = 0;
                    
                }
                lcd_display_value(Motor1.Actual);
                
                LATEbits.LATE0 = 1;
                
            
				break;
			case Update_Desired: 
                
                LATEbits.LATE1 = 1;;
                PIR1bits.ADIF=0; //Clear ADIF bit
                if (ENTER_E)  
                {        //state actions with guard
                    Motor1.Desired = ADC_Result;
                }
                ADC_Result=ADRESH;
                ADC_Result=ADC_Result>>2;
                if (ADC_Result > 50)
                    ADC_Result=50;  
                lcd_cursor ( 12, 0 ) ;
                lcd_display_value(ADC_Result);          
                ADCON0bits.GO_DONE=1;
				//lcd_cursor ( 12, 0 ) ;
                //lcd_display_value(Motor1.Desired);

				break;
			default: 
				lcd_cursor ( 0, 0 ) ;
                lcd_clear();
				lcd_print ( problem );
                //LATE = 0x05;
				break;
		}
		
        Button_Press.Full = 0;  //Clear all events since only allowing one button event at a time
        TICK_E=0;                       //which will be dealt with immediately
        }
    }


void Initial(void)
{

	TRISB = 0xFF; //Buttons
	TRISE = 0x00;   //LEDS
    TRISCbits.RC1 = 0; //ccp input
	    
    //0n, 16bit, internal clock(Fosc/4), prescale by 2)
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);
    WriteTimer0(45536);  //65,536 - 20,000  //overflows every 10mS
    
    ADCON0 =0x00;
    ADCON1 = 0x0E;  //Set An0 to Analog and AN1 to Digital
    ADCON2 = 0x13;
    PIR1bits.ADIF=0; //Clear ADIF bit
    ADCON0bits.ADON=0; //Enable the ADC 
    
    
       /* timer configure bits bits.     */ 
    T1CONbits.TMR1ON =1;    // timer on 
    T1CONbits.RD16 = 1;     // 16 bit config
    T1CONbits.TMR1CS = 0;     // timer clock source, int inst clk
    T1CONbits.T1CKPS0 = 1;      // prescaler
    T1CONbits.T1CKPS1 = 1;      // prescaler set to 8
    
    CCP1CONbits.CCP1M0 = 1; // 1101 = Capture mode, every rising edge
    CCP1CONbits.CCP1M1 = 0;
    CCP1CONbits.CCP1M2 = 1;
    CCP1CONbits.CCP1M3 = 0;
    
    // PWM CONFIG BITS
    T2CONbits.TMR2ON =1;    // timer 2 on
    CCP2CONbits.CCP2M3 = 1; //turns on pwm
    CCP2CONbits.CCP2M2 = 1;
    PR2 = 100;
    
//    /* interrupt configure bits bits.     */ 
    RCONbits.IPEN = 0;      // interrupt priority
    INTCONbits.GIE = 1;     // enable interrupts
//    INTCONbits.GIEL = 1;    // high priority interrupts
//    INTCONbits.TMR0IE = 1;  // timer 0 overflow interrupt enabled
    INTCONbits.PEIE = 1;    // enable peripheral interrupts
//    INTCONbits.INT0IE = 1;  // enable ext interrupt 0
//    INTCON3bits.INT1IE = 1; // enable ext interrupt 1
//    INTCON2bits.TMR0IP = 0; // timer 0 interrupt priority
//    INTCON2bits.INTEDG0 = 1; // edge declaration
//    INTCON2bits.INTEDG1 = 1;// edge declaration
    
    /* controller constants*/
    control1.kp = 0.4;
    control1.ki = 0.3;
    control1.kpki = 0.7;
    control1.ek = 0;
    control1.ek1 = 0;
    control1.uk = 30;
    control1.uk1 = 30;
    Motor1.Desired=20;
    Motor1.Actual=0;
            
}

// change timer source
void timer (void)
{

 OSCCONbits.IRCF0 = 1;
 OSCCONbits.IRCF1 = 1;
 OSCCONbits.IRCF2 = 1;
 OSCCONbits.SCS1 = 1;   
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
void controller_funct (void)
{
    float temp = 0;
    control1.ek1 = control1.ek;
    control1.ek = Motor1.Desired - Motor1.Actual;
    control1.uk1 = control1.uk;
    temp = ((control1.kpki * control1.ek )- (control1.kp * control1.ek1));
    control1.uk = control1.uk1 + temp;
}