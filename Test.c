/*
 * File:   Motor_Control_1
 * Author: D00064143
 * Created on 17th November 2016, 15:03
 */

/*************************************************************
    Device Header File (Indirectly pulls in the pic18f4520.h
*************************************************************/
#include <xc.h>
#include "../../../Buttons_Debounce_State_Mch/Buttons_Debounce.X/Buttons_Debounce.h"
#include "../../GitHub/LCD_library/lcdlib_2016.h"
#include <plib/timers.h>
/*************************************************************
 Compiler Directive to tell he compiler what clock freq is being used
*************************************************************/
#define _XTAL_FREQ 19660800
//#define _XTAL_FREQ 19660800

/*************************************************************
 Function Prototypes
*************************************************************/
void configure(void);
void start_sequence(void);
void timer (void);
void delay_1s(void);
void heart_beat(void);
void Window(unsigned char num);
//void PWM (void);

/*************************************************************
 Global Variables
*************************************************************/
unsigned char count = 0;
unsigned char rev1 = 0;
unsigned char rev = 0;
unsigned int capture = 0;
/*************************************************************
 * LCD variables
*************************************************************/
const unsigned char * hello = "HELLO";
const unsigned char * welcome = "WELCOME";
const unsigned char * ed = "Big Ed";
const unsigned char * date = "22/2/16";
const unsigned char * desired = "Desired";
const unsigned char * motor = "Actual";


/****************************************
 Interrupt function
****************************************/
void __interrupt ISR(void)
{
	if (PIR1bits.CCP1IF)	//Interrupt from ccp code here
    {
    PIR1bits.CCP1IF = 0;
    capture = CCPR1L + (CCPR1H * 256);
    TMR1H = 0;
    TMR1L = 0;
	}
}

/************************** MAIN ****************************************/
/************************* CODING ***************************************/
void main(void)
{
    unsigned short long t1clk = 250000;
    unsigned char duty [3]= {30,50,90};
    unsigned char i = 0;
    unsigned char j = 0;
    configure();       // Configuration bits for program
    start_sequence();  // Initial start up sequence
    timer();           // Configuration of timer bits
 
    lcd_clear();   
    lcd_cursor ( 0, 0 ) ;
    lcd_print ( ccp ) ;
    lcd_cursor ( 0, 1 ) ;
    lcd_print ( speed ) ;
    lcd_cursor ( 6, 1 ) ;
    
    while (1)
    {
     heart_beat ();  
      
     for (i = 0; i < 3; i++) 
      {
         CCPR2L = duty [i];
         for (j = 0; j < 50; j++)
         {
            lcd_cursor (8,0);
            lcd_display_int_value(capture);
            if (capture !=0)
                rev1 = t1clk/capture;
                rev = rev1/4;
                capture = 0;
              lcd_cursor (8,1);
                lcd_display_value(rev);
                rev = 0;
            PR2 = 100;
            TRISCbits.RC1 = 0;         
            delay_fifth_s();
         }
    }
  }
}

/************************ FUNCTION ***************************************/
/************************* CODING ***************************************/
/**The following steps configure the CCP module for PWM operation:
1. Establish the PWM period by writing to the PR2 register.
2. Establish the PWM duty cycle by writing to the DCxB9:DCxB0 bits.
3. Make the CCPx pin an output by clearing the appropriate TRIS bit.
4. Establish the TMR2 prescale value and enable Timer2 by writing to T2CON.
5. Configure the CCP module for PWM operation**/
void configure (void)
{
	TRISB = 0x00;
    PORTB = 0x00;
	TRISD = 0x00;
	TRISA = 0x00;
    TRISCbits.TRISC1 = 0; //These pins are configured individually
	TRISCbits.TRISC2 = 1; //as CCP1 is an input and CCP2 an output
    PORTC = 0x00;
        
    PIE1bits.CCP1IE = 1; // enable ccp interrupts
    ADCON1 = 0x0F;
    
    
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
}

// change timer source
void timer (void)
{
 OSCCONbits.IRCF0 = 1;
 OSCCONbits.IRCF1 = 1;
 OSCCONbits.IRCF2 = 1;
 OSCCONbits.SCS1 = 1;   
}

// delay cycle
void delay_1s(void) // 1 sec delay
{
    unsigned char i;
    for (i=0;i<25;i++)
            __delay_ms(40);  //max value is 40 since this depends on the _delay() function which has a max number of cycles
}


// functional heartbeat
void heart_beat(void) // flashes led to indicate program is running
{
       PORTBbits.INT0 = ~PORTBbits.INT0;
        delay_1s();
}

// flash led at start of program and display welcome message
void start_sequence(void) 
{
    PORTA = 0xff;
	delay_1s();
	PORTA = 0x00;
	
    
    delay_1s();
    lcd_start () ;
    lcd_cursor ( 5, 0 ) ;
    lcd_print ( hello ) ;
    delay_1s();
    lcd_clear();
    lcd_cursor ( 4, 0 ) ;
    lcd_print ( ed ) ;
    lcd_cursor ( 4, 1 ) ;
    lcd_print (date);
	delay_1s();
    lcd_clear();
}
