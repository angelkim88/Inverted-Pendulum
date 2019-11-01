/******************************************

Department of System Control Engineering
        College of Engineering
The underGradute School, Hoseo University

Supervised by professor Suk-Won Lee  

ID   : 20071739
Name : Jung-Hwan Kim
Tel  : 010-5594-5988

*******************************************/

#include <mega128.h> 
#include <delay.h>

#define KI 0 
#define KD 0//2 
#define KP 1//10

unsigned char  degree_serial[7]; // motor[5]
int count=0, back_degree=0, front_degree=0, degree=0, direction=0;

volatile float enco_data; // PID GAIN 
volatile float l_con, f_con, error, pre_error, sum_err, diff_err; //PID Calculation
volatile float stand=90; // Standard

/************** Data Send To Computer using Serial Port ******************/
void Putch(char data)   // Data Send To Computer
{
   while(!(UCSR0A & 0x20));
   UDR0=data;    
}
                    
/**************************** Main ***************************************/
void main() 
{
    
    DDRE=0b00011000;    // E PORT is use to interrupt & motor
    //| NC | interrupt Input | NC | Moter_2 | Moter_2 | NC | NC | NC |
    DDRA=0xff;          // LED
    DDRB=0xfe;          // PORTB.0 is Encoder B_Phase & PORTB.5,6 is motor

    EIMSK=0b01000000;   // Interrupt channel 6
    EICRB=0b00100000;   // INT 6  PTE.6
    
    TCCR1A=0b10100001;  // PWM Register
    TCCR1B=0x05;        // 8 Bit 
    TCCR3A=0b10100001;  // Phase Correct PWM
    TCCR3B=0x05;
    TCNT1=0x0000;
    
    TIMSK = 0b00000001; // Over flow Interrupt On
    TCCR0 = 0b00000111; // CLK/1024   
    TCNT0 = 100;        // Start 100

    UCSR0A=0x00;        // Serial Port 0
    UCSR0B=0b00011000;
    UCSR0C=0b00000110;    
    UBRR0H=0;                      
    UBRR0L=103;         // 9600 bps
    
    SREG=0x80;          // All interrupt Enable
    
    while(1)
    {   
    
        Putch(degree_serial[6]);
        Putch(degree_serial[5]); 
        Putch(degree_serial[4]); 
        Putch(degree_serial[3]); 
        Putch(degree_serial[2]);     
        Putch(degree_serial[1]); 
        Putch(degree_serial[0]);
        
        /**************************** PID Program *********************************/

        enco_data = degree; //  Substitution
        
        error = stand - enco_data; // 
        diff_err = error - pre_error;
        // Previous error Gap = This time error - Previous error
        
        sum_err += error;       // Cumulative value of error
        if(error == 0) sum_err = 0;
        // Reset to prevent the accumulation of cumulative.
        
        f_con = KP * (float)error + KI * (float)sum_err + KD * (float)diff_err;
        //f_con = KP*Deviation + KI*Previous Deviation Gap + KI*Accumulated Value
        l_con = ((f_con*255)/90);  //  Substitution   
        
        pre_error = error;      // Previous error = This time error
        
        if(l_con<0) l_con = l_con * (-1); 
        //To change a Negative Number for Positive Number.
        
        if(l_con > 255) l_con = 255; // Overflow Prevent
               

    }
    
}
   
/*************** External Interrupt is Rotary Encoder ********************/    
interrupt [EXT_INT6] void external_int6(void)
{
     
    if(PINB.0==1)       // cw.  pendulum fell behind of the car.
    {
        PORTA.0=1;      // Green LED  ON
        PORTA.1=0;      // Yellow LED OFF
        direction=0x42; // Back -> B
        back_degree=count++;  // Count 3000pulse
        degree=back_degree/4; // 1 degree       왜 8인데 4가 되는지 모르겟다. 알아볼것.
        
    }
    

    else if(PINB.0==0)  //ccw.  pendulum fell in front of the car.
    {
        PORTA.0=0;      // Green LED  OFF
        PORTA.1=1;      // Yellow LED ON
        direction=0x46; // Front -> F
        front_degree=count--;  // Count 3000pulse
        degree=front_degree/4; // 1 degree
        
    }
    
}

/****************** External Interrupt is sampling ***********************/
interrupt [TIM0_OVF] void timer_ovf0(void)
{
    
    PORTA.2 = 0xff;     // It's Test. Never mind.
        
    degree_serial[6] = direction; // It's Direction. F or B
    degree_serial[5] = 0x20;
    degree_serial[4] = ((degree/1000)%10) + '0';                 
    degree_serial[3] = ((degree/100)%10) + '0';
    degree_serial[2] = ((degree/10)%10) + '0';
    degree_serial[1] = ((degree/1)%10) + '0';         
    degree_serial[0] = 0x0D;
    
    if(degree<90) // Front
    {
        
        // Straight
        OCR1AL=l_con; // Right
        OCR1BL=0;
            
        OCR3AL=0;            
        OCR3BL=l_con; // Left
        delay_ms(10);     
        
    }
        
    else if(degree>90) // Back
    {
       
        // Back
        OCR1AL=0; 
        OCR1BL=l_con; // Right
            
        OCR3AL=l_con; // Left            
        OCR3BL=0; 
        delay_ms(10);
    
    }
    
    else if(degree==90) // Back
    {
        // Stop
        OCR1AL=1; 
        OCR1BL=1; // Right
            
        OCR3AL=1; // Left            
        OCR3BL=1; 
        delay_ms(1);
    
    }
        
    PORTA.2 = 0x00;     // It's Test. Never mind.
     
    TCNT0=100;          // Sampling Time is 0.01S
    
}
        



