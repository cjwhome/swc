//set configuration bits
//summary of registers associated with i2c operation on page 248 of pic12f1840 manual
#include <pic12f1840.h>
#include <xc.h>
#include <stdio.h>
#include <string.h>
#include "putc.c"

#pragma config CPD = OFF, BOREN = OFF, IESO = OFF, FOSC = INTOSC, FCMEN = OFF, MCLRE = ON, WDTE = OFF, CP = OFF, PWRTE = OFF, CLKOUTEN = OFF, PLLEN = ON, LVP = OFF
// Data Memory Code Protection: Data memory code protection is disabled
// Brown-out Reset Enable: Brown-out Reset disabled
// Internal/External Switchover: Internal/External Switchover mode is disabled
// Oscillator Selection: INTOSC oscillator: I/O function on CLKIN pin
// Fail-Safe Clock Monitor Enable: Fail-Safe Clock Monitor is disabled
// MCLR Pin Function Select: ON	MCLR/VPP pin function is MCLR
// Watchdog Timer Enable: WDT disabled
// Flash Program Memory Code Protection: Program memory code protection is disabled
// Power-up Timer Enable: PWRT disabled
// Clock Out Enable: CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin

#define _XTAL_FREQ 16000000
#define MAX_DUTY_CYCLE 1023
#define GAIN_FACTOR 0.001
#define HALF_DUTY 511
#define RCV_STRING_SIZE 14

char rcv_str[RCV_STRING_SIZE] ;
char rcv_char_index;
int got_serial_string;
char error;
char recieve_char;

void get_duty_from_master(void);
//void REC_int(void);
void enable_recieve_interrupt(void);
void read_ADC0(void);
//void putch(char c);

void interrupt REC_int(void)		//interupt to stuff uart recieved bytes into a
{
	if(RCIF){
	//check the framing error status bit
            
            error = RCSTA;
            recieve_char = RCREG;
            //rcv_str[rcv_char_index] = RCREG;
            if(rcv_str[rcv_char_index]==13||rcv_char_index>RCV_STRING_SIZE){		//check if carriage return was sent
                    rcv_str[rcv_char_index]=0;		//null terminate string
                    got_serial_string = 1;
                    rcv_char_index = 0;
            }
            rcv_char_index++;
	}
}

void enable_recieve_interrupt(void){
    RCIE = 1;   //Interrupt Enable bit of the PIE1 register
    PEIE = 1;   //Peripheral Interrupt Enable bit of the INTCON register
    GIE = 1;    //Global Interrupt Enable bit of the INTCON register
    error = 0;
}



void init_pic12f1840(void){
	int i;			//used for counter
        long f;
	char data;
	
	OSCCON |= 0xC0;
        //OSCCON |= 0b10111000;       //oscillator running at 500 kHz MF??
        OSCTUNE = 0x00;
	
        for(f=0;f<10000;f++)        //delay for startup
            _nop();
	//**init i/o pins
	RXDTSEL = 1;
	TXCKSEL = 1;
        //enable_recieve_interrupt();
	
	TRISA4 = 0;		//Set eusart transmit pin as output
	TRISA5 = 1;		//set eusart recieve pin as input
	
	//**init i2c SSP1CON1 register
	SSPEN = 1;		//enable i2c serial port and configure sda and scl pins as the source of the serial port pins
	CKP = 1;		//enable clock or if 0, hold the clock low to ensure data setup time
	SSPM0 = 0;		//set bits 1,2 for i2c slave mode with 7 bit address
	SSPM1 = 1;
	SSPM2 = 1;
	SSPM3 = 0;
	//SEN = 1;
	//SDAHT = 1;
	
	TRISA2 = 1;		//Set SPIDATA pin as input
	TRISA1 = 1;		//Set SPICLK pin as input
	//SSPSTAT register
	//SMP = 1;		//Slew rate control disabled for standard speed mode
	
	//SSP1CON2 register
	//SEN = 1;		//enable clock stretching
	
	SSP1ADD = 0xA0;	//setup the address for the i2c device
	
//	SSP1IE = 1;		//enable the mssp interrupt
//	PEIE = 1;		//enable all active peripheral interrupts
//	GIE = 1;		//enable all active interrupts
	
	
	//PIE1 |= 0x80;	//enable MSSP interrupt
	
	//**init uart	
	//Set the baud rate:
	//Solving for SPBRGH:SPBRGL:
	//X = FOSC/Desired Baud Rate/64 – 1
	//=	16000000/9600/64 – 1
	//= 25.042 = 25
	SPBRGH = 0;
	SPBRGL = 0x19;	//Baud rate 9600
	//SPBRGL = 0x0C;		//Baud rate 19200

	SYNC = 0;		//enable the asynchronous serial port by clearing the SYNC bit (bit 4 of TXSTA REG)
	SPEN = 1;		//set the SPEN bit (bit 7 of RCSTA)
	TXEN = 1;		//enable the transmission by setting the TXEN control bit.  This will cause the TXIF interrupt bit to be set
        //CREN = 1;
	//__delay_ms(1000);//pause for startup

        /*Setup for PWM pg.185 in spec sheet for pic12f1840*/
        //setup pin A5 for PWM output
        APFCON |= 0x01; //alternate pin function control for CCP1 - normally it is pin RA2
        TRISA5 = 1;     //Disable the CCP1 pin output driver by setting the associated TRIS bit.
        PR2 = 0xFF;     //Load the PR2 register with the PWM period value, must be max for 10 bits
                                //PWM Period = [(PR2)+1]*4*TOSC*(TMR2 Prescale Value) where TOSC = 1/FOSC
        CCP1CON = 0x0C;//Configure the CCP1 module for the PWM mode by loading the CCP1CON register with the appropriate values.
                    //bits <7-6>_ are for enhanced pwm mode...
        CCPR1L = 0;//Load the CCPR1L register and the DC1B1 bits of the CCP1CON register, with the PWM duty cycle value.

        //Configure and start Timer2:
        PIR1 &= 0xFD;       // Clear the TMR2IF interrupt flag bit of the PIR1 register. See Note below.
        T2CON |= 0x03;      // pg 167 Configure the T2CKPS bits of the T2CON register with the Timer prescale value.
        T2CON |= 0x04;      // pg 167 Enable the Timer by setting the TMR2ON bit of the T2CON register.

        //Enable PWM output pin:
         //? Wait until the Timer overflows and the TMR2IF bit of the PIR1 register is set. See Note below.
        TRISA5 = 0;//? Enable the CCP1 pin output driver by clearing

        //enable the ADC for A0 on Pin 7
        ADCON0 = 0x01;      //enable channel a0
}

//change the duty cycle of the PIC12f1840 by writing to CCP1CON and CCPR1L
void update_duty_cycle(unsigned int duty){
    unsigned int temp;

    if(duty > MAX_DUTY_CYCLE)
        duty = MAX_DUTY_CYCLE;
    //copy the duty cycle into a temp place and shift right 2 bits to move to ccpr1l
    temp = duty >> 2;
    CCPR1L = temp;         //8 MSb's of the duty cycle are loaded into this reg

    //copy the duty cycle again into the temp place and move into bits <5-4> of ccp1con reg
    temp = duty & 0x03;     //isolate bits 1-0
    temp = temp << 4;       //shift left 4 bits
    CCP1CON |= temp;         //only bits <5-4> are used for the lsb's of the duty cycle
}

void main(void){
	unsigned char address;
	int count, i;
        unsigned char command;
        long a;
        
	init_pic12f1840();
        got_serial_string = 0;
        rcv_char_index = 0;
        
        printf("SWC interface firmware\r\n");

        update_duty_cycle(512);
	//wait for data to come from the master on the i2c line and send it right out the uart when it comes in
	//SSP1IF = 0;
	while(1){			//continue forever
            //printf("Blah ");
            read_ADC0();
            //printf("%1.2f\n\r", read_ADC0());
            recieve_char = getchar();
            printf("got char:%c\n\r", recieve_char);
		//while(!SSP1IF);	//wait for start from i2c to set the SSP1IF interrupt bit
            if(SSP1IF){
		SSP1IF = 0;						//*clear the SSP1IF bit
		address = SSP1BUF;				//*read the recieved address from SSP1BUG to clear the BF flag
		if(SEN)							//*If SEN=1 (BIT 0 of SSP1CON2), set the CKP bit (BIT 4 of SSPCON1) to release the SCL line
			CKP = 1;	
                //printf("address: %b\n\r", address);
		if(address == 0xA0){
                    while(!SSP1IF);	//wait for start from i2c to set the SSP1IF interrupt bit
                    SSP1IF = 0;						//*clear the ssp1IF bit (Bit 3 of PIR1 register)
                    command = SSP1BUF;
                    //if(command == 0x0C)
                        //get_duty_from_master();
    
                }
            }
        }
}
/*
1.Configure Port:
    ?  Disable pin output driver (Refer to the TRIS register)
    ?  Configure pin as analog (Refer to the ANSEL register)
2.Configure the ADC module:
    ?  Select ADC conversion clock
    ?  Configure voltage reference
    ?  Select ADC input channel
    ?  Turn on ADC module
3.Configure ADC interrupt (optional):
    ?  Clear ADC interrupt flag 
    ?  Enable ADC interrupt
    ?  Enable peripheral interrupt
    ?  Enable global interrupt(1)
4.Wait the required acquisition time(2).
5.Start conversion by setting the GO/DONE bit.
6.Wait for ADC conversion to complete by one ofthe following:
    ?  Polling the GO/DONE bit
    ?  Waiting for the ADC interrupt (interrupts enabled)
7.Read ADC Result.
8.Clear the ADC interrupt flag (required if interrupt is enabled).
 */
void read_ADC0(void){
    long i;
    //int readings[30];
    long high_count;
    unsigned int duty_cycle;
    union {
        int Integer;
        char BitMap[2];
    }DoubleWord;
    printf("Reading ADC channel 0\n\r");
    TRISA |= 0x01;      //disable the output on A0
    ANSELA = 0x01;      //set channel A0 to be analog input
    ADCON1 = 0xC0;      //conversion speed is FOSC/2
    duty_cycle = HALF_DUTY;
    update_duty_cycle(duty_cycle);
    while(1){
        high_count = 0;
        for(i=0;i<300000;i++){
            ADCON0bits.GO = 1;          //set the go/done bit to start conversion
            while(ADCON0bits.GO);     //wait for conversion to complete

            DoubleWord.BitMap[1] = ADRESH;
            DoubleWord.BitMap[0] = ADRESL;
            //readings[i] = DoubleWord.Integer;
            if(DoubleWord.Integer > 400)
                high_count++;
            else
                high_count--;
        }
        //duty_cycle -= high_count*GAIN_FACTOR;
        duty_cycle += high_count*GAIN_FACTOR;       //testing if this helps control valve flipping sign?

        if(duty_cycle > MAX_DUTY_CYCLE)
            duty_cycle = MAX_DUTY_CYCLE;
        else if(duty_cycle > 0)
            duty_cycle = 0;
        update_duty_cycle(duty_cycle);
        printf("High count:%ld, duty cycle:%d\n\r", high_count, duty_cycle);
        //for(i=0;i<30;i++)
            //printf("Reading[%d]=%d\n\r", i, readings[i]);
    }
    //for(i=0;i<10;i++){
    //    printf("Reading[%d]=%d\n\r", i, readings[i]);
    //}

}
/*void get_duty_from_master(void){
    unsigned int duty_cycle, temp_int;
    unsigned char duty_cycle_lo;
    unsigned char duty_cycle_hi;

    while(!SSP1IF);	//wait for start from i2c to set the SSP1IF interrupt bit
    SSP1IF = 0;						//*clear the ssp1IF bit (Bit 3 of PIR1 register)
    duty_cycle_lo = SSP1BUF;
    while(!SSP1IF);	//wait for start from i2c to set the SSP1IF interrupt bit
    SSP1IF = 0;						//*clear the ssp1IF bit (Bit 3 of PIR1 register)
    duty_cycle_hi = SSP1BUF;

    //printf("duty_hi = %X,duty_lo=%X\r\n", duty_cycle_hi, duty_cycle_lo);
    duty_cycle = duty_cycle_lo;

    temp_int = duty_cycle_hi;
    temp_int = temp_int << 8;
    temp_int &= 0xFF00;         //make sure lower 8 bits are clear??

    duty_cycle |= temp_int;
   // printf("duty_cycle = %d\r\n", duty_cycle);
    update_duty_cycle(duty_cycle);

}*/


//read
