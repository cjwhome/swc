
#define _XTAL_FREQ 16000000 	
void putch(char data){
	while(!TXIF)
	continue;
	//__delay_ms(1);
	TXREG = data;
}